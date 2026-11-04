# Diagnostic hardware run ? 2026-11-04 16:06

Source: `data/device-monitor-261104-160607.log` (byte-identical original). Decoded:
`data/swingup_diagnostic_2026-11-04_1606.csv`. No firmware edited or uploaded during
this analysis. Boot identifies `swingup-v2.1-diag`; pump caps 100 rad/s? and
4.363323 rad/s; balance caps 60 and pi.

## Completeness and outcome

469 decoded records: 5 M, 199 S, 53 T, 52 R, 50 B, 50 E, 60 N. Event flags retain
simultaneous events: 53 turns, 52 reversals, 51 bottoms, 51 completions. No malformed
records, reported drops, full rotations, BALANCE transition or timing-fault stop.
Pump entered at t=23969 ms and timeout stopped it at t=43968 ms (19.999 s by
integer timestamps). This is a complete bounded failed-capture attempt.

The successful timing/transport outcome applies to this run only, not worst-case
execution time. Event/snapshot data are not uniform 50 Hz samples.

| Measure | Result |
|---|---:|
| Best completed negative tip | -152.41 degrees from down |
| Best completed positive tip | +133.90 degrees from down |
| Closest recorded tip to upright | 27.59 degrees |
| Closest instantaneous snapshot | 28.17 degrees |
| Recorded commanded arm range | -20.11 to +52.20 degrees |
| Late negative/positive median completed tip, t>=30 s | 145.07 / 120.52 degrees |

The peak field records the detector's 200 Hz running extreme, so the best completed
tip can be closer than any transmitted instantaneous snapshot. Do not describe
the snapshot as the full run's closest approach. Recorded arm range is sampled,
not an exhaustive mechanical excursion measurement.

At the closest instantaneous snapshot (t=38269 ms), alpha=-2.650 rad, theta=.198,
rate=-.88 rad/s, arm command=-.397 rad/s, z=5.05 rad/s and raw A request=255.1 rad/s?.
Reject mask19 means angle, z and raw-A gates failed. The best completed-tip record
(t=40925) has peak=-2.660, actual current alpha=-2.647, z=6.49 and raw A319.9. These
are different instants; do not combine the peak angle with the current rate/request.
Widening angle alone would not capture either and a 27.6-degree gate exceeds the
unchanged 25-degree fall cutoff.

## Confirmed detector chatter

At t=24129, 24134 and 24139 ms the detector reports successive turns only5 ms apart,
with peak angles .283, .263 and .280 rad (about15?16 degrees). Energy estimates
are .040, .034 and .039; loss changes .300 -> .830 ->1.090. Measured filtered rate
alternates -1.28, +.77, -2.00 rad/s. No bottom traversal occurs between those turns.
The actual software counted small local sign changes as new half swings and
applied large loss updates to them. Whether the local sign changes come from
sensor noise, physical small reversals, motor vibration or a combination is not
resolved. They are not full energy-pumping half cycles.

There are only two <=50 ms inter-turn gaps in the full trace; this is confirmed
startup chatter, not evidence of continuous near-tip chatter throughout the run.
Loss reaches its1.2 cap on23 turn records. Planned Vp is then frequently near/capped
at4.363. Simply raising the loss limit or target is not supported as the first
change.

## Commanded reversals and centering

For completed reversal pairs after t=30000 ms (R paired with the next E before a
new R), excluding startup:

| Pendulum direction at R | Pairs | Median commanded delta-v | Median delta-v/(2Vp) | Median duration | R target clipping |
|---|---:|---:|---:|---:|---:|
| Positive |17|7.238 rad/s|86.35%|70 ms|0/17|
| Negative |16|7.8075 rad/s|89.47%|75 ms|16/16|

Example positive reversal at43764 ms: Vp4.273, centered raw target4.139,
command before=-3.251; bottom crossing43799; completion43834 with command4.072,
delta-v7.324, versus ideal2Vp8.546. Negative reversal43314: Vp4.363, raw target
-5.204 clipped to-4.363; completion43389, delta-v-7.922 versus ideal-8.726.
The pump model's ideal reversal is not the command actually delivered. Clipping
and centering are direction-dependent; the weaker positive tips are consistent
with that asymmetry, but these data do not isolate its causal contribution from
friction, cable effects or sensor nonlinearity.

These are integrated **commands**, not measured arm velocities. No arm encoder
exists. E uses a moving target and0.05 rad/s tolerance; pairing does not establish
motor impulse or effective mechanical energy transfer. The step generator also
has a low-speed deadband not represented by the integrated velocity alone.

## Work proxies

Summing the quantized bins from completed late turns (t>=30000): lower+=2311,
lower-=2418, upper+=100, upper-=237. These are sums of rate*cos(alpha)*u*dt,
classified by measured rate sign. Upper-half proxy is positive in both directions,
larger in the negative direction. It does NOT support the simple claim that
centering always removes upper-half energy. It does support investigating phase
and directional effects. Sensor-rate distortion, requested rather than measured
acceleration, velocity caps, dry friction and centrifugal terms limit this proxy.

## Next experiment for discussion

Keep A, all capture gates, motor limits, travel guards and timeout. The narrowest
software correction supported directly is to qualify half-cycle/turn updates by
phase traversal (e.g. a bottom traversal between accepted turning events, with
careful first-turn seeding). Separate raw candidate-turn telemetry from accepted
energy updates. Add replay tests for the24129/24134/24139 chatter, startup from
rest, asymmetric full swings, wrap/rotation and cancellation before hardware.
Do not modify capture or centering in the same comparison run.

After a qualified-turn run, compare loss evolution, actual command reversals and
viable near-top states. Phase-dependent pump/centering changes remain a separate
candidate because this run independently exposes delivery asymmetry. Removing
startup chatter is not a promise of successful capture: it may still drive loss
to its cap on legitimate subtarget turns. Do not increase limits to force it.

Await discussion/authorization before changing control: the current user supplied
this diagnostic log, following a discussion-first experiment. Arjarn Bot has the
prior pump-vs-capture review; new findings should be checked with them before a
larger tuning change. Hardware balance A remains the proven fallback.

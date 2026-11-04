# Diagnostics schema (v2.3: same 31-field #D format; mode 4 PARK added)

v2.3 changes for log readers: mode 4 = PARK (after the 20 s pump timeout the arm
returns to commanded 0, then the attempt ends with reason 5). Targets x6/x7 are
now copied from the controller (lastTargetRaw/lastTarget) instead of being
recomputed; they are 0 outside PUMP. In the lower half x6 = +/-vp + centring bias,
where vp <= Vp (x5) is reduced by the bias. Run procedure: `SWINGUP.md`.

# Historical: qualified-turn comparison - v2.2-qualified

## Current override: user-requested18-degree hardware experiment

The existing `swingup` target now uses +/-18 degrees and identifies
`swingup-v2.2-qualified-18` with capture_deg=18.0. Other gates, A, pump/centering,
limits and25-degree fall cutoff are unchanged. This explicitly supersedes the
15-degree instructions below for the present experiment. The earlier18-degree
lossless simulation hit the arm limit; that limitation remains unresolved. No
claim of hardware success or general capture safety. Upload/run remains the user?s
step; x stops. Earlier15-degree results/procedures below remain historical.

This is the first control comparison after the telemetry-only v2.1 experiment.
Only turning-event qualification changes. Motor limits, pump formula/centering,
A gains, 15-degree capture gate, other capture gates, travel, timeout and terminal
stop behavior remain unchanged. `balance/balance.ino` is byte-identical.

## Qualified turn contract

Filtered rate sign changes are raw candidates. The first seed is accepted without
requiring a prior bottom traversal, once an outward-to-inward candidate has at
least 5 degrees amplitude. Smaller seed candidates are logged and rejected.
After a seed, phase must traverse from the accepted tip's side beyond +5 degrees
to the opposite side beyond -5 degrees (or reverse) before another turn can be
accepted. The next candidate must turn inward in that opposite lobe. This is
phase hysteresis, not a time-only debounce. Rejected candidates do not update
energy, learned loss, pump speed, accepted phase history, or reset the tracked
physical directional extreme. The raw candidate detector continues independently.
A full rotation invalidates qualification history and preserves the existing
loss backoff; cancellation is terminal and a new attempt starts fresh.

The uploadable target **still uses +/-15 degrees**. The separate offline +/-18
comparison found an arm-limit failure in a full lossless swing-up case despite
passing a sampled near-center catch-state grid. Do not enable the wider window
for this comparison. See `docs/SWINGUP_QUALIFICATION_2026-11-04.md`.

## Run once

1. VMOT off, arm physically centered, pendulum hanging still. Upload/reset calibrates down.
2. Use the existing target, then open the log monitor:

```powershell
pio run -e swingup -t upload
pio device monitor -f log2file
```

3. Confirm banner **swingup-v2.2-qualified**. VMOT on, `z`, `s`; keep the swept area clear.
   No hand catch. Countdown/pumping/automatic capture are unchanged. `x`/space stops.
4. Send the complete log including startup and the final records, even if it times out.
   Leave the monitor open a second after stopping so pending records can drain.
   Do not start repeated experiments yet. A timing stop is useful diagnostic evidence;
   do not raise its threshold to bypass it.

The test is to identify pump delivery and turning-detector behavior, not to claim
an improved swing-up. The preceding diagnostic hardware run (16:06) reached a recorded tip 27.59 degrees
from upright, never entered BALANCE, and timed out. It confirmed startup turn chatter.

## Output and decoder

The normal 50 Hz CSV is **replaced** by `#D` records (31 fields in v2.2; 30 in archived v2.1). There are 10 Hz snapshots,
near-top snapshots at up to 20 Hz inside 45 degrees of upright, and 200 Hz-detected
turn/reversal/bottom/completion/rotation events. Simultaneous events share one
record and retain their bits. Chatter can create more events than UART capacity.

Four records plus one formatted line are buffered. UART sends at most 16 bytes
per drain call, only when TX capacity is available and the next control deadline
is at least 1 ms away. Control never waits for diagnostic output. On overflow,
an ordinary new record is dropped; state/status records evict the oldest unsent
queued record. A partially transmitted line remains intact. The cumulative drop
counter identifies incomplete evidence. Queue resets do not erase this counter.
`l` toggles ordinary diagnostic records; state/status messages remain enabled.

```powershell
python analysis/decode_swingup_diagnostics.py path/to/device-monitor.log diagnostic.csv
```

Use a new CSV filename: the decoder refuses to overwrite existing files and
reports malformed records and the largest observed drop count. Its decoded
units/column names should be used for analysis, not the old `compare_runs.py`.

Raw record, all comma-separated:

```text
#D,t_ms,kind,direction,mode,reason,reject_mask,dropped,events,x0,...,x21
```

| kind | Meaning |
|---|---|
| M | Mode/status message; x0 is message code, other x fields unused |
| T | Accepted first seed or qualified half-swing turn; completed signed peak |
| J | Rejected raw candidate; signed local candidate peak, no energy update |
| O | Full rotation detected |
| R | Planned pump component changed sign (includes seed reversals) |
| B | Measured alpha crossed hanging-down without wrapping over upright |
| E | Commanded velocity reached current clipped target within 0.05 rad/s after R |
| N | Near-top snapshot |
| S | Periodic snapshot |

Event bitmask: raw turn candidate=1, bottom=2, reversal=4, completion=8, rotation=16, accepted=32, rejected=64. The kind
selects the highest-priority label; inspect bits for simultaneous events.
Direction is the sign of the post-filter pendulum rate; for T/J this is the new
direction, while the signed peak identifies the completed tip.

Modes: 0 IDLE, 1 COUNTDOWN, 2 BALANCE, 3 PUMP, 4 PARK (v2.3). Stop reasons: 0 none, 1 key,
2 sensor/posture, 3 deadline, 4 travel, 5 timeout, 6 fell. M message codes:
0 state transition, 1 calibration OK, 2 calibration failed, 3 arm-zero accepted,
4 start rejected, 5 requested status (`p`). M state transitions replace the old
human-readable COUNTDOWN/SWINGUP/BALANCE/STOP lines. A terminal M is mode 0 with
the reason; it is prioritized but cannot survive power loss before transmission.

Reject bits: angle=1, unstable-mode z=2, arm angle=4, arm velocity=8, raw A request=16.
Bits indicate failed gates using the pre-integration arm command for that tick.
They are meaningful for PUMP; mode 2 means A already captured.

| field | Value | Raw scale (divide by) |
|---|---|---:|
| x0, x1 | alpha from down, signed completed/current directional peak (radians) | 1000 |
| x2, x3 | turning estimate energy before/after this tick | 1000 |
| x4, x5 | learned loss, planned Vp (rad/s) | 1000 |
| x6, x7 | pump target before/after speed clipping (rad/s); controller's own values since v2.3 | 1000 |
| x8, x9 | integrated commanded arm velocity before/after this tick (rad/s) | 1000 |
| x10 | z = wn*d + filtered rate (rad/s) | 100 |
| x11 | unsaturated A acceleration request (rad/s²) | 10 |
| x12..x15 | accumulated work proxies: lower+/lower-/upper+/upper- | 1 |
| x16 | current command minus command at most recent R (rad/s) | 1000 |
| x17 | commanded arm theta (radians) | 1000 |
| x18 | filtered pendulum rate (rad/s) | 100 |
| x19 | trim (radians) | 1000 |
| x20 | capped controller acceleration u (rad/s²) | 10 |

Turn status x21: 0 none, 1 accepted seed, 2 accepted qualified turn, 3 rejected/no qualified traversal, 4 rejected/wrong lobe or direction, 5 rejected/seed below 5 degrees, 6 full rotation. Inspect J records to distinguish raw chatter from accepted history. Archived v2.1 rows decode with turn_status empty.

Each raw x is a signed 16-bit value. -32768 means nonfinite; values clamp at
+/-32767, so a boundary value may represent clipping. M.x0 is an unscaled code.
Energy is the last turning-point estimate, not instantaneous mechanical energy.
The completed T energy is x3; O energy is a marker, not measured rotation energy.
Targets x6/x7 are meaningful only in PUMP. Since v2.3 they are copied from the
controller, never recomputed and never fed back (v2.1-v2.2 logs: reconstructed).

At R, x8 is the command immediately before the new planned reversal; at E, x16
is the command change since that R. Compare it with nominal 2*Vp and inspect B
timing. Targets move with centering and phase, so E may never occur. These are
**commanded** velocities: motor movement/missed steps are not measured. The step
generator stops below 31 steps/s (~0.0609 rad/s), even if integrated velocity is
nonzero. A high-speed cap can also make integrated acceleration differ from u.

The proxy sums `filtered_rate*cos(alpha)*u*0.005` at 200 Hz, classified by
lower/upper half and pendulum-rate sign. Bins reset after accepted T/O (not J), whether or not the
record can be queued. Multiply by K/wn² for a reduced-model normalized-energy
proxy. It is **not measured work**: rate distortion, command saturation/deadband,
friction, centrifugal effects and missed steps all limit interpretation.

## Verification

Host tests check unchanged controller output with observation and a blocked UART,
queue overflow, terminal priority, partial writes, simultaneous event flags,
serialization and decoding. Existing state/fault/full-sketch and A parity checks
pass; nonlinear simulation results are unchanged, including the failing strict-
stiction stress case. Both Uno targets compile. Qualified diagnostic swingup uses 19,708
flash bytes and 1,058 static RAM bytes; balance uses 9,432 and 469. Host tests do
not prove worst-case execution time, stack margin or actual UART/ISR timing on Uno.
No firmware was uploaded by the agent. Next evidence is one user-run hardware log.

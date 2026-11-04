# Qualified turns and capture-window evaluation ? 2026-11-04

## Implemented hardware step 1

Uploadable `swingup` identifies `swingup-v2.2-qualified` and prints capture_deg=15.0.
Qualified turns use +/-5-degree bottom-phase hysteresis; the first seed needs no
prior bottom but must be >=5 degrees and outward-to-inward. After a seed, only a
candidate following a traversal into the opposite lobe can advance energy/loss.
Rejected raw sign changes do not replace the accepted directional peak or accepted
energy history. Full rotations reset qualification and retain existing overshoot
backoff. Terminal cancellation/rearm contract is unchanged.

A, its 60 rad/s? and pi rad/s caps, pump 100/4.363323 caps, centering, 20-second
timeout, travel guards and all capture gates remain unchanged. The default capture
angle remains 15 degrees. The original balance source SHA256 remains
`6705F59ADC818668F385ECDA32CD569200B138A2E528DC8EAB15E18F008C2C5C`.

Diagnostics preserve raw candidates: T means accepted; J rejected. Event bits32/64
and appended x21 turn_status show decision/reason. The decoder supports both new
31-field and archived30-field records. Work bins reset on accepted turns only.
See `swingup/DIAGNOSTICS.md` for exact enum/scaling.

## Regression and replay evidence

The 24129/24134/24139ms chatter regression failed against v2.1 before the fix. It now
passes in both directions. Tests cover zero-crossing jitter within hysteresis,
qualified opposite-lobe traversal, preserved peaks/history, rest seeding, small
seed rejection, full rotation/wrap and terminal cancellation/restart.

A fixed replay of recorded observations and stored raw-turn extrema from the
16:06 diagnostic trace accepts 50 turns and rejects 3 candidates. Both rapid chatter
updates are rejected while loss stays 0.30, rather than rising 0.30->0.83->1.09.
This is an estimator replay, **not** a prediction of the altered plant trajectory:
samples are sparse and the changed pump would feed back into real motion.

The original A parity check (800 ticks), full-sketch faults/cancellation, diagnostic
queue/backpressure/serialization and archived-log decoding pass. Required 15-degree
full nonlinear simulations still capture and hold 5 seconds; strict-stiction stress
still fails at travel. Successful simulation assumptions remain reduced near-upright
friction, ideal commanded motor tracking and approximate magnetic errors.

## Step 2: offline 15 vs18-degree comparison

`analysis/evaluate_capture_windows.py` compiles host-only variants; **it does not
create/upload an 18-degree hardware target**. Raw outputs/JSON remain separate from
measured logs under `.pio/capture-window-evaluation/`.

A 486-state grid samples tilt +/-14,16,17.5 degrees; inward rates 3/4/5 rad/s;
arm angles -15/0/15 degrees; arm speeds -1/0/1 rad/s; upright biases -1/0/1 degree.
Other capture gates remain active. The reduced nonlinear plant includes viscous
zeta .06, dry friction .02, centrifugal term, quantized encoder and step deadband.
These are illustrative assumptions, not measured complete hardware uncertainty.

| Angle gate | Eligible states | Rejected | Held 5 seconds among eligible |
|---|---:|---:|---:|
|15 degrees|159|327|159|
|18 degrees|398|88|398|

That limited near-center grid omits many reachable pump states. The full swing-up
comparison is decisive against enabling18 now:

- 15-degree lossless case: capture 19.385 s, held 5 s, max commanded arm 77.74 degrees.
- 18-degree lossless case: capture 7.075 s, **failed to hold; arm travel stop 88.99 degrees**.
- 18-degree identified-dry case improves simulated capture 4.290 -> 3.710 s and holds;
 other required non-lossless cases pass, but this does not cancel the travel failure.
- Strict-stiction stress fails at both windows.

Therefore the 18-degree evaluation reports a failed required rollout (exit1) and
is **not approved/enabled for hardware**. The evaluation runner reports that result
without disguising it as a passing regression. Wider angle alone is not a proven
safe region of attraction, even with the other gates.

## Build and next hardware action

Both Uno builds pass. Qualified swingup: 19,708 bytes flash / 1,058 static RAM; balance:
9,432 / 469. MCU deadline/stack behavior with the new observer still needs a hardware
check, despite the previous diagnostic build completing without timing faults.

```powershell
uv run --no-project --with ziglang python analysis/run_swingup_checks.py
uv run --no-project --with ziglang python analysis/evaluate_capture_windows.py
pio run -e swingup -e balance
```

User's next step is one centered attempt with the qualified15-degree build:

```powershell
pio run -e swingup -t upload
pio device monitor -f log2file
```

Reset centered/hanging still with VMOT off; confirm v2.2-qualified and 15.0 degrees;
VMOT on, z, s. x stops. Send the complete log and leave monitor open a second after
stop. Compare raw/accepted/rejected turns, time to loss saturation, reversal delivery,
directional tips and viable capture states. Do not change centering/capture alongside
this comparison. If qualification works but asymmetry/saturation persists, next
performance experiment is pump timing/centering at the same limits.

No firmware was uploaded and no git commit made by the agent. Step 1 implementation
and step 2 offline evaluation are complete; hardware comparison remains pending.

# motion_core open-loop test, 2026-11-04 13:54

Data: `data/motion_core_2026-11-04_1354.csv`. User report: everything smooth, arm returned as expected.

## Timing
Logged every 2nd 5 ms loop: intervals 9–11 ms (millis() granularity), no gaps > 11 ms, so the 200 Hz loop kept its rate while Timer1 generated steps.

## Velocity test (key 1)
Commanded theta peak 60.19°, speed 60°/s, homing ended at theta = 0.00 (commanded). Pendulum kicked −9.3° / +12.3° by the start/stop accelerations.

## Sine tests (steady part, first 2 cycles skipped)
| Drive | Arm amplitude | Pendulum amplitude | Ratio alpha/theta | Phase alpha − theta |
|---|---|---|---|---|
| 1 Hz | 15.1° | 4.6° | 0.31 | ≈ 156° |
| 2 Hz | 8.2° | 40.2° (peaks +36.5/−54.6) | 4.9 | ≈ 81° |

## Comparison with the linear model (acceleration input)
Hanging pendulum driven by arm acceleration: |alpha/theta| = (L_r/l_eq) ω² / |ω_n² − ω² + j 2ζ ω_n ω|, with ω_n = 12.1 rad/s (decay test), l_eq = g/ω_n² ≈ 0.067 m, L_r ≈ 0.07 m (approximate), ζ ≈ 0.06:
- 1 Hz: predicted ≈ 0.39, measured 0.31.
- 2 Hz (just above ω_n): predicted ≈ 7.6 with phase ≈ 90°, measured 4.9 with phase ≈ 81°.
Same order and the expected near-90° phase at resonance. The model over-predicts, as expected from the dry hinge friction (not in the viscous model), the 40° amplitude (nonlinear) and the ±5° encoder error. L_r must be measured precisely.

## Not verified
Real arm angle (no arm encoder: theta is the commanded value; the user saw it return to the mark). Single run.

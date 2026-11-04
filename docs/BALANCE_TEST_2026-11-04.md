# First hardware balance test, 2026-11-04 14:27

Data: `data/balance_2026-11-04_1427.csv`. User: "balances itself smoothly and very well".
Build: the log shows |u| reaching 100 rad/s^2, so this run used the earlier build (U_MAX 100, V_MAX 360 deg/s), before the limits were lowered to 60 rad/s^2 / 180 deg/s.

| Quantity | Value |
|---|---|
| Calibration | down 1289.5 counts, upright 3337.5 |
| Balanced | 10.0 s logged, still balancing when the log ended (no stop event) |
| Pendulum d | max 3.6°, rms 0.87° after the first 2 s |
| Arm theta | −9.9° … +24.9°, mean +5°, slow wander (half-period ≈ 0.9 s) |
| Control u | rms 18.5 rad/s^2, at the 100 limit 0.3 % of the time |
| Auto-trim | converged to −1.10° (upright reference was ~1.1° off; arm mean returned toward ~1°) |

Interpretation: the LQR balance works on the real hardware. The friction simulation (analysis/friction_check.py) was too pessimistic: the real hinge friction did not prevent settling, though the slow arm wander of ±10–25° may be partly friction (stick–slip) and encoder error. Auto-trim behaved as designed.

Not yet tested: the new lower limits; disturbance response (handout in-lab task 4); repeatability (one run); longer runs.

# LQR weighting comparison A vs B (handout in-lab task 6), 2026-11-04

Data: `data/balance_A_disturbance_2026-11-04_1437.csv`, `data/balance_B_disturbance_2026-11-04_1441.csv`.
Analysis: `python analysis/compare_runs.py <logs>`. Disturbance = light taps near the pendulum tip by hand (not repeatable in size).

| | A (moderate) | B (stiffer) |
|---|---|---|
| Gains K (θ, d, θ̇, ḋ) | −27.6, −600.5, −14.2, −44.9 | −36.2, −791.7, −19.3, −55.1 |
| Balanced | 17.5 s | 7.2 s, ended by the **arm limit** stop (θ reached −90°) |
| Pendulum d rms / max | 1.44° / 5.6° | 2.23° / 8.6° |
| Arm θ range | −35.8° … +38.7° | −90° … +48.4° |
| Control u rms / max | 23.9 / 100 rad/s² | 36.6 / 60 rad/s² (at the cap) |
| Disturbances (peak tilt → recovery) | 2.1° → 0.97 s; 4.3° → 1.81 s; 5.6° → 4.19 s | 6.3° → 1.76 s; 8.6° → not recovered (arm limit) |

User observation: B reacts more and oscillates. Decision: **use A**.

**Confound (important):** run A used the earlier upload (U_MAX = 100 rad/s², V_MAX = 360°/s; its log lacks the gain-set column and reaches u = 100). Run B used the current build (U_MAX = 60 rad/s², V_MAX = 180°/s) and sat at the 60 cap. Part of B's worse behaviour may be the lower acceleration cap rather than the weighting. The **current build with set A** (A + 60 rad/s² / 180°/s) has not been tested on hardware yet.

Interpretation: higher pendulum weight → larger gains → with the cap, more saturation and a larger arm swing; together with dry hinge friction and encoder noise this gives the oscillation the user saw. Simulation (no friction) predicted similar behaviour for both, so the difference is a hardware effect (saturation/friction/noise).

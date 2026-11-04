# Parameter sheet (SI) — status 2026-11-04

| Symbol | Value | Source |
|---|---|---|
| L_r (motor axis → pendulum hinge) | 0.070 m | measured by user |
| m_p (pendulum) | 0.012 kg (11.80–12.40 g by material) | user, average |
| ω_n (pendulum, hanging) | 12.08 rad/s (T = 0.52 s) | free-decay identification, 2 runs |
| l_eq = J_p/(m_p l_p) = g/ω_n² | 0.0672 m | from ω_n |
| ζ (equivalent viscous) | ≈ 0.06 | first decay cycle; friction is mainly dry |
| T_c (dry hinge friction) | ≈ 0.10 · m_p g l_p | decay: ~11.4° amplitude drop per half cycle |
| l_p (hinge → pendulum centre of mass) | **not measured yet** | balance on an edge |
| J_p (about the hinge) | = m_p l_p l_eq = 8.06e-4 · l_p kg·m² (l_p in m) | needs l_p |
| b_p | = 2 ζ ω_n J_p | needs l_p |
| J_r, b_r, motor constants | not needed with the stepper | the stepper imposes the arm motion (acceleration-input model) |
| Steps | 3200 steps/rev (1/16), + = clockwise from above | step_test |

Note: a uniform 7 cm rod would give l_eq = 2L/3 ≈ 4.7 cm; the measured 6.7 cm means the mass is concentrated further out (tip mass / hub), so do not assume l_p = L/2 — measure it.

Controller design uses the acceleration-input model: needs only ω_n, l_eq (both identified), ζ and L_r. See `analysis/lqr_design.py`.

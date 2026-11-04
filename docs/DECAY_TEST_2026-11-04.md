# Pendulum free-decay test, 2026-11-04 (arm held, VMOT off)

Data: `data/decay_2026-11-04_1335.csv` (100 Hz, 6.9 s). Analysis: `python analysis/decay_analysis.py data/decay_2026-11-04_1335.csv`.

| Quantity | Value | Note |
|---|---|---|
| Release amplitude | about 86° | larger than the planned 20–30°; large-angle period is a little longer |
| Period T | 0.516 s (omega_d ≈ 12.2 rad/s, f ≈ 1.94 Hz) | from successive peaks; damping is light, so omega_n ≈ omega_d |
| Peaks (deg, about rest) | −86.1, +54.7, −59.5, +36.5, −33.2, +18.3, −5.6, then stops | stopped after ~1.6 s, about 3 cycles |
| Log decrement (viscous model) | 0.37–0.69, rising as amplitude falls (ζ ≈ 0.06–0.11) | NOT constant, so a pure viscous b_p fits poorly |
| Amplitude drop | peak-to-peak falls by an almost constant 18–28° per half cycle | constant drop = dry (Coulomb) friction signature |
| Rest position | 1275 counts before, 1264 after (≈1° apart) | stuck off-centre: also dry friction |

## Interpretation
- The hinge is dominated by **dry friction**, not viscous damping: the amplitude falls by a roughly constant amount each swing (instead of a constant ratio) and the pendulum stops abruptly, slightly off-centre. The alternating 18°/27° drops are likely the sensor's ±5° nonlinearity (weak magnet) plus the zero estimate.
- For the handout's viscous b_p you can still report an *equivalent* value from the first cycle (ζ ≈ 0.06): b_p = 2 ζ ω_n J_p (J_p about the hinge). State that the friction is mainly Coulomb and that this is an approximation.
- From the period: ω_n² = m_p g l_p / J_p (J_p about the hinge) gives **J_p = m_p g l_p / ω_n² ≈ m_p l_p · 0.066 m**. This needs m_p (scale) and l_p (hinge → centre of mass, balance the pendulum on an edge). Equivalent simple-pendulum length g/ω_n² ≈ 6.6 cm.
- Practical: friction this large (about 10 % of the maximum gravity torque) will matter in the model; check the hinge for rubbing or a tight bearing before balancing tests.

## Not verified
Encoder accuracy ±5° (weak magnet) limits the amplitudes; only one run; large release angle. Repeat 3× from ~30° for repeatability.

## Run 2 (13:38, held horizontal then released): `data/decay_2026-11-04_1338.csv`
| Quantity | Run 1 (13:35) | Run 2 (13:38) |
|---|---|---|
| Release amplitude | ~86° | ~82° (held horizontal) |
| Period T | 0.516 s | 0.523 s |
| omega_d | 12.2 rad/s | 12.0 rad/s |
| Amplitude drop per half cycle (Coulomb fit) | 11.7° | 11.2° |
| T_c / (m_p g l_p) | 0.102 | 0.097 |
| Equivalent ζ, first same-side cycle | 0.06 | 0.05–0.065 |

The two runs agree: **T ≈ 0.52 s (ω ≈ 12.1 rad/s)** and a **dry-friction torque T_c ≈ 0.10 · m_p g l_p**.
The first swing in run 2 reads −92.7° after a +82° release (it cannot gain energy): the negative side reads larger in both runs, consistent with the encoder nonlinearity found in the quarter-turn check (≈±5°). Per-side ratios are therefore used for ζ, and the Coulomb estimate uses peak-to-peak values.
The analysis script now detects the release point after a hold (it previously counted the held position as a peak).

# Swing-up v2.3 - centred pump, raised pump limits, park on timeout

Status: implemented, host-tested and compiled. **Not uploaded or hardware-tested.**
Workflow unchanged: `z`, `s` -> countdown -> pump -> automatic catch -> balance A.
`balance/balance.ino` is unchanged (SHA256 6705F59A...C2C5C).

## Evidence that motivated the changes (hardware 16:06, 16:27, 16:31)

- Pump speed Vp sat at the 250 deg/s cap for the whole 16:31 run; learned loss sat
  at its 1.2 cap. More loss/energy target cannot add energy; only delivery can.
- The commanded arm oscillated about +17..+22 deg (all three runs). Negative
  reversals were clipped (raw -5.4..-5.6 rad/s -> -4.36) so instantaneous centering
  could not remove the offset; positive tips were 20-25 deg lower than negative.
- The open-loop start used sin(wn t) from rest, so the arm swept 0..+65 deg
  (mean about +32 deg) before pumping began.
- Decay tests: dry friction T_c ~ 0.10 m g l_p, i.e. roughly 0.1 x swept angle of
  normalized energy lost per half swing (about 0.45 at 130 deg tips).

## Changes (`swingup/controller.h`)

| # | Change | Why |
|---|---|---|
| 1 | Lower-half pump target = +/-vp + bias, bias = clamp(-2/s * thetaMean, +/-1 rad/s), thetaMean = arm angle low-passed with tau 0.5 s, vp = min(Vp, V_MAX - abs(bias)) | Centring can no longer be clipped away; both reversals are 2*vp |
| 2 | Start kick uses cos(wn t) | Arm start motion centred on 0 |
| 3 | Pump-only U_MAX 150 rad/s^2, V_MAX 300 deg/s (was 100 / 250) | More delta-v per bottom reversal; sharper reversal |
| 4 | PARK mode (4) after pump timeout: driver on, arm to commanded 0 at <= 1 rad/s (decelerating at <= U_MAX first), max 4 s, then TIMEOUT stop | Lost-step check each run; leaves arm centred |
| 5 | Controller records lastPump / lastTargetRaw / lastTarget; diagnostics copy them | Logged targets can no longer drift from the real law |

Unchanged: balance A gains, 60 rad/s^2 and 180 deg/s balance limits, trim,
25-degree drop (driver off, same as balance.ino), capture gates (18 deg, z<3,
arm<60 deg, arm speed<=pi rad/s, raw A<=75), turn qualification, 20 s pump time,
75-degree predictive braking and 791-step ISR cutoff.

## Motor margin (estimate, not measured)

17HS4023: about 13 N cm holding torque at its 0.7 A rating
([datasheetcafe](https://www.datasheetcafe.com/17hs4023-datasheet-stepper-motor/),
[uelectronics](https://uelectronics.com/producto/nema-17-17hs4023-motor-a-pasos-0-7amp-1-4kg-cm/)),
so roughly 9 N cm at the configured ~0.5 A. No pull-out curve was found for this motor;
pull-out torque depends on drive and supply, and general guidance is a 25-50 %
margin ([motioncontroltips](https://www.motioncontroltips.com/faq-what-is-pullout-torque-and-why-is-it-a-critical-stepper-motor-value)).
300 deg/s is 2667 microsteps/s (50 rpm), a low speed for a NEMA 17 at 12 V. Load
inertia is not measured; an assumed ~1e-4 kg m^2 gives ~1.5 N cm at 150 rad/s^2 plus
~1-2 N cm pendulum reaction. **Missed steps are undetectable by firmware** (no arm
encoder); the park-to-zero check is the practical test.

## Verification (local)

`uv run --no-project --with ziglang python analysis/run_swingup_checks.py`: all pass.
New regression tests: timeout -> PARK -> TIMEOUT; PARK deceleration <= U_MAX per tick,
speed <= PARK_V once slow, never catches with an upright pendulum, reaches 0 within
4 s with whole-step quantisation and the 31 steps/s deadband, key stop during PARK;
mean-centring targets never clipped for arm offsets -60..+60 deg; centred kick.

Required nonlinear scenarios (capture 18): all capture and hold 5 s; catch time
0.83 s lossless (was arm-travel failure), 2.50 s identified dry 0.10 (was 3.71),
3.38 s dry 0.12, 3.78 s coupling -15 %. Strict stiction still fails (unchanged).
15 vs 18 grid: 159/159 vs 398/398 eligible states held; both full sweeps pass.

Harder exploratory plant models (not hardware-calibrated; motor lag added only
while pumping to imitate the observed shortfall), late median tips:

| Model | v2.2 (100/250) | v2.3 |
|---|---|---|
| dry 0.10 + 40 ms lag | catches 10.2 s | catches 2.8 s |
| dry 0.10 + 60 ms lag | 105 deg, timeout | 123 deg, timeout |
| coupling x0.7 + 40 ms lag | 102 deg, timeout | 124 deg, timeout |
| dry 0.15 + 40 ms lag | 99-102 deg, timeout | 117-121 deg, timeout |

So v2.3 raises the reachable swing in every model tried, but if the real shortfall
is as large as the harsh models, it still will not reach the top. The simulator
does not reproduce the hardware's arm offset, so the centring fix is verified by
unit tests, not by a reproduced failure.

Builds: swingup 20,004 B flash / 1,078 B RAM; balance 9,432 / 469.

## What to send back after the hardware run

The full log (banner `swingup-v2.3`), and if it timed out: is the arm on its 0 mark
after the park? Off-mark = lost steps -> lower U_MAX/V_MAX via `-DSWING_U_MAX` /
`-DSWING_V_MAX_DEG` build flags or raise driver current within the motor rating.

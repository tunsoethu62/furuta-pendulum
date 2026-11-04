# HANDOFF — Furuta (rotary inverted) pendulum, Arduino Uno + stepper

Status date: **2026-11-04**. Read this file first, then `docs/` as needed. Everything below is from this repo and the user's hardware logs; items marked *not verified* were not tested.

## 1. Goal and course context
Differential-equations course project (handout `EN-Furuta_Handout_LQR.pdf`, not in this repo; the user has it). Required: model derivation, linearisation, state space, LQR, sensing, balance near upright, disturbance test, a second LQR weighting, **swing-up with mode switching**, logging, simulation vs experiment comparison, parameter sheet with sources.

**Done:** hardware bring-up, identification, open-loop validation, LQR balance on hardware (set A), disturbance tests, A/B weighting comparison, worked calculation (Markdown + LaTeX/PDF).
**Not done:** hardware swing-up validation/tuning, repeatability runs, post-lab plots/report, l_p measurement, θ sign convention choice.

## 2. Working rules (from the user)
- All Furuta code/docs live in **`D:\Uni\Semester-3\Diff-Equations\futura\furuta-pendulum`** (git repo, origin github.com/tunsoethu62/furuta-pendulum). Nothing is committed beyond `first-commit`: all the work below is **uncommitted**; ask before committing/pushing. `logs/` and `.vscode/` were created by the user's tools.
- The user wants **complete copy-paste sketches**, not partial edits, and **uploads/runs hardware themselves**. Never flash or move the motor yourself; give exact commands and what to send back.
- Collaborators in OpenBot: **Arjarn Bot** (tutor; derivation/teaching), **Canvas** (LaTeX/PDF typesetting), **QA Tester**.

## 3. Hardware (confirmed unless noted)
| Item | Value |
|---|---|
| MCU | Arduino Uno (ATmega328P, 5 V) |
| Driver | DRV8825, sense resistor R100 (I = 2·Vref), Vref set by user (target ≈ 0.25 V ≈ 0.5 A, max 0.35 V), 12 V VMOT, 100 µF cap |
| Motor | 17HS4023, 0.7 A/phase (user-reported) |
| Microstepping | M0=GND, M1=GND, M2=5V → **1/16, 3200 steps/rev**; confirmed: 800 pulses = exactly 90° and back |
| Pins | STEP=D3, DIR=D4, EN=D5 (active LOW), AS5600 SDA=A4, SCL=A5 |
| Direction | `INVERT_DIR = true` → positive steps = **clockwise seen from above** (DIR LOW = CW) |
| Arm encoder | **none**: arm angle θ = commanded step count (missed steps undetectable) |
| Pendulum encoder | one AS5600 at the hinge; **magnet field too weak** (STATUS 0x13, AGC 128 = max, magnitude ~160); noise ±0.1°, nonlinearity about ±5° per quarter turn. User chose not to fix the magnet |
| Arm travel | wiring does not allow full turns; software limit **±90°** |
| L_r | 0.070 m (motor axis → hinge) |
| m_p | 0.012 kg (11.8–12.4 g) |
| l_p | **not measured** (hinge → pendulum COM); must be ≤ l_eq = 6.7 cm |

## 4. Identified model (acceleration-input)
Stepper imposes the arm motion, so input **u = θ̈**. Pendulum (α from hanging down):
α̈ + 2ζω_n α̇ + ω_n² sin α = s·K cos α · u, with K = L_r/l_eq, l_eq = J_p/(m_p l_p) = g/ω_n², **s = +1** for the current encoder/arm directions (derived as a convention bit: s = −1 if α were defined positive along the arm motion; re-derive if any direction changes).
- ω_n = 12.08 rad/s (T ≈ 0.52 s, 2 decay runs; ±3 %), l_eq = 0.0672 m, K = 1.042, ζ ≈ 0.06 (equivalent only).
- Hinge friction is mainly **dry (Coulomb)**: T_c ≈ 0.10·m_p g l_p (≈ 11.4° amplitude drop per half cycle).
- Upright (d = α − π): A = [[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,ω_n²,0,−2ζω_n]], B = [0,0,1,−K]ᵀ; open-loop eig +11.38, −12.83, 0, 0; controllable (det C = −K²ω_n⁴).
- Centrifugal term neglected: valid for θ̇² ≪ ω_n² (≈ 27 % error at 2π rad/s).
Full derivation and numbers: `docs/FURUTA_MODEL_WORKED_CALCULATION.md` (+ `.tex`, `.pdf`), `docs/PARAMETERS.md`.

## 5. Controller (chosen: gain set A)
Discrete LQR (ZOH, dt = 5 ms), Bryson weights. `analysis/lqr_design.py A|B`.
- **A (in use):** Q = diag(1/0.5², 1/0.08², 1/3², 1/5²), R = 1/15² → **K = [−27.555, −600.497, −14.217, −44.909]**, u = −Kx, x = [θ, d, θ̇, ḋ].
- B (rejected by user: more reaction/oscillation; removed from the sketch, still reproducible with `lqr_design.py B`): K = [−36.181, −791.749, −19.308, −55.106].
- Implementation (`balance/balance.ino`): u integrated to a speed command (θ̇ = commanded speed), ḋ = 25 Hz low-pass of the finite difference, Timer1 CTC step generator (prescaler 8; speeds < 31 steps/s = stop), 200 Hz loop, auto-trim of the upright reference (ċ = −0.023·θ after 1 s, ±3°), catch window ±4° for 0.2 s, stop at |d| > 25° or |θ| > 90°.
- Limits in the **current** build: U_MAX = 60 rad/s², V_MAX = 180°/s (lowered after Arjarn Bot's review: untested motor limits, centrifugal term).

## 5a. Experimental swing-up v2.4 (hardware test pending)
- **17:00 v2.3 RESULT:** first automatic catch at d 15.6 deg (rate -2.57 inward, z 0.73, raw A 59) after 14.8 s pumping; arm speed hit the 180 deg/s balance cap within ~55 ms, arm 7.7->69 deg, pendulum reached ~5 deg then fell (FELL at 0.385 s). Best negative tips 18.2-19 deg from upright; positive side still ~25 deg weaker. Arm mean +11 deg (was +22), no clipped reversals, no full rotation, pump never reached the 75-deg brake. Report `docs/SWINGUP_TEST_2026-11-04_1700_v2.3.md`.
- **CURRENT: v2.4** (banner `swingup-v2.4`) = v2.3 + catch phase: 1 s after an automatic catch A uses 100 rad/s^2 / 360 deg/s (older A build balanced on hardware), then the speed limit ramps down at 60 rad/s^2 to the proven 60/180. Gains/trim/drop/capture gates unchanged; balance parity unaffected. Sim catch basin d 10-18: 351->374/375 nominal, 251->374 with 0.05 near-top friction, 202->338 weaker plant; held cases use up to ~85 deg arm travel. The simulator holds the exact hardware catch at 60/180, so the hardware failure cause is not reproduced. All checks pass; builds 20,304 B / 1,080 B. Not hardware-tested.
- **CURRENT: v2.3** (banner `swingup-v2.3`, capture 18). User asked to implement every possible fix with workflow swing up -> catch -> balance. Changes in `swingup/controller.h`: mean-arm-centred pump bias with reserved headroom (no clipped reversals), cos start kick (arm centred), pump-only limits 150 rad/s^2 / 300 deg/s (was 100/250; motor margin estimated, not measured), PARK mode 4 after pump timeout (arm returns to commanded 0 at <=1 rad/s, max 4 s, then TIMEOUT; arm off its 0 mark = lost steps), diagnostics copy controller targets. Balance A, its limits/trim/25-degree drop (driver off), capture gates, qualification unchanged; `balance.ino` unchanged. All host checks pass; 18-degree lossless arm-travel failure gone; strict stiction still fails; harsh exploratory plant models still plateau below the top. Builds 20,004 B flash / 1,078 B RAM. Not uploaded or hardware-tested. Report `docs/SWINGUP_V2.3_2026-11-04.md`. Pre-v2.3 sources backed up in ignored `.pio/backup-v2.2-qualified-18/`. Ask user for the log and whether the arm ended on its 0 mark.
- **16:31 RESULT (capture 18):** no BALANCE, 20 s timeout, 0 dropped. Closest tip 45.07 deg from upright (16:27 at capture 15: 26.16). Never within 25 deg, so the 18-degree window was not exercised; difference vs 16:27 is run variation (pump code identical). Vp saturated at 4.363 the whole run; commanded arm mean +17..+22 deg in all three diagnostic runs, negative reversals clipped, positive lobe ~20-25 deg weaker. Proposed (not implemented): reserve centering headroom under V_MAX so the arm oscillates about zero. Report `docs/SWINGUP_TEST_2026-11-04_1631_capture18.md`.
- **LATEST USER-AUTHORIZED EXPERIMENT: capture +/-18 degrees.** User explicitly requested changing firmware to18 first after supplying16:27 log. `swingup/controller.h` default SWING_CAPTURE_DEG is now18.0; banner `swingup-v2.2-qualified-18`, capture_deg18.0. Only angle gate and build identity changed; A, other gates, qualification, centering, caps, travel and25-degree fall cutoff unchanged. Earlier18-degree full lossless simulation failed at arm travel; this remains an experimental user-requested test, not a resolved safety/robustness result. No upload/commit by agent. Raw16:27 log preserved as `data/device-monitor-261104-162715.log`; not yet fully analyzed. Earlier15-degree recommendations below are historical and superseded for this requested experiment.

- **CURRENT HARDWARE STEP: v2.2-qualified, capture15degrees.** User authorized stepwise implementation. Phase-qualified turns now require a hysteretic traversal from accepted lobe beyond+/-5degrees into opposite lobe before the next accepted turn; first >=5-degree outward/inward seed is allowed without prior bottom. Raw candidate sign history is separate; rejections preserve accepted peak/energy/loss. Rotation resets qualification, cancellation remains terminal. A/centering/caps/travel/timeout unchanged. Tests replay16:06 chatter:50accepted/3rejected; both rapid loss updates rejected at0.30. T accepted/J rejected; events bits32/64; appended x21 turn_status; decoder supports31-field new and30-field legacy logs.
- **18-degree evaluation completed, NOT enabled.**486-state near-center grid:15deg159/159eligible held;18deg398/398held. Full18-degree lossless rollout catches earlier then fails at arm travel88.99deg, while15-degree baseline holds. Keep hardware at15; no widened hardware target. Details `docs/SWINGUP_QUALIFICATION_2026-11-04.md`; reproducible `analysis/evaluate_capture_windows.py`. All default regression/build checks pass; strict-stiction stress still fails. Build19,708flash/1,058static RAM. Next: user uploads existing swingup target, confirms v2.2-qualified/capture_deg15.0, runs one centered attempt, sends full log. No agent upload or commit.

- **DIAGNOSTIC RESULT (16:06):** saved raw `data/device-monitor-261104-160607.log`, decoded `data/swingup_diagnostic_2026-11-04_1606.csv`, report `docs/SWINGUP_DIAGNOSTIC_2026-11-04_1606.md`. 469 valid records, zero malformed/dropped, no timing fault, normal20-second timeout, no BALANCE. Best completed tip152.41deg from down (27.59deg short); negative/positive maxima152.41/133.90deg. Startup turn events24129/24134/24139ms (5ms apart) drive loss.30 ->.83 ->1.09 without bottom traversal. Late reversal command delta-v is median86.35%/89.47% of2Vp for positive/negative directions, with all16 late negative R targets clipped vs0/17 positive. Actual motor velocities remain unmeasured. Proposal for discussion: qualify accepted energy turns with phase traversal, leaving A/capture/caps/centering unchanged; then one comparison run. Firmware unchanged during analysis; no claim this alone guarantees capture. See report before further tuning.

- **Historical experiment: v2.1-diag.** User approved the proposed telemetry-only run following Arjarn Bot's review. `swingup/controller.h` remains SHA256 `6E6B39D6DAD0BC6FF4D84CC51D85642F3B9095A96CC02D03A1EF553858BE4B33`; no control or capture/limit changes. `swingup/diagnostics.h` is a read-only observer. Sketch now emits buffered integer `#D` records (normal 50 Hz CSV replaced), 10 Hz snapshots plus turn/reversal/bottom/near-top events and work proxies. A 4-record queue, 240-byte line buffer, UART-capacity checks and <=16-byte drains avoid blocking for output; dropped counter exposes overflow. Status/stop codes are numeric M records. Read `swingup/DIAGNOSTICS.md`; decoder is `analysis/decode_swingup_diagnostics.py`. Both builds and host tests pass; measured MCU deadline/stack behavior remains unverified. Diagnostic build: 18,956 bytes flash / 1,024 static RAM. User uploads/runs; no agent hardware operation.
- Latest v2.1 hardware log saved at `data/device-monitor-261104-154657.log`: 1000 pump samples, closest upright 34.30 degrees, alpha -145.70..126.05 degrees, commanded arm -19.91..51.30 degrees, no predictive braking, no BALANCE, terminal timeout. Arjarn independently confirmed the worsening versus v2 and recommended logging delivered reversals before tuning; widening angle alone fails other capture gates and 34 degrees exceeds the 25-degree fall cutoff. That diagnostic log has now been received and analyzed (16:06 report above). Keep A and all gates/limits unchanged while interpreting this run.

- User explicitly requested reusing the successful `balance/balance.ino` for catch. That source remains unchanged (SHA256 `6705F59ADC818668F385ECDA32CD569200B138A2E528DC8EAB15E18F008C2C5C`). The swing-up BALANCE path reproduces A's gains, 60 rad/s^2 cap, pi rad/s cap, trimmed derivative, filtering, auto-trim and 25-degree terminal drop. `tests/balance_parity.cpp` compares 800 ticks with the actual original sketch. Moving capture retains real velocity/filter state instead of the manual-catch reset/dwell.
- Latest supplied v1 log `logs/device-monitor-261104-151201.log`: 478 pump samples, peak excursion 94.32 degrees, speed-energy estimate up to 2.497 despite stalling. V2 uses turning-point amplitude energy and bottom-timed arm-velocity reversals, with adaptive half-swing loss and near-top braking. Encoder-speed distortion is a hypothesis, not directly measured truth.
- Flow: `z`, `s`, 3-second driver-off countdown, pump, automatic A capture. Capture checks tilt <15 degrees, unstable-mode magnitude <3 rad/s, arm <60 degrees, speed <=pi rad/s and raw A request <=75 rad/s^2. No additional command required for balance. Every stop is terminal; physical recenter + `z` required before explicit restart.
- **Experimental PUMP-only limits: 100 rad/s^2 / 250 deg/s. BALANCE: proven 60 / 180.** Do not confuse old A's acceleration evidence with validation of the new pump combination. Startup banner prints both sets. No hardware uploaded by the agent.
- Pump braking at 75 degrees, ISR cutoff at 791 steps (88.9875 degrees), 20-second pump timeout. Faults/cancellation disable the driver. No automatic failed-catch retry.
- Core, complete-sketch and parity tests pass with assertions explicitly enabled (`-UNDEBUG`; optimized Zig otherwise disabled them). Both Uno targets compile. Required nonlinear simulation cases capture and hold five seconds, including dry 0.10/0.12, coupling +/-15%, encoder-error shapes and bias. Identified-dry simulation now captures at 4.290 s. Lossless is marginal at 19.385 s; strict-stiction stress fails to hold and stops at travel. Near-upright friction reduction is an explicit simulation assumption.
- Latest hardware log (15:32, v2) saved as `data/device-monitor-261104-153252.log`: 1000 pump samples, closest upright 16.37 degrees, arm -18.56..52.43 degrees, no predictive braking, terminal 20-second timeout, no BALANCE. V2.1 fixes absolute-peak carryover across asymmetric swings (regression reproduced before fix) with signed directional extrema, and halves learned compensation after full rotations to prevent repeated simulated overshoots. A/capture gates/limits unchanged. Banner `swingup-v2.1`. All required simulations and regression checks pass; strict-stiction stress still fails. Hardware v2.1 untested. Details: `docs/SWINGUP_TEST_2026-11-04_1532.md`.
- Read `docs/SWINGUP.md` (identical copy at `swingup/SWINGUP.md`) for exact procedure/evidence. Tests: `uv run --no-project --with ziglang python analysis/run_swingup_checks.py`. Generated simulation files stay in ignored `.pio/swingup-host/`, not measured `data/`. Telemetry is 50 Hz/11 named columns; old `compare_runs.py` does not match this schema.

## 6. Hardware results
| Test | Result | Doc |
|---|---|---|
| step_test | 1/16 confirmed, no missed steps at 100 pulses/s | `docs/HARDWARE_CHECKS.md` |
| Encoder quarter-turn check | steps 968/1068/969 counts (ideal 1024); down→up 2036 | `docs/HARDWARE_CHECKS.md` |
| Free decay ×2 | T 0.516/0.523 s; dry friction | `docs/DECAY_TEST_2026-11-04.md` |
| motion_core open loop | 200 Hz held; sine 1 Hz ratio 0.31 (model 0.385), 2 Hz 4.9 (model 7.56) | `docs/MOTION_CORE_TEST_2026-11-04.md` |
| Balance A (old limits) | 10 s, d rms 0.87°, θ −10…+25°, trim −1.1° | `docs/BALANCE_TEST_2026-11-04.md` |
| Balance A, current limits (14:46) | 9.579 s through log end; d RMS 1.19 degrees, peak 4.95 degrees; recovery 1.969 s; u cap 60 | `docs/BALANCE_A_CURRENT_LIMITS_2026-11-04.md` |
| Disturbance A vs B | A: 17.5 s, 3 taps recovered (≤ 4.2 s); B: oscillated, hit arm limit | `docs/WEIGHTING_COMPARISON_2026-11-04.md` |

**Update (14:46 run A):** A balanced throughout the 9.579 s recorded segment; a 4.95-degree excursion recovered in 1.969 s. The log ended in BALANCE. Acceleration clipped at 60 rad/s^2 (3.65% of samples), consistent with the current source. Keep A and current limits. Firmware identity and speed cap are not printed in the log. See `docs/BALANCE_A_CURRENT_LIMITS_2026-11-04.md`. Longer repeatability runs remain pending.

## 7. Files
| Path | Purpose |
|---|---|
| `platformio.ini`, `pio/<sketch>/main.cpp` | one PlatformIO env per sketch; wrappers include the unchanged `.ino` (Arduino IDE still works) |
| `step_test/` | bounded step/direction test (f/b/z/x) |
| `vref_reader/` | Vref via A0, driver disabled |
| `as5600_read/` | encoder logger (100 Hz; `PRINT_DIAG` for status/AGC/magnitude) |
| `motion_core/` | Timer1 step generator + open-loop velocity/sine tests |
| `balance/` | LQR balance, gain set A only |
| `swingup/`, `pio/swingup/` | Experimental v2 amplitude pump and automatic proven-A capture; hardware validation pending |
| `analysis/swingup_sim.cpp`, `analysis/run_swingup_checks.py`, `tests/swingup*` | Shared-core nonlinear simulations and host regression tests, no hardware access |
| `motor_testing/`, `acceleartion/` | user's original sketches, fixed (EN order, MICROSTEPS=16, no auto-start); superseded |
| `analysis/decay_analysis.py` | period, log decrement, Coulomb estimate from a decay log |
| `analysis/lqr_design.py` | model, A/B matrices, DARE gains, nonlinear sim (needs numpy/scipy) |
| `analysis/friction_check.py` | crude stiction sim (was too pessimistic vs hardware) |
| `analysis/compare_runs.py` | balance/disturbance metrics per run |
| `data/` | all analysed logs (CSV) |
| `docs/` | test reports, parameter sheet, worked calculation, PlatformIO guide |

## 8. How to build / run
- PlatformIO Core: `D:\PlatformIO\.platformio` → `$env:PLATFORMIO_CORE_DIR="D:/PlatformIO/.platformio"; D:\PlatformIO\.platformio\penv\Scripts\pio.exe run -e balance` (all 7 envs compile).
- User flow: `pio run -e <env> -t upload`, then `pio device monitor -f log2file` (log appears in `logs/` or the project folder).
- Python analysis: `uv run --no-project --with numpy --with scipy python analysis/lqr_design.py A`; the other scripts are stdlib-only (`python analysis/<script>.py data/<log>.csv`).
- Balance procedure: VMOT off, arm on its 0 mark, pendulum hanging still → upload/reset (calibrates down) → VMOT on → `z`, `a` → lift the pendulum to upright and hold → `# BALANCE` → let go. `x`/space = stop.

## 9. Next steps (suggested order)
1. **First current-limit A check completed:** retain A and current limits; see `docs/BALANCE_A_CURRENT_LIMITS_2026-11-04.md`. No reason to raise the limits based on this log.
2. Repeatability: 3 balance runs ≥ 30 s; 3 small-amplitude (≤ 20°) decay runs to tighten ω_n.
3. Measure **l_p** (balance on an edge) → J_p = m_p l_p l_eq, b_p = 2ζω_n J_p for the parameter sheet.
4. Choose and document the θ sign convention for the report; re-check s if anything changes.
5. **Swing-up prototype implemented; hardware validation/tuning pending.** Read `docs/SWINGUP.md`. User uploads/runs the separate `swingup` target and sends full telemetry. V2 local simulations now capture; the strict-stiction stress case fails. Pump-only limits differ from proven balance; obtain new v2 hardware telemetry before declaring success. Keep `balance` as the established fallback.
6. Post-lab: plots of θ, α, θ̇, α̇, u; simulation vs experiment; ≥ 3 causes of model mismatch (dry friction, encoder nonlinearity/weak magnet, open-loop arm angle / possible missed steps, neglected centrifugal term, large-amplitude nonlinearity).

## 10. Known limitations
No arm encoder (θ is commanded, not measured); weak magnet (±5° nonlinearity, upright offset needs trim); dry hinge friction not in the linear model; ω_n ±3 %; viscous ζ only an equivalent; motor torque/speed limits not characterised beyond 60°/s open loop and the balance runs; single runs only.

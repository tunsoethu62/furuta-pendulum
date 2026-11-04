# Experimental swing-up v2.4 - automatic catch into proven A

## Current: v2.4 (banner `swingup-v2.4`)

17:00 v2.3 run: first automatic catch (d 15.6 deg), lost after 0.385 s with the arm
pinned at the 180 deg/s balance speed cap. v2.4 = v2.3 plus a **catch phase**: for
1 s after an automatic catch A uses 100 rad/s^2 / 360 deg/s (an older A build that
balanced on this hardware), then ramps back to the proven 60 / 180. Evidence and
simulation: `docs/SWINGUP_TEST_2026-11-04_1700_v2.3.md`. Same procedure as below;
banner also prints `# catch phase U_MAX=100.0 V_MAX=6.283185 ms=1000`.

## v2.3 (still included in v2.4), capture +/-18 degrees

Workflow unchanged: swing up -> automatic catch -> balance A. Changes after the
16:31 run (closest 45 deg; pump speed at its cap for 20 s; commanded arm centred
near +20 deg in all three diagnostic runs). Details and evidence:
`docs/SWINGUP_V2.3_2026-11-04.md`.

1. **Centred pumping.** Lower-half pumping uses a bias from the low-passed arm
   angle (tau 0.5 s, 2/s, max 1 rad/s) and reserves that speed, so the final
   clamp never removes centring and both reversals are 2*vp.
2. **Centred start kick** (cos instead of sin): the open-loop start no longer
   pushes the arm to 0..+65 deg.
3. **Pump-only limits raised to 150 rad/s^2 / 300 deg/s** (from 100 / 250).
   Balance A keeps its proven 60 rad/s^2 / 180 deg/s, gains, trim and 25-degree
   drop. Motor margin is an estimate, not measured: watch for lost steps.
4. **Park on timeout.** After the 20 s pump timeout the driver stays on and the
   arm returns slowly (<=1 rad/s) to commanded 0, then turns off (mode 4 in logs,
   at most 4 s). **If the arm is not on its 0 mark afterwards, steps were lost.**
   `x`/space still stops at once, including during park. A failed catch still
   turns the driver off immediately, exactly like balance.ino.

Capture gates unchanged (18 deg, z<3, arm<60 deg, arm speed<=pi, raw A<=75).
With these changes the earlier 18-degree lossless arm-travel failure no longer
occurs; all required simulation scenarios capture and hold at both 15 and 18.

**Not hardware-tested.** Run once, as before: VMOT off, arm on 0 mark, pendulum
still -> upload/reset -> VMOT on -> `z` -> `s`. Keep clear. Send the full log and
say whether the arm ended on its 0 mark (if it timed out).

## Historical: user-requested18-degree hardware experiment (v2.2-qualified-18)

The existing `swingup` target now uses +/-18 degrees and identifies
`swingup-v2.2-qualified-18` with capture_deg=18.0. Other gates, A, pump/centering,
limits and25-degree fall cutoff are unchanged. This explicitly supersedes the
15-degree instructions below for the present experiment. The earlier18-degree
lossless simulation hit the arm limit; that limitation remains unresolved. No
claim of hardware success or general capture safety. Upload/run remains the user?s
step; x stops. Earlier15-degree results/procedures below remain historical.

## Active step: qualified turns, 15-degree capture (v2.2-qualified)

The `swingup` target now qualifies accepted turns by opposite-lobe bottom traversal
with +/-5-degree phase hysteresis. Raw chatter remains visible as J records, with
accept/reject bits and appended turn_status. A, centering, limits and capture gates
remain unchanged. The 18-degree offline comparison failed a full lossless rollout
at arm travel, so **hardware remains at 15 degrees**. Read
`docs/SWINGUP_QUALIFICATION_2026-11-04.md` and `swingup/DIAGNOSTICS.md`; upload once
and return one centered comparison log. Historical v2.1 results below are retained
for context; the current log format is 31-field #D, not 50 Hz CSV.

Status (2026-11-04): builds and local regression/simulation checks pass. **This revision has not been uploaded or tested on hardware.** `balance/balance.ino` is unchanged.

## Workflow

Reset with VMOT off, arm physically centered and pendulum hanging still. After calibration, turn VMOT on, press `z`, then `s`. A 3-second countdown leads to pumping, then **automatic balance A when the capture conditions hold**. No second monitor command or hand catch is required. Reset alone does not start motion. `x`/space immediately stops and disables the driver. A fall, timeout or fault is terminal: physically recenter, then `z`, `s` for a new attempt.

```powershell
pio run -e swingup -t upload
pio device monitor -f log2file
```

The user uploads/runs hardware. Keep the swept area clear; do not hand-catch during pumping. Send the complete log including the `swingup-v2.2-qualified` banner and final numeric M status records. To restore the established manual-catch build, upload `balance`.

## Hardware v2 feedback and v2.1 correction

The 15:32 v2 hardware run reached 16.37 degrees from upright and timed out; no balance transition. V2.1 fixes previous-tip carryover on asymmetric swings by tracking signed directional extrema, and backs off learned compensation after a full rotation. All limits, capture gates and A remain unchanged. See `SWINGUP_TEST_2026-11-04_1532.md` for measured evidence, the failing-then-passing regression and simulation limitations. Hardware v2.1 is not yet tested.

## Why v1 stalled

The supplied `logs/device-monitor-261104-151201.log` has 478 PUMP samples, maximum measured excursion 94.32 degrees from down and speed-based energy reaching 2.497. Near down (within 10 degrees), the estimated energy reaches 2.457, above the upright target 2. The old speed-energy law therefore reverses the energy input on some bottom passes. Distorted differentiated encoder speed is a plausible contributor given the weak magnet; the log alone cannot identify the true speed or prove that cause.

V2 estimates energy at pendulum turning points (`1-cos(peak)`) instead. It learns a bounded half-swing loss and chooses arm-velocity reversals around the bottom to add energy. The seed is a bounded square velocity command at the identified natural frequency for at most 1.5 seconds while amplitude is tiny. Pumping is restricted to the lower half; the arm slows as the pendulum approaches upright. Near-top overspeed braking removes excess energy. Following a detected full rotation, braking targets energy 2.02 rather than leaving the previous 2.2 residual that prevented lossless capture in simulation.

These are empirical control approximations, not a convergence proof. The simulator's friction assumptions do not establish real hinge behavior.

## Catch reuses the successful balance behavior

The BALANCE path reproduces `balance/balance.ino`:

- A gains: `[-27.5553, -600.4973, -14.2170, -44.9091]`, state `[theta,d,theta_dot,d_dot]`, `u=-Kx`.
- 200 Hz loop, 25 Hz derivative filter, trimmed-angle derivative and rejection of derivative spikes above 100 rad/s.
- Acceleration limited to **60 rad/s^2**, integrated arm speed limited to **pi rad/s (180 deg/s)**.
- Auto-trim after one second: `trim -= 0.023*theta*0.005`, bounded to +/-3 degrees.
- Tilt beyond 25 degrees stops the attempt. Pump centering and predictive braking do not modify A's balancing command.

Automatic capture requires trimmed tilt below 15 degrees, `abs(wn*d + filtered_rate) < 3 rad/s`, arm angle below 60 degrees, commanded arm speed at most pi rad/s, and requested A acceleration at most 75 rad/s^2 (allows mild initial clipping to the 60 cap). The switching gates are simulated heuristics, not a certified region of attraction. **Do not wait 0.2 seconds or reset speed/filter at this moving catch**: the original hand-catch did those because the pendulum and arm were held still. Here the measured state must be retained.

The original sketch stays independently buildable and unchanged. `tests/balance_parity.cpp` includes the actual original sketch and compares its BALANCE path against the swing-up core for 800 ticks, including speed/acceleration clipping, filtering and active trim. This locks the copied implementation against future drift.

## Limits and state transitions

| State | Behavior | Exit |
|---|---|---|
| IDLE (0) | Driver off | `z`, then `s` after valid calibration and down/still posture |
| COUNTDOWN (1) | Driver off for 3 seconds | Pump, or stop if posture/sensor invalid or cancelled |
| PUMP (3) | Bottom-timed reversals, amplitude energy, arm centering/braking | Automatic A capture, 20-second timeout, or fault |
| BALANCE (2) | Proven A law | Terminal fall/travel/sensor/timing/key stop |

**Pump-only experimental limits: 100 rad/s^2 and 250 deg/s.** The old A run used up to 100 rad/s^2; that does not validate this swing-up speed/acceleration combination. The current proven balance uses 60 and 180. Both sets of limits are printed at startup directly from constants.

PUMP applies stopping-distance braking about +/-75 degrees (`v^2/(2*100) + abs(v)*0.005`). BALANCE follows A, retaining the independent ISR guard. No pulse may cross +/-791 commanded steps (88.9875 degrees); that fault latches and disables the driver. The main-loop hard threshold is 89 degrees. There is no arm encoder, so none of this detects missed steps or physical displacement.

Calibration requires all 50 reads. Sensor faults invalidate calibration; successful reads alone do not rearm: hang still, `c`, `z`, `s`. I2C has a 2 ms timeout. A whole 5 ms missed deadline stops instead of catching up stale iterations. Duplicate starts do not reset an attempt. Cancellation wins over later commands in the same received batch. No automatic failed-catch retry or restart.

## Verification and limitations

```powershell
uv run --no-project --with ziglang python analysis/run_swingup_checks.py
pio run -e swingup -e balance
```

The runner explicitly undefines NDEBUG: Zig's optimized mode otherwise disabled C assertions. Core tests, complete-sketch stub tests (including automatic catch and terminal fall), and direct comparison with the actual balance sketch pass. Uno build: swingup 13,824 bytes flash / 529 bytes static RAM; balance 9,432 / 469.

Nonlinear simulations execute the exact firmware core, with quantization, filtering, step-rate deadband, centrifugal term and the ISR's commanded travel boundary. Each required scenario captured and held A for five seconds:

| Simulated plant | Capture after pumping starts |
|---|---:|
| Lossless | 19.385 s (close to 20 s timeout) |
| Dry friction 0.10 | 4.290 s |
| Dry friction 0.12 | 4.690 s |
| Dry 0.10 + viscous 0.02 | 2.180 s |
| Dry 0.10, coupling -15% / +15% | 4.490 / 3.945 s |
| Dry 0.10, 5-degree sine encoder errors (two shapes) | 4.410 / 3.840 s |
| Dry 0.10, 1-degree upright bias | 3.695 s |
| Equivalent viscous 0.06 | 2.920 s |

The **strict stiction stress case fails to hold**, stopping at commanded travel after capture (5.370 s). The regular dry-friction simulations deliberately use lower friction near upright to reflect successful balance runs; that is an assumption, not an independently measured parameter. Simulations do not model motor torque/missed steps, true magnetic distortion, MCU execution deadlines or interrupt concurrency. Hardware success remains unverified.

Generated CSV/binaries are under ignored `.pio/swingup-host/`; never call them measured data. Telemetry stays at 50 Hz with columns:

```text
t_ms,theta_deg,d_deg,u_rad_s2,trim_deg,mode,alpha_deg,alpha_dot_rad_s,v_cmd_rad_s,energy,braking
```

`energy` is now a turning-point estimate and may be stale between turns; it is not instantaneous kinetic energy. Old `compare_runs.py` assumes a different log schema and must not be used unchanged.

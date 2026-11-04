# Run A with current limits ? 2026-11-04, 14:46

Source: `device-monitor-261104-144647.log`, archived verbatim (including comments and blank lines) as `data/balance_A_current_limits_2026-11-04_1446.csv`.

## Result

Run A maintained hardware balance throughout the recorded BALANCE segment, from 12.972 to 22.551 s after reset (9.579 s, 959 samples). The log ends in mode 2; there is no STOP message. This is a successful bounded observation, not evidence of an indefinitely stable run.

| Metric | Result |
|---|---|
| Pendulum error RMS, excluding first 2 s | 1.19 degrees |
| Peak absolute pendulum error, entire segment | 4.95 degrees |
| Commanded arm angle range | -25.09 to +29.81 degrees |
| Acceleration command RMS, excluding first 2 s | 20.3 rad/s? |
| Peak absolute acceleration command | 60.0 rad/s? |
| Samples at acceleration cap | 35/959 (3.65%) |
| Final 2 s pendulum error RMS | 0.44 degrees |
| Final commanded arm angle / pendulum error | -1.24 / +0.09 degrees |
| Final trim | +0.22 degrees |
| Logged sample spacing | 9?11 ms (100 Hz telemetry) |

One disturbance-like excursion crosses 2 degrees at 3.900 s after balance begins, peaks at 4.95 degrees, and returns inside +/-1 degree in 1.969 s, remaining inside for at least 0.5 s. A physical tap is not marked in the log, so its cause cannot be established from telemetry alone. Recovery was independently checked using a full 500 ms window, in addition to `analysis/compare_runs.py`.

## Interpretation and limitations

The user identifies this as run A. Six-column telemetry matches the A-only sketch; the 60 rad/s? clipping matches the current acceleration cap. The current source sets V_MAX = pi rad/s (180 degrees/s), but firmware identity and the speed limit are not explicitly printed in this log. Arm angle is commanded step count, not an independent measurement; missed steps remain undetectable. Error d is relative to the trimmed upright reference.

This closes the handoff's pending first run A check with the reduced limits, subject to that firmware provenance limitation. Retain gain set A and the current limits; this run gives no reason to raise the acceleration cap. The older A disturbance run had 1.44-degree RMS, 5.6-degree peak and a larger arm range, but unequal manual disturbances and different durations prevent a controlled performance comparison. Repeatability remains to be established with longer runs.

No controller settings were changed and no firmware was uploaded during this analysis.

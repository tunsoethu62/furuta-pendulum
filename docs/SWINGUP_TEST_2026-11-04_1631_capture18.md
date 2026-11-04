# Swing-up hardware run 16:31 — v2.2-qualified-18 (capture +/-18 deg)

Raw log `data/device-monitor-261104-163105.log`; decoded
`data/swingup_diagnostic_2026-11-04_1631.csv` (449 records, 0 malformed, 0 dropped).
Compared with 16:27 (v2.2-qualified, capture 15 deg; decoded
`data/swingup_diagnostic_2026-11-04_1627.csv`, also 0 dropped, not previously reported)
and 16:06 (v2.1-diag). Analysis is of commanded arm motion and the weak-magnet
pendulum encoder; arm position is not measured.

## Result

Banner `swingup-v2.2-qualified-18`, `capture_deg=18.0`. Pumping for 20 s, terminal
timeout (reason 5), **no BALANCE**. The closest accepted tip was **45.07 deg from
upright** (negative lobe, -134.93 deg); the closest positive tip was 68.16 deg.
**The pendulum never entered the 25 deg region, so the capture-window change was
never exercised.** This run is not evidence for or against +/-18 deg capture.

| Run | Capture | Closest tip neg / pos (deg from upright) | Late (>8 s) median neg / pos | Commanded arm mean |
|---|---:|---:|---:|---:|
| 16:06 v2.1-diag | 15 | 27.59 / 46.10 | — | +16.9 deg |
| 16:27 v2.2-qualified | 15 | 26.16 / 50.40 | 34.24 / 58.16 | +18.6 deg |
| 16:31 v2.2-qualified-18 | 18 | 45.07 / 68.16 | 49.65 / 71.45 | +21.8 deg |

Capture checks only run when the pendulum is near upright; pumping code is identical
between 16:27 and 16:31. The difference between these two runs is therefore
run-to-run variation (arm drift, friction, encoder, initial conditions), not the
capture window. Qualified turns behaved as designed: 56 accepted T, 8 rejected J,
no startup loss jump.

## Findings

1. **Pump is saturated.** Planned Vp was at the 4.363 rad/s (250 deg/s) cap for the
   entire 16:31 run; learned loss hits its 1.2 cap after every positive tip. More
   loss compensation or a higher energy target cannot increase delivered energy.
2. **Positive lobe is consistently weaker** in all three runs (about 20-25 deg lower).
3. **The pumping oscillation is centered on a positive arm offset.** Commanded arm
   mean +17 to +22 deg. At negative reversals the arm is at median +29 to +35 deg, so
   the raw target is about -5.4 to -5.6 rad/s and is clipped to -4.36 (centering
   correction lost); at positive reversals the arm is at +7 to +10 deg and the
   target is reduced to about +4.0. Centering cannot remove the offset while the
   pump is at the speed cap, and the lopsided arm motion matches the lopsided swing.
4. Reversal delta-v median 7.5-8.0 rad/s versus nominal 2Vp = 8.73.

## Proposed next change (not implemented)

Pump-only, one change, A/capture gates/limits/timeout unchanged:
reserve centering headroom so the centering term is not clipped — limit the
pump component to V_MAX minus a centering margin (or saturate the pump component
before adding centering), with the goal of a near-zero mean arm angle and symmetric
reversals. Discriminating result: arm mean within a few degrees of zero and
positive/negative tip heights within a few degrees of each other. If tips become
symmetric but remain short, the speed/acceleration cap is the limit; raising
pump-only V_MAX/U_MAX is the next candidate, with missed-step risk (0.5 A drive,
no arm encoder) to be checked on hardware.

Keep +/-18 deg or return to 15 deg: no evidence either way from this run. The
earlier 18 deg lossless simulation hit the arm limit after an early catch.

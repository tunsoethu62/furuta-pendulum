# Hardware checks before any motor motion (no multimeter needed)

Order: **A. current limit (Vref)** → **B. coil pairs** → **C. step_test (microsteps + direction)**. Motion is only for step C, and only after A and B.

Golden rules: power **off** before connecting or changing any wire or jumper. **Never connect or disconnect the motor while VMOT (12 V) is on**: that can destroy the DRV8825.

## A. Read Vref with the Uno (`vref_reader/vref_reader.ino`)

1. Everything off. **Disconnect the motor** from the driver.
2. Wire Uno **A0** to the Vref test point (the metal centre/top of the trimpot) and make sure Uno **GND** is connected to the driver GND.
3. Upload `vref_reader.ino`, then switch **VMOT 12 V on** (the Pololu-style carrier has no logic-power pin; Vref exists only with VMOT on). Open the Serial Monitor at 115200.
4. Find the sense resistors (two tiny resistors next to the chip): `R100` = 0.100 Ω → current limit = 2 × Vref; `R200` = 0.200 Ω → current limit = Vref. Use the matching column.
5. Turn the trimpot **slowly** with a **plastic / non-conductive** screwdriver and watch the value. The sketch's A0 wire stays connected, so you do not need to hold a probe.
6. Target: the current limit should not exceed your motor's rated current per phase (read the label or datasheet of your exact 17HS4023 variant; I did not assume a value). Because the motor has run hot before, start **below** the rated current and raise it only if the motor misses steps.
7. VMOT off, wait for the driver LED/caps to settle, remove the A0 wire if you like, then reconnect the motor (power still off).

If the reading is ~0 V: check the A0 wire, the common GND, that VMOT is on, and that the board's SLEEP and RESET pins are tied high as in your normal wiring (the reference comes from the driver's internal 3.3 V supply). Accuracy is about ±5 % (USB 5 V is not exact).

## B. Find the coil pairs without a meter

The motor has 4 wires = 2 coils. Wrong pairing is a common cause of buzzing and heat.

- Motor **disconnected from the driver**. Spin the shaft by hand and feel the normal resistance.
- Twist two wires together and spin again. If the shaft becomes clearly **harder to turn (cogging)**, those two wires are one coil. Untwist, try another pair. The two wires left over are the other coil.
- (Alternative: put an LED across two wires and spin the shaft fast; it flickers on a coil pair.)
- Connect one coil to `1A/1B` (often labelled A1/A2) and the other to `2A/2B` (B1/B2). Swapping the two wires **within** one coil only reverses direction, which is harmless.
- Signs of **wrong pairing** when running: buzzing or vibrating, jittering back and forth, little or no rotation, fast heating. Turn off and recheck.

## C. Steps per revolution and direction (`step_test/step_test.ino`)

Command-gated, bounded (≤ 400 pulses per move), slow (100 pulses/s), driver disabled between moves, `x` or space stops at once. Tape a mark on the arm and the base, then follow the instructions at the top of the sketch: 200 pulses give 360° at full step, 22.5° at 1/16, and so on. Then set `MICROSTEPS` in `motor_testing.ino` and `acceleartion.ino` (they refuse to compile until it is set).

## What these checks can and cannot verify (no arm encoder)

| Item | How | Verified? |
|---|---|---|
| Current limit | Vref via A0 + sense-resistor marking | Yes, ±5 %, given the marking is read correctly |
| Coil pairs | cogging test, then smooth rotation | Yes (by feel/observation) |
| Microstep mode / steps per rev | 200-pulse move vs tape mark, full-turn return | Yes, by eye; ±a few degrees |
| Direction sign of `f` (DIR HIGH) | watch from above | Yes, by eye |
| Missed steps during balancing / fast moves | needs arm angle measurement | **No.** Without an arm encoder, arm angle is only the commanded pulse count. A full-turn return to the mark at test speed is the only check |
| Arm angle while the driver is disabled | — | **No.** The arm can be pushed freely; the counter does not know |

The pendulum AS5600 cannot measure the arm: it sits on the arm and measures the pendulum relative to the arm. After an arm move it only shows the pendulum swinging (useful later for the free-decay damping test, which needs the arm still, and for checking that the pendulum sign is consistent with the arm direction).

## Questions still open
1. Exact motor label / rated current of the 17HS4023 variant.
2. Sense resistor marking on the driver (R100 or R200).
3. The Vref value you settle on.
4. Result of step_test: angle for 200 pulses (→ microstep mode) and which way `f` turns (seen from above).
5. Cable wrapping: the arm can rotate freely, but does the pendulum encoder cable limit how many turns are safe? If so, choose a software limit; I will not invent one.

## Results (step_test, reported by the user, 2026-11-04)
- Microstepping **1/16 confirmed**: 800 pulses = exactly 90° and 4 × 200 pulses back returned exactly to 0° (100 pulses/s, no missed steps seen). So **3200 pulses/rev, 8.889 pulses per degree** (0.1125° per pulse).
- Direction: with `INVERT_DIR = true` in step_test, `f` turns the arm **clockwise seen from above**. In hardware terms: **DIR LOW = clockwise, DIR HIGH = counter-clockwise** (seen from above). motor_testing and acceleartion still use DIR HIGH for their "+" direction (= counter-clockwise); pick one sign convention for theta before writing the controller.
- Not tested: a full 360° turn (wiring), higher speeds, missed steps under load.

## Pendulum encoder check (as5600_read, user log 2026-11-04)
- Magnet field too weak: STATUS 0x13 (ML set, MD clear), AGC 128 = maximum (3.3 V mode), magnitude 155-245 and varying with angle (suggests the magnet is off-centre). User chose to continue without changing the magnet; accuracy is reduced.
- Still: noise at rest about ±1-2 counts (±0.1-0.2°); full slow turn tracked smoothly, unwrap worked across 4095 -> 0.
- Held positions (counts): down 1280 | ~90° 2248 | up ~3316 | ~270° 4285 (unwrapped). Steps: 968, 1068, 969 (ideal 1024), i.e. about ±5° per quarter turn (sensor nonlinearity and/or hand-held positions).
- Down -> up = 2036 counts (178.9°). Gravity-defined down + 2048 = 3328, within ~12 counts (~1°) of the hand-held up reading 3316.
- Hanging-down read 1295 in an earlier session and 1280 now (~1.3° shift): re-read the down position at the start of every session and compute alpha from it (alpha_up_ref = down + 2048).
- Sign: rotating the pendulum in this test increased counts. Viewing side for "clockwise" still to be fixed in the model.

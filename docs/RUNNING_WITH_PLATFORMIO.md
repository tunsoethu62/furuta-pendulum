# Build, upload and monitor with PlatformIO (VS Code)

Each sketch is a PlatformIO environment (`platformio.ini`). The sketch folders are unchanged and still work in the Arduino IDE; `pio/<sketch>/main.cpp` just includes the `.ino`. Edit the `.ino` files.

## 0. One-time setup
The VS Code extension *PlatformIO IDE* is installed, but its core was **not** installed yet on this PC (no `%USERPROFILE%\.platformio\penv`). Open this folder in VS Code (`File > Open Folder > ...\furuta-pendulum`) and let PlatformIO finish installing (status bar; first build also downloads the AVR toolchain, needs internet). Then use the **PlatformIO terminal** (PlatformIO sidebar > Quick Access > Miscellaneous > *New Terminal*), where `pio` is on the PATH.

## 1. Build only (safe, the board is not touched)
```
pio run -e step_test
```
Expect `SUCCESS`. Build all: `pio run`.

## 2. Upload (the board will run the sketch: do this only when you are ready)
Before uploading step_test: arm free to turn, hands and cables clear, VMOT 12 V **off** at first.
```
pio device list                  # find the Uno's COM port
pio run -e step_test -t upload   # add  --upload-port COM3  if it picks the wrong port
```
In VS Code you can also pick the env in the status bar and click → (Upload). Close any other serial monitor first (Arduino IDE, another terminal): the port can only be open once.

## 3. Serial monitor
```
pio device monitor               # 115200 from platformio.ini; Ctrl+C quits
```
Opening the monitor resets the Uno. Keys are sent immediately (no Enter needed); typed characters are not echoed.

### What step_test should print
```
step_test: driver disabled. Commands: 1/2/3 size, f, b, z, p, x=STOP
move size = 25 pulses | net commanded pulses = 0 | driver disabled
```
Then switch VMOT on. Type `3` (size 200), then `f`:
```
move size = 200 pulses | net commanded pulses = 0 | driver disabled
f 200 pulses
done | net commanded pulses = 200
```
The move takes 2 s. With 1/16 microstepping the arm should turn **22.5°**. Note the direction seen from above. `x` or space stops at once. To confirm 1/16: put the arm on the tape mark, type `z`, then `f` 16 times (16 × 200 = 3200 pulses): the arm must be back exactly on the mark.

## 4. When it does not work
| Symptom | Check |
|---|---|
| Upload: port not found / access denied / `not in sync` | `pio device list`; close other serial monitors; try another USB cable (some are charge-only) or port; clones with CH340 need the CH340 driver; set `--upload-port COMx` |
| Monitor shows nothing | reset button; right port; baud 115200; upload finished before opening the monitor |
| Garbled characters | baud mismatch: must be 115200 |
| Commands do nothing | keys go to the monitor window? (click into it) ; a move is running (`busy`) |
| Prints `done` but the motor does not move | VMOT on? common GND Uno–driver; EN wire on D5; SLEEP and RESET tied high; motor connector seated; current limit not set too low |
| Motor buzzes / vibrates / jitters instead of turning | coil pairing (see HARDWARE_CHECKS.md B); current too low; loose wire. Stop with `x`, VMOT off before touching wires |
| Motor or driver gets hot | the driver is enabled only during moves here; if hot anyway, lower Vref; heat sink on the driver |
| Arm turns 360° for 200 pulses (or another angle than 22.5°) | microstep jumpers differ from M0=GND, M1=GND, M2=5V: tell us the angle, MICROSTEPS will be corrected |

## Other environments
`pio run -e vref_reader -t upload` (motor disconnected), `pio run -e as5600_read -t upload` (read-only encoder log), `motor_testing`, `acceleartion` (both move the motor on command; MICROSTEPS = 16 set from the reported jumpers).

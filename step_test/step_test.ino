/*
  step_test.ino  -  find steps-per-revolution (microstep mode) and direction sign, without an arm encoder

  Pins: STEP = D3, DIR = D4, EN = D5 (active LOW).  Motor supply 12 V.  Current limit set FIRST (vref_reader).

  SAFETY
    - The driver is DISABLED (EN HIGH) except while a move is running.
    - Type  x  (or a space) at ANY time: the move stops at once and the driver is disabled.
    - Every move is bounded (at most MAX_PULSES pulses) and slow (PULSE_RATE_HZ).
    - Nothing moves until you type a command.

  Serial Monitor: 115200 baud, line ending "No line ending" (or any; extra characters are ignored).

  Commands
    1 / 2 / 3   choose move size: 25, 50 or 200 STEP pulses
    f           forward  by the chosen number of pulses (DIR level set by INVERT_DIR)
    b           backward by the chosen number of pulses
    z           reset the pulse counter to 0 (do this when the arm is on your tape mark)
    p           print status
    x / space   STOP and disable driver

  HOW TO USE (find the microstep mode)
    Put a tape mark on the arm and on the base. Choose 3 (200 pulses) and type f once.
    Arm angle for 200 pulses:  full step = 360 deg (back on the mark) | 1/2 = 180 | 1/4 = 90 |
                               1/8 = 45 | 1/16 = 22.5 | 1/32 = 11.25
    The row that matches is your microstep mode (microsteps = 360 / angle seen).
    Check: 200 x microsteps pulses in total must bring the arm exactly back to the mark
    (e.g. 1/16 -> 3200 pulses = 16 x "f" with size 200). Landing short = missed steps or a wrong guess.
    NOTE: the driver is disabled between moves, so the arm can be pushed by hand; keep it still between moves,
    otherwise the pulse counter no longer matches where the arm really is.
    DIRECTION: note whether f turns the arm clockwise or counter-clockwise seen from above; that fixes the sign
    of theta for f (DIR HIGH) in your model.
*/

const uint8_t STEP_PIN = 3;
const uint8_t DIR_PIN  = 4;
const uint8_t EN_PIN   = 5;

// Direction fix: on this build "f" turned the arm the opposite way to what you expect.
// true = swap the DIR level so "f" turns the expected way. (Swapping the two wires of ONE coil, power off, does the same in hardware.)
const bool INVERT_DIR = true;

const uint16_t PULSE_RATE_HZ = 100;                          // slow: 100 pulses per second
const uint32_t PULSE_PERIOD_US = 1000000UL / PULSE_RATE_HZ;
const uint16_t MAX_PULSES = 400;                              // hard upper bound per move
const uint16_t SIZES[3] = {25, 50, 200};

uint16_t moveSize = 25;        // pulses per f/b command
int32_t  netPulses = 0;        // + for f, - for b (commanded, NOT measured)
uint16_t remaining = 0;        // pulses left in the current move
int8_t   moveDir = 0;          // +1 or -1 while moving
uint32_t nextPulseUs = 0;

void disableDriver(const __FlashStringHelper *why) {
  remaining = 0; moveDir = 0;
  digitalWrite(EN_PIN, HIGH);
  Serial.print(F("DISABLED: ")); Serial.println(why);
}

void startMove(int8_t dir) {
  if (remaining) { Serial.println(F("busy: wait or press x")); return; }
  uint16_t n = moveSize > MAX_PULSES ? MAX_PULSES : moveSize;
  bool dirHigh = (dir > 0) != INVERT_DIR;
  digitalWrite(DIR_PIN, dirHigh ? HIGH : LOW);
  digitalWrite(EN_PIN, LOW);                 // enable only for the move
  delayMicroseconds(5);                      // DIR setup time (datasheet minimum is well below this)
  moveDir = dir; remaining = n;
  nextPulseUs = micros() + 2000;             // short settle after enabling
  Serial.print(dir > 0 ? F("f ") : F("b ")); Serial.print(n); Serial.println(F(" pulses"));
}

void printStatus() {
  Serial.print(F("move size = ")); Serial.print(moveSize);
  Serial.print(F(" pulses | net commanded pulses = ")); Serial.print(netPulses);
  Serial.print(F(" | driver ")); Serial.println(digitalRead(EN_PIN) ? F("disabled") : F("ENABLED"));
}

void setup() {
  digitalWrite(EN_PIN, HIGH); pinMode(EN_PIN, OUTPUT);   // HIGH before OUTPUT: never enabled by accident
  digitalWrite(STEP_PIN, LOW); pinMode(STEP_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("step_test: driver disabled. Commands: 1/2/3 size, f, b, z, p, x=STOP"));
  printStatus();
}

void loop() {
  // 1) Commands are checked on EVERY loop pass, also during a move, so x always works.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X' || c == ' ') disableDriver(F("stop command"));
    else if (c >= '1' && c <= '3') { moveSize = SIZES[c - '1']; printStatus(); }
    else if (c == 'f' || c == 'F') startMove(+1);
    else if (c == 'b' || c == 'B') startMove(-1);
    else if (c == 'z' || c == 'Z') { if (!remaining) { netPulses = 0; Serial.println(F("counter = 0")); } }
    else if (c == 'p' || c == 'P') printStatus();
  }

  // 2) Non-blocking pulse generation: one pulse every PULSE_PERIOD_US.
  if (remaining && (int32_t)(micros() - nextPulseUs) >= 0) {
    nextPulseUs += PULSE_PERIOD_US;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);                    // pulse width (DRV8825 minimum ~1.9 us)
    digitalWrite(STEP_PIN, LOW);
    netPulses += moveDir;
    if (--remaining == 0) {
      digitalWrite(EN_PIN, HIGH);            // disable after the move: no holding current, less heat
      Serial.print(F("done | net commanded pulses = ")); Serial.println(netPulses);
    }
  }
}

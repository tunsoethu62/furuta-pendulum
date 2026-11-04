/*
  motion_core.ino  -  Furuta step 3: background step generation (Timer1) + fixed-rate loop + pendulum logging

  This is the base the balance controller will be built on. It does NOT balance anything yet.
  It lets you check, with the motor really running, that:
    - Timer1 makes smooth STEP pulses while the main loop keeps reading the encoder at a fixed rate,
    - the arm follows a commanded velocity / acceleration (no lost steps at these settings),
    - the pendulum reacts to arm motion (data for "validate the model with a small open-loop test").

  Hardware (confirmed earlier): STEP=D3, DIR=D4, EN=D5 (active LOW), DRV8825 at 1/16 -> 3200 steps/rev,
  INVERT_DIR=true makes positive steps = CLOCKWISE seen from above. AS5600 on the pendulum, I2C A4/A5.

  SAFETY
    - Nothing moves until you type a command. x or SPACE = emergency stop: pulses stop at once, driver disabled.
    - Software arm limit ARM_LIMIT_DEG (default +-90 deg, the range you already tested; the wiring does not
      allow full turns). Crossing it = emergency stop. Change it only if your wiring allows more.
    - Speed and acceleration are capped (V_MAX, A_MAX). Start with these defaults.
    - The driver is disabled when idle (no holding current, less heat). The arm is then free: do not push it,
      or the step counter no longer matches the real arm angle (there is no arm encoder).

  Serial Monitor 115200. Commands (single keys, no Enter needed):
    x / space  EMERGENCY STOP (disable)          s  smooth stop (ramp to 0, then disable)
    z          set arm position = 0 here (arm must be still)
    c          re-calibrate pendulum "down" (pendulum hanging still)
    h          go home to arm position 0 (ramped)
    1          velocity test: +60 deg/s for 1 s, pause, back to 0 (start near 0: press z first)
    2          sine test: arm +-15 deg at 1 Hz for 5 cycles (pendulum swings; log it for model validation)
    3          sine test: arm +-8 deg at 2 Hz (near the pendulum's 1.9 Hz resonance) for 5 cycles
    l          toggle logging on/off (100 Hz CSV)
    p          print status

  Log columns (100 Hz): t_ms, theta_deg, omega_cmd_deg_s, alpha_deg, mode
    mode: 0 idle, 1 velocity test, 2 sine test, 3 homing, 4 stopping
    theta_deg      arm angle from COMMANDED steps (not measured)
    omega_cmd      commanded arm speed
    alpha_deg      pendulum angle, 0 = hanging down (calibrated at start / with c), sign as the encoder counts
*/

#include <Wire.h>

// ---------------- pins & mechanics ----------------
const uint8_t STEP_PIN = 3, DIR_PIN = 4, EN_PIN = 5;
const bool    INVERT_DIR = true;                 // confirmed with step_test: f (positive) = clockwise from above
const float   STEPS_PER_DEG = 3200.0 / 360.0;    // 1/16 microstepping

// ---------------- limits (conservative defaults) ----------------
const float ARM_LIMIT_DEG = 90.0;    // software limit, +- from position 0
const float V_MAX_DEG_S   = 180.0;   // max arm speed
const float A_MAX_DEG_S2  = 1500.0;  // max arm acceleration
const float V_MIN_STEPS   = 31.0;    // below this the timer cannot run (16-bit, prescaler 8) -> treated as stop

// ---------------- loop timing ----------------
const uint32_t LOOP_US = 5000;       // 200 Hz control/sample loop
uint32_t nextLoopUs;

// ---------------- AS5600 ----------------
const uint8_t AS5600 = 0x36;
int16_t  lastRaw = -1;
int32_t  alphaCounts = 0;     // unwrapped
float    alphaDownCounts = 0; // calibration: hanging-down reading

// ---------------- step generator state (shared with the ISR) ----------------
volatile int32_t stepPos = 0;     // commanded arm position in steps (+ = clockwise)
volatile int8_t  stepDir = 0;     // +1 / -1 while the timer runs, 0 = stopped
float vCmd = 0;                   // current commanded speed, steps/s (after the ramp)
float vTarget = 0;                // wanted speed, steps/s

// ---------------- test modes ----------------
enum Mode : uint8_t { IDLE, VEL_TEST, SINE_TEST, HOMING, STOPPING };
Mode mode = IDLE;
uint32_t modeStartMs = 0;
float sineAmpSteps = 0, sineW = 0;   // sine test parameters
uint8_t sineCycles = 0;
bool logging = true;
uint8_t logDiv = 0;

// ================= Timer1 step generation =================
// Timer1 in CTC mode, prescaler 8 -> 2 MHz ticks. One compare match = one STEP pulse.
ISR(TIMER1_COMPA_vect) {
  PORTD |= _BV(3);                         // STEP high (D3 = PD3)
  stepPos += stepDir;
  __builtin_avr_delay_cycles(40);          // ~2.5 us pulse (DRV8825 needs >= 1.9 us)
  PORTD &= ~_BV(3);                        // STEP low
}

void timerStop() {
  TIMSK1 &= ~_BV(OCIE1A);
  stepDir = 0;
}

// Set the step rate (steps/s, signed). Called from the 200 Hz loop only.
void setStepRate(float v) {
  if (fabs(v) < V_MIN_STEPS) { timerStop(); return; }
  int8_t dir = v > 0 ? 1 : -1;
  if (dir != stepDir) {                    // direction change only from standstill (the ramp guarantees a stop tick)
    timerStop();
    digitalWrite(DIR_PIN, ((dir > 0) != INVERT_DIR) ? HIGH : LOW);
    delayMicroseconds(2);                  // DIR setup time before the next pulse
  }
  uint32_t top = (uint32_t)(2000000.0 / fabs(v)) - 1;
  if (top > 65535) top = 65535;
  noInterrupts();
  OCR1A = (uint16_t)top;
  if (TCNT1 > OCR1A) TCNT1 = 0;            // avoid a 32 ms wait when the period gets shorter
  stepDir = dir;
  TIMSK1 |= _BV(OCIE1A);
  interrupts();
}

int32_t readStepPos() { noInterrupts(); int32_t p = stepPos; interrupts(); return p; }

// ================= helpers =================
void driverEnable(bool on) { digitalWrite(EN_PIN, on ? LOW : HIGH); }

void emergencyStop(const __FlashStringHelper *why) {
  timerStop(); vCmd = vTarget = 0; mode = IDLE; driverEnable(false);
  Serial.print(F("# E-STOP: ")); Serial.println(why);
}

bool readRaw(int16_t &raw) {
  Wire.beginTransmission(AS5600); Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  raw = ((int16_t)(hi & 0x0F) << 8) | lo;
  return true;
}

void updateAlpha() {
  int16_t raw;
  if (!readRaw(raw)) return;               // keep the last value on a failed read
  if (lastRaw < 0) alphaCounts = raw;
  else {
    int16_t d = raw - lastRaw;
    if (d > 2048) d -= 4096;
    if (d < -2048) d += 4096;
    alphaCounts += d;
  }
  lastRaw = raw;
}

void calibrateDown() {
  float sum = 0;
  for (uint8_t i = 0; i < 50; i++) { updateAlpha(); sum += alphaCounts; delay(5); }   // blocking is fine: motor idle
  alphaDownCounts = sum / 50;
  Serial.print(F("# pendulum down = ")); Serial.print(alphaDownCounts, 1); Serial.println(F(" counts"));
}

float alphaDeg() { return (alphaCounts - alphaDownCounts) * (360.0 / 4096.0); }

void startMode(Mode m) {
  if (mode != IDLE) { Serial.println(F("# busy: s (smooth stop) or x first")); return; }
  driverEnable(true); delay(2);
  mode = m; modeStartMs = millis();
}

void printStatus() {
  Serial.print(F("# mode=")); Serial.print(mode);
  Serial.print(F(" theta_deg=")); Serial.print(readStepPos() / STEPS_PER_DEG, 2);
  Serial.print(F(" alpha_deg=")); Serial.print(alphaDeg(), 2);
  Serial.print(F(" driver=")); Serial.println(digitalRead(EN_PIN) ? F("off") : F("ON"));
}

// ================= setup =================
void setup() {
  digitalWrite(EN_PIN, HIGH); pinMode(EN_PIN, OUTPUT);      // disabled before becoming an output
  digitalWrite(STEP_PIN, LOW); pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000);

  // Timer1: CTC (WGM12), prescaler 8 (CS11), interrupt off until a speed is set
  noInterrupts();
  TCCR1A = 0; TCCR1B = _BV(WGM12) | _BV(CS11); TCNT1 = 0; OCR1A = 65535; TIMSK1 = 0;
  interrupts();

  Serial.println(F("# motion_core: driver disabled. Keep the pendulum hanging still for calibration..."));
  calibrateDown();
  Serial.println(F("# keys: x/space E-STOP, s stop, z zero arm, c calib, h home, 1 vel, 2 sine 1Hz, 3 sine 2Hz, l log, p status"));
  Serial.println(F("t_ms,theta_deg,omega_cmd_deg_s,alpha_deg,mode"));
  nextLoopUs = micros();
}

// ================= main loop =================
void handleKey(char c) {
  switch (c) {
    case 'x': case 'X': case ' ': emergencyStop(F("key")); break;
    case 's': case 'S': if (mode != IDLE) mode = STOPPING; break;
    case 'z': case 'Z': if (mode == IDLE) { noInterrupts(); stepPos = 0; interrupts(); Serial.println(F("# arm zero set")); } break;
    case 'c': case 'C': if (mode == IDLE) { calibrateDown(); nextLoopUs = micros(); } else Serial.println(F("# stop first")); break;
    case 'h': case 'H': startMode(HOMING); break;
    case '1': startMode(VEL_TEST); break;
    case '2': sineAmpSteps = 15 * STEPS_PER_DEG; sineW = 2 * PI * 1.0; sineCycles = 5; startMode(SINE_TEST); break;
    case '3': sineAmpSteps = 8 * STEPS_PER_DEG; sineW = 2 * PI * 2.0; sineCycles = 5; startMode(SINE_TEST); break;
    case 'l': case 'L': logging = !logging; break;
    case 'p': case 'P': printStatus(); break;
  }
}

void loop() {
  while (Serial.available()) handleKey(Serial.read());

  uint32_t now = micros();
  if ((int32_t)(now - nextLoopUs) < 0) return;
  nextLoopUs += LOOP_US;
  const float dt = LOOP_US * 1e-6;

  updateAlpha();
  int32_t pos = readStepPos();
  float posDeg = pos / STEPS_PER_DEG;
  float t = (millis() - modeStartMs) * 1e-3;

  // ---- software arm limit ----
  if (mode != IDLE && fabs(posDeg) > ARM_LIMIT_DEG) emergencyStop(F("arm limit"));

  // ---- choose the target speed for this tick ----
  const float VMAX = V_MAX_DEG_S * STEPS_PER_DEG, AMAX = A_MAX_DEG_S2 * STEPS_PER_DEG;
  switch (mode) {
    case IDLE: vTarget = 0; break;
    case VEL_TEST:   // +60 deg/s for 1 s, pause 0.5 s, then home
      if (t < 1.0) vTarget = 60 * STEPS_PER_DEG;
      else if (t < 1.5) vTarget = 0;
      else { mode = HOMING; }
      break;
    case SINE_TEST: { // theta_ref = A sin(w t): feed-forward speed + small position correction
      float ref = sineAmpSteps * sin(sineW * t);
      vTarget = sineAmpSteps * sineW * cos(sineW * t) + 5.0 * (ref - pos);
      if (t > sineCycles * 2 * PI / sineW) mode = HOMING;
      break; }
    case HOMING:     // proportional approach to 0 with a minimum creep speed (the timer cannot run slower)
      if (pos == 0) { mode = STOPPING; vTarget = 0; }
      else vTarget = (pos > 0 ? -1 : 1) * max(4.0f * abs(pos), 40.0f);
      break;
    case STOPPING: vTarget = 0; break;
  }
  vTarget = constrain(vTarget, -VMAX, VMAX);

  // ---- acceleration-limited ramp; never jump across zero in one tick ----
  if (mode != IDLE) {
    float dv = constrain(vTarget - vCmd, -AMAX * dt, AMAX * dt);
    float vNew = vCmd + dv;
    if ((vCmd > 0 && vNew < 0) || (vCmd < 0 && vNew > 0)) vNew = 0;   // stop tick before reversing
    vCmd = vNew;
    setStepRate(vCmd);
    if (mode == STOPPING && fabs(vCmd) < V_MIN_STEPS) {
      timerStop(); vCmd = 0; mode = IDLE; driverEnable(false);
      Serial.println(F("# stopped, driver off")); printStatus();
    }
  }

  // ---- 100 Hz log ----
  if (logging && (++logDiv & 1)) {
    Serial.print(millis());              Serial.print(',');
    Serial.print(posDeg, 2);             Serial.print(',');
    Serial.print(vCmd / STEPS_PER_DEG, 1); Serial.print(',');
    Serial.print(alphaDeg(), 2);         Serial.print(',');
    Serial.println(mode);
  }
}

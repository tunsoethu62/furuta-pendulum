/*
  balance.ino  -  Furuta step 4: LQR balance near upright (catch by hand, no swing-up yet)

  Model and gains: analysis/lqr_design.py (run it again if you change parameters, then paste the 4 gains below).
    Stepper = arm ACCELERATION input u = theta''. Pendulum: w_n = 12.08 rad/s, l_eq = 6.72 cm, L_r = 7 cm, zeta 0.06.
    State x = [theta, d, theta', d']  (rad, rad/s):  theta = arm angle (+ = clockwise from above, from commanded steps),
    d = pendulum angle from UPRIGHT (from the AS5600, same direction as its counts).
    Control: u = -(K_THETA*theta + K_ALPHA*d + K_THETA_DOT*theta' + K_ALPHA_DOT*d'), integrated to a speed command.
  Simulated (nominal and with +-20 % model error): balances from 5 deg; a 1 deg upright-calibration error is removed
  by the slow auto-trim. Hardware: balanced 10-17 s with tap disturbances (docs/BALANCE_TEST_2026-11-04.md,
  docs/WEIGHTING_COMPARISON_2026-11-04.md); those runs used U_MAX 100 / V_MAX 360 deg/s, the current limits are untested.

  SAFETY
    - x or SPACE = emergency stop at any time (pulses stop, driver off).
    - Automatic stop if the pendulum leaves +-25 deg around upright, or the arm passes +-90 deg (your wiring range).
    - The arm can move fast (up to 180 deg/s) when it catches the pendulum: keep the other hand, cables and the
      USB lead clear of the arm. Hold the pendulum by its tip, lightly.

  HOW TO USE (Serial Monitor 115200)
    1. VMOT off, arm on its 0 mark, pendulum hanging still -> reset/upload. It calibrates "down" (upright = down + 180 deg).
    2. VMOT on. Press z (arm zero here).
    3. Press a (ARM): driver on, arm holds still. Lift the pendulum by hand slowly to upright and hold it still.
       When it is within +-4 deg of upright for 0.2 s the controller takes over ("# BALANCE"): let go gently.
    4. Watch the arm: if it drifts to one side and stays there, the upright reference is off; the auto-trim corrects
       it within a few seconds. You can also trim by hand with [ and ] (0.2 deg steps) while balancing.
    5. x to stop. The pendulum then falls: that is expected.
  Keys: a arm/catch | x/space E-STOP | z zero arm | c re-calibrate down | [ ] trim | l log on/off | p status
  Log (100 Hz): t_ms, theta_deg, d_deg, u_rad_s2, trim_deg, mode   (mode 0 idle, 1 armed, 2 balance)
*/

#include <Wire.h>

// ---------------- LQR gains: weighting A (analysis/lqr_design.py A) ----------------
//   Q = diag(1/0.5^2, 1/0.08^2, 1/3^2, 1/5^2), R = 1/15^2. Chosen after the A/B comparison
//   (docs/WEIGHTING_COMPARISON_2026-11-04.md); the stiffer set B was removed.
const float K_THETA = -27.5553, K_ALPHA = -600.4973, K_THETA_DOT = -14.2170, K_ALPHA_DOT = -44.9091;

// ---------------- hardware ----------------
const uint8_t STEP_PIN = 3, DIR_PIN = 4, EN_PIN = 5;
const bool    INVERT_DIR = true;                    // positive steps = clockwise from above (step_test)
const float   STEPS_PER_RAD = 3200.0 / (2 * PI);    // 1/16 microstepping
const uint8_t AS5600 = 0x36;
const float   RAD_PER_COUNT = 2 * PI / 4096.0;

// ---------------- limits ----------------
const float U_MAX       = 60.0;             // rad/s^2 cap: simulation peaks at 56; untested motor limits (Arjarn review)
const float V_MAX       = PI;               // rad/s (180 deg/s): linear model invalid at 2*pi (centrifugal term ~27 %)
const float ARM_LIMIT   = radians(90);      // software arm limit
const float ENGAGE_D    = radians(4);       // catch window around upright
const float DROP_D      = radians(25);      // give up beyond this
const float V_MIN_STEPS = 31.0;             // slowest rate Timer1 (prescaler 8) can make
const float KI_TRIM     = 0.023;            // auto-trim rate (rad of trim per rad*s of arm offset)
const float TRIM_MAX    = radians(3);

// ---------------- timing / filters ----------------
const uint32_t LOOP_US = 5000;              // 200 Hz
const float DT = LOOP_US * 1e-6;
const float TAU_D = 1.0 / (2 * PI * 25);    // d' low-pass, 25 Hz (same as the simulation)
uint32_t nextLoopUs, overruns = 0;

// ---------------- state ----------------
volatile int32_t stepPos = 0;
volatile int8_t  stepDir = 0;
float vCmd = 0;                             // arm speed command, rad/s
int16_t lastRaw = -1;
int32_t alphaCounts = 0;
float downCounts = 0, trim = 0, dPrev = 0, dDotF = 0;
bool dPrevValid = false;
uint8_t readFails = 0;
enum Mode : uint8_t { IDLE, ARMED, BALANCE };
Mode mode = IDLE;
uint16_t engageTicks = 0;
uint32_t balanceStartMs = 0;
bool logging = true;
uint8_t logDiv = 0;
float uLast = 0;

// ================= step generator (Timer1, same as motion_core) =================
ISR(TIMER1_COMPA_vect) {
  PORTD |= _BV(3);
  stepPos += stepDir;
  __builtin_avr_delay_cycles(40);           // ~2.5 us STEP pulse
  PORTD &= ~_BV(3);
}
void timerStop() { TIMSK1 &= ~_BV(OCIE1A); stepDir = 0; }
void setStepRate(float v) {                 // v in steps/s, signed
  if (fabs(v) < V_MIN_STEPS) { timerStop(); return; }
  int8_t dir = v > 0 ? 1 : -1;
  if (dir != stepDir) {
    timerStop();
    digitalWrite(DIR_PIN, ((dir > 0) != INVERT_DIR) ? HIGH : LOW);
    delayMicroseconds(2);
  }
  uint32_t top = (uint32_t)(2000000.0 / fabs(v)) - 1;
  if (top > 65535) top = 65535;
  noInterrupts();
  OCR1A = (uint16_t)top;
  if (TCNT1 > OCR1A) TCNT1 = 0;
  stepDir = dir;
  TIMSK1 |= _BV(OCIE1A);
  interrupts();
}
int32_t readStepPos() { noInterrupts(); int32_t p = stepPos; interrupts(); return p; }

// ================= encoder =================
bool updateAlpha() {
  Wire.beginTransmission(AS5600); Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  int16_t raw = ((int16_t)(hi & 0x0F) << 8) | lo;
  if (lastRaw < 0) alphaCounts = raw;
  else {
    int16_t d = raw - lastRaw;
    if (d > 2048) d -= 4096;
    if (d < -2048) d += 4096;
    alphaCounts += d;
  }
  lastRaw = raw;
  return true;
}
void calibrateDown() {
  float sum = 0; uint8_t n = 0;
  for (uint8_t i = 0; i < 50; i++) { if (updateAlpha()) { sum += alphaCounts; n++; } delay(5); }
  if (n) downCounts = sum / n;
  Serial.print(F("# down = ")); Serial.print(downCounts, 1); Serial.print(F(" counts, upright = ")); Serial.println(downCounts + 2048, 1);
  nextLoopUs = micros();
}
// pendulum angle from upright, wrapped to -pi..pi, minus the trim
float dFromUpright() {
  float d = (alphaCounts - (downCounts + 2048)) * RAD_PER_COUNT;
  while (d > PI) d -= 2 * PI;
  while (d < -PI) d += 2 * PI;
  return d - trim;
}

// ================= modes =================
void driverEnable(bool on) { digitalWrite(EN_PIN, on ? LOW : HIGH); }
void stopAll(const __FlashStringHelper *why) {
  timerStop(); vCmd = 0; mode = IDLE; engageTicks = 0; driverEnable(false);
  Serial.print(F("# STOP: ")); Serial.println(why);
}
void printStatus() {
  Serial.print(F("# mode=")); Serial.print(mode);
  Serial.print(F(" theta_deg=")); Serial.print(degrees(readStepPos() / STEPS_PER_RAD), 2);
  Serial.print(F(" d_deg=")); Serial.print(degrees(dFromUpright()), 2);
  Serial.print(F(" trim_deg=")); Serial.print(degrees(trim), 2);
  Serial.print(F(" overruns=")); Serial.println(overruns);
}

void setup() {
  digitalWrite(EN_PIN, HIGH); pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW); pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000);
  noInterrupts();
  TCCR1A = 0; TCCR1B = _BV(WGM12) | _BV(CS11); TCNT1 = 0; OCR1A = 65535; TIMSK1 = 0;
  interrupts();
  Serial.println(F("# balance: driver off. Pendulum must hang still for calibration..."));
  calibrateDown();
  Serial.println(F("# keys: a arm/catch, x/space STOP, z zero arm, c calib, [ ] trim, l log, p status"));
  Serial.println(F("t_ms,theta_deg,d_deg,u_rad_s2,trim_deg,mode"));
}

void handleKey(char c) {
  switch (c) {
    case 'x': case 'X': case ' ': stopAll(F("key")); break;
    case 'a': case 'A':
      if (mode == IDLE) { driverEnable(true); mode = ARMED; engageTicks = 0; Serial.println(F("# ARMED: lift the pendulum to upright and hold still")); }
      break;
    case 'z': case 'Z': if (mode == IDLE) { noInterrupts(); stepPos = 0; interrupts(); Serial.println(F("# arm zero set")); } break;
    case 'c': case 'C': if (mode == IDLE) calibrateDown(); break;
    case '[': trim -= radians(0.2); break;
    case ']': trim += radians(0.2); break;
    case 'l': case 'L': logging = !logging; break;
    case 'p': case 'P': printStatus(); break;
  }
}

void loop() {
  while (Serial.available()) handleKey(Serial.read());

  uint32_t now = micros();
  if ((int32_t)(now - nextLoopUs) < 0) return;
  if ((int32_t)(now - nextLoopUs) > (int32_t)LOOP_US) overruns++;   // a whole period late
  nextLoopUs += LOOP_US;

  // ---- measure ----
  if (updateAlpha()) readFails = 0;
  else if (++readFails >= 3 && mode == BALANCE) stopAll(F("encoder read failed"));
  float d = dFromUpright();
  float dRaw = dPrevValid ? (d - dPrev) / DT : 0;
  if (fabs(dRaw) > 100) dRaw = 0;                          // ignore a wrap/glitch spike
  dPrev = d; dPrevValid = true;
  dDotF += (DT / (TAU_D + DT)) * (dRaw - dDotF);
  float theta = readStepPos() / STEPS_PER_RAD;

  // ---- mode logic ----
  float u = 0;
  if (mode == ARMED) {
    if (fabs(d) < ENGAGE_D && fabs(dDotF) < 1.0) {
      if (++engageTicks >= 40) {                           // 0.2 s inside the window
        mode = BALANCE; balanceStartMs = millis(); vCmd = 0; dDotF = 0;
        Serial.println(F("# BALANCE"));
      }
    } else engageTicks = 0;
  } else if (mode == BALANCE) {
    if (fabs(d) > DROP_D) stopAll(F("pendulum fell (|d| > 25 deg)"));
    else if (fabs(theta) > ARM_LIMIT) stopAll(F("arm limit"));
    else {
      u = -(K_THETA * theta + K_ALPHA * d + K_THETA_DOT * vCmd + K_ALPHA_DOT * dDotF);
      u = constrain(u, -U_MAX, U_MAX);
      vCmd = constrain(vCmd + u * DT, -V_MAX, V_MAX);
      setStepRate(vCmd * STEPS_PER_RAD);
      if (millis() - balanceStartMs > 1000)                // auto-trim after the catch transient
        trim = constrain(trim - KI_TRIM * theta * DT, -TRIM_MAX, TRIM_MAX);
    }
  }
  uLast = u;

  // ---- 100 Hz log ----
  if (logging && (++logDiv & 1)) {
    Serial.print(millis());               Serial.print(',');
    Serial.print(degrees(theta), 2);      Serial.print(',');
    Serial.print(degrees(d), 2);          Serial.print(',');
    Serial.print(uLast, 1);               Serial.print(',');
    Serial.print(degrees(trim), 2);       Serial.print(',');
    Serial.println(mode);
  }
}

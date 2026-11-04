/*
  EXPERIMENTAL swing-up v2.4, separate from the tested balance sketch.
  Same pins, 1/16 microsteps, direction and gain set A. No automatic start.
  Reset with VMOT OFF, arm physically centered, pendulum hanging still.
  VMOT ON -> z -> s -> 3-second countdown -> pumping -> automatic LQR catch.
  x/SPACE stops, disables the driver, and cancels the attempt permanently.
  Before another attempt, physically recenter the disabled arm, then z and s.
  A failed catch, travel, encoder or timing fault also ends the attempt.
  On the 20 s pump timeout the arm first PARKS at commanded 0 (driver on, slow),
  then the driver turns off. Arm not on its 0 mark afterwards = lost steps.
  DO NOT hold or catch the pendulum by hand during pumping. Read docs/SWINGUP.md.
  Diagnostic build: buffered event records + 10 Hz state snapshots.
  See DIAGNOSTICS.md for schema/scaling; normal 50 Hz CSV is replaced.
  Modes: 0 idle, 1 countdown (driver OFF), 2 balance A, 3 energy pump, 4 park.
*/
#include <Wire.h>
#include "controller.h"
#include "diagnostics.h"

const uint8_t STEP_PIN = 3, DIR_PIN = 4, EN_PIN = 5, AS5600 = 0x36;
const bool INVERT_DIR = true;
const float STEPS_PER_RAD = 3200.0f / (2 * PI);
const int32_t HARD_STEPS = 791; // 88.9875 degrees; before the wiring's +/-90 limit.
volatile int32_t stepPos = 0;
volatile int8_t stepDir = 0;
volatile bool travelFault = false;
int16_t lastRaw = -1;
int32_t alphaCounts = 0;
float downCounts = 0;
bool calibrated = false, sensorOK = false;
uint32_t nextLoopUs, overruns = 0;
uint8_t logDiv = 0;
bool logging = true;
Swing::Controller controller;
SwingDiag::Logger diagnostics;

ISR(TIMER1_COMPA_vect) {
  const int32_t next = stepPos + stepDir;
  if (next > HARD_STEPS || next < -HARD_STEPS) {
    TIMSK1 &= ~_BV(OCIE1A);
    PORTD |= _BV(5); // Driver off even if the main loop is delayed.
    travelFault = true;
    return;
  }
  PORTD |= _BV(3);
  stepPos = next;
  __builtin_avr_delay_cycles(40);
  PORTD &= ~_BV(3);
}
void timerStop() {
  noInterrupts(); TIMSK1 &= ~_BV(OCIE1A); stepDir = 0; interrupts();
}
void motorOff() { timerStop(); digitalWrite(EN_PIN, HIGH); }
int32_t readStepPos() {
  noInterrupts(); const int32_t p = stepPos; interrupts(); return p;
}
void setStepRate(float v) {
  if (fabs(v) < 31.0f) { timerStop(); return; }
  const int8_t dir = v > 0 ? 1 : -1;
  if (dir != stepDir) {
    timerStop();
    digitalWrite(DIR_PIN, ((dir > 0) != INVERT_DIR) ? HIGH : LOW);
    delayMicroseconds(2);
  }
  uint32_t top = (uint32_t)(2000000.0f / fabs(v)) - 1;
  if (top > 65535) top = 65535;
  noInterrupts();
  if (!travelFault) {
    OCR1A = (uint16_t)top;
    if (TCNT1 > OCR1A) TCNT1 = 0;
    stepDir = dir; TIMSK1 |= _BV(OCIE1A);
  }
  interrupts();
}
bool updateAlpha() {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(AS5600); Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600, (uint8_t)2) != 2 || Wire.getWireTimeoutFlag()) return false;
  const uint8_t hi = Wire.read(), lo = Wire.read();
  const int16_t raw = ((int16_t)(hi & 0x0F) << 8) | lo;
  if (lastRaw < 0) alphaCounts = raw;
  else {
    int16_t delta = raw - lastRaw;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    alphaCounts += delta;
  }
  lastRaw = raw;
  return true;
}
float measuredAlpha() {
  return Swing::wrap((alphaCounts - downCounts) * (2 * PI / 4096.0f));
}
void calibrateDown() {
  motorOff(); calibrated = false;
  float sum = 0; uint8_t n = 0;
  for (uint8_t i = 0; i < 50; ++i) {
    if (updateAlpha()) { sum += alphaCounts; ++n; }
    delay(5);
  }
  if (n == 50) { downCounts = sum / n; calibrated = true; }
  controller.resetMeasurement(); controller.zeroed = false;
  sensorOK = calibrated;
  diagnostics.message(millis(), controller, calibrated ? 1 : 2);
  nextLoopUs = micros() + 5000;
}
const __FlashStringHelper *reasonText(Swing::Reason reason) {
  switch (reason) {
    case Swing::KEY: return F("key");
    case Swing::SENSOR: return F("encoder/start posture");
    case Swing::TIMING: return F("control deadline missed");
    case Swing::ARM: return F("arm travel");
    case Swing::TIMEOUT: return F("20 s swing-up timeout (after park)");
    case Swing::FELL: return F("lost balance (>25 deg)");
    default: return F("none");
  }
}
void reportChange(Swing::Mode oldMode) {
  if (oldMode == controller.mode) return;
  diagnostics.message(millis(), controller, 0);
}
void handleKey(char c) {
  const Swing::Mode oldMode = controller.mode;
  if (c == 'x' || c == 'X' || c == ' ') {
    controller.stop(Swing::KEY); motorOff();
  } else if ((c == 'z' || c == 'Z') && controller.mode == Swing::IDLE) {
    motorOff();
    noInterrupts(); stepPos = 0; travelFault = false; interrupts();
    controller.zero(); diagnostics.message(millis(), controller, 3);
  } else if (c == 's' || c == 'S') {
    if (!controller.start(millis(), readStepPos() / STEPS_PER_RAD, measuredAlpha(), calibrated && sensorOK))
      diagnostics.message(millis(), controller, 4);
    else diagnostics.resetAttempt();
  } else if ((c == 'c' || c == 'C') && controller.mode == Swing::IDLE) calibrateDown();
  else if (c == 'l' || c == 'L') logging = !logging;
  else if (c == 'p' || c == 'P') {
    diagnostics.message(millis(), controller, 5);
  }
  reportChange(oldMode);
}
void setup() {
  digitalWrite(EN_PIN, HIGH); pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW); pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  Serial.begin(115200); Wire.begin(); Wire.setClock(400000);
  Wire.setWireTimeout(2000, true);
  noInterrupts();
  TCCR1A = 0; TCCR1B = _BV(WGM12) | _BV(CS11); TCNT1 = 0; OCR1A = 65535; TIMSK1 = 0;
  interrupts();
  Serial.println(F("# swingup-v2.4 EXPERIMENTAL; centred pump, park on timeout, catch phase 100/360 for 1 s"));
  Serial.print(F("# catch phase U_MAX=")); Serial.print(Swing::CATCH_U_MAX, 1);
  Serial.print(F(" V_MAX=")); Serial.print(Swing::CATCH_V_MAX, 6);
  Serial.print(F(" ms=")); Serial.println(Swing::CATCH_MS);
  Serial.print(F("# pump U_MAX=")); Serial.print(Swing::U_MAX, 1);
  Serial.print(F(" V_MAX=")); Serial.println(Swing::V_MAX, 6);
  Serial.print(F("# balance U_MAX=")); Serial.print(Swing::BALANCE_U_MAX, 1);
  Serial.print(F(" V_MAX=")); Serial.println(Swing::BALANCE_V_MAX, 6);
  Serial.print(F("# capture_deg=")); Serial.println(degrees(Swing::CAPTURE_D),1);
  Serial.println(F("# driver OFF: calibrating hanging down; do not move pendulum"));
  calibrateDown();
  Serial.println(F("# z zero, s 3s countdown/swing-up, x/space stop, c calibrate, l log, p status"));
  Serial.println(F("# diagnostics schema: swingup/DIAGNOSTICS.md; integer scaled #D records"));
}
void loop() {
  // Stop wins over any later start/zero characters in the same received batch.
  bool cancelled = false;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'x' || c == 'X' || c == ' ') { handleKey(c); cancelled = true; }
    else if (!cancelled) handleKey(c);
  }
  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextLoopUs) < 0) {
    diagnostics.drain(Serial, nextLoopUs-nowUs);
    return;
  }
  const Swing::Mode oldMode = controller.mode;
  const SwingDiag::Before before(controller);
  if ((uint32_t)(nowUs - nextLoopUs) >= 5000) {
    ++overruns;
    if (controller.mode != Swing::IDLE) controller.stop(Swing::TIMING);
    nextLoopUs = nowUs;
  }
  nextLoopUs += 5000;
  sensorOK = updateAlpha();
  if (!sensorOK) calibrated = false; // Successful reads alone do not clear a sensor fault.
  if (travelFault && controller.mode != Swing::IDLE) controller.stop(Swing::ARM);
  const float theta = readStepPos() / STEPS_PER_RAD;
  controller.tick(millis(), theta, measuredAlpha(), sensorOK && calibrated);
  if (controller.mode == Swing::PUMP || controller.mode == Swing::BALANCE ||
      controller.mode == Swing::PARK) {
    // ISR owns a latched travel stop; main loop must never re-enable over it.
    noInterrupts();
    if (!travelFault) PORTD &= ~_BV(5);
    interrupts();
    setStepRate(controller.velocity * STEPS_PER_RAD);
  } else motorOff();
  reportChange(oldMode);
  diagnostics.observe(millis(), theta, measuredAlpha(), before, controller, logging);
}

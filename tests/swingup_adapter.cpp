// Executes the complete sketch with stubbed hardware. Does not emulate electrical
// timing/interrupt concurrency, sensor magnet errors, or motor torque.
#include "swingup_stubs/Arduino.h"
#include "../swingup/swingup.ino"
#include <assert.h>
#include <stdio.h>

void tick() { fakeUs = nextLoopUs; loop(); }
void startPump() {
  Serial.pending = "zs"; tick();
  assert(controller.mode == Swing::COUNTDOWN && (PORTD & _BV(5)));
  fakeUs = (controller.requestedMs + 3000) * 1000;
  nextLoopUs = fakeUs; loop();
  assert(controller.mode == Swing::PUMP && !(PORTD & _BV(5)));
}
int main() {
  setup();
  assert(calibrated && !controller.zeroed && (PORTD & _BV(5)) && TIMSK1 == 0);
  logging = false;
  startPump();
  Serial.pending = "xzs"; tick();
  assert(controller.mode == Swing::IDLE && !controller.zeroed);
  assert((PORTD & _BV(5)) && !(TIMSK1 & _BV(OCIE1A)));
  tick(); assert(controller.mode == Swing::IDLE); // No automatic retry.
  startPump();
  Wire.fail = true; tick();
  assert(controller.reason == Swing::SENSOR && controller.mode == Swing::IDLE);
  assert(!calibrated && (PORTD & _BV(5)) && !(TIMSK1 & _BV(OCIE1A)));
  Wire.fail = false; Serial.pending = "zs"; tick();
  assert(controller.mode == Swing::IDLE); // Read recovery cannot silently clear fault.
  Serial.pending = "c"; tick(); assert(calibrated && !controller.zeroed);
  startPump();
  fakeUs = nextLoopUs + 6000; loop();
  assert(controller.reason == Swing::TIMING && controller.mode == Swing::IDLE);
  assert((PORTD & _BV(5)) && !(TIMSK1 & _BV(OCIE1A)));
  startPump();
  stepPos = HARD_STEPS; stepDir = 1; TIMER1_COMPA_vect();
  assert(stepPos == HARD_STEPS && travelFault && (PORTD & _BV(5)));
  tick(); assert(controller.reason == Swing::ARM && controller.mode == Swing::IDLE);
  assert((PORTD & _BV(5)) && !(TIMSK1 & _BV(OCIE1A)));
  // Negative travel has the same pulse-level guard.
  travelFault = false; stepPos = -HARD_STEPS; stepDir = -1;
  TIMER1_COMPA_vect(); assert(stepPos == -HARD_STEPS && travelFault);
  // The complete adapter hands the moving plant to A without another key.
  startPump();
  Wire.raw = 1269 + 2048;
  controller.previousAlpha = Swing::PI_F; controller.rate = 0;
  controller.previousD = 0; controller.velocity = .5f;
  tick();
  assert(controller.mode == Swing::BALANCE && controller.velocity > .5f);
  assert(!(PORTD & _BV(5)));
  Wire.raw += 300; tick(); // >25 degrees: terminal failed catch.
  assert(controller.mode == Swing::IDLE && controller.reason == Swing::FELL);
  tick(); assert(controller.mode == Swing::IDLE && (PORTD & _BV(5)));
  Wire.fail = true; Serial.pending = "c"; tick(); assert(!calibrated);
  puts("PASS: complete sketch startup, countdown enable, cancellation batch, no auto-retry, sensor recovery lockout, recalibration, timing stop, positive/negative ISR travel cutoff");
}

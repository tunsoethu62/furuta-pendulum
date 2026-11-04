// Compare the swing-up BALANCE path with the actual, untouched proven sketch.
#include "swingup_stubs/Arduino.h"
#include "swingup_stubs/Wire.h"
#include "../swingup/controller.h"
#include <assert.h>
#include <stdio.h>
namespace Proven {
#include "../balance/balance.ino"
}
int main() {
  Swing::Controller c;
  c.mode = Swing::BALANCE;
  Proven::mode = Proven::BALANCE;
  Proven::logging = false;
  Proven::downCounts = 0;
  // Sustained offset exercises trim, saturation, nonzero velocity, and filtering.
  for (int n = 0; n < 800; ++n) {
    fakeUs = (n + 1) * 5000;
    Proven::nextLoopUs = fakeUs;
    Wire.raw = 2048 + int(18 * sinf(n * .019f));
    Proven::stepPos = int(80 * sinf(n * .007f));
    const float theta = Proven::stepPos / Proven::STEPS_PER_RAD;
    const float alpha = Wire.raw * Proven::RAD_PER_COUNT;
    Proven::loop();
    c.tick(millis(), theta, alpha);
    assert(c.mode == Swing::BALANCE);
    if (fabsf(c.u - Proven::uLast) > .01f ||
        fabsf(c.velocity - Proven::vCmd) > .001f ||
        fabsf(c.trim - Proven::trim) > .00001f) {
      printf("FAIL parity tick=%d u=%.5f/%.5f v=%.5f/%.5f trim=%.6f/%.6f\n",
             n, c.u, Proven::uLast, c.velocity, Proven::vCmd, c.trim, Proven::trim);
      return 1;
    }
  }
  c.tick(5000, 0, Swing::PI_F + radians(26));
  assert(c.mode == Swing::IDLE && c.reason == Swing::FELL);
  puts("PASS: 800 ticks match actual balance.ino output, velocity, trim; 25-degree drop stops");
}

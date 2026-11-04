// Host regression tests and nonlinear simulation of the SAME controller.h used
// by the AVR sketch. No motor/serial access. Build/run with run_swingup_checks.py.
#include "../swingup/controller.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

using namespace Swing;
static float rad(float degrees) { return degrees * PI_F / 180; }

void tests() {
  // Replay the 16:06 run's local reversals (24129/24134/24139 ms).
  // These candidates contain no bottom traversal and must not learn loss.
  {
    Controller c;c.rate=2;c.trackAmplitude(0,.283f);
    c.rate=-1.28f;c.trackAmplitude(.283f,.263f); // first real seed
    const float e=c.energy, peak=c.swingPeak;
    c.pumpedLastHalf=true;
    c.rate=.77f;c.trackAmplitude(.263f,.280f);
    assert(c.loss==LOSS0 && c.energy==e && c.swingSign==-1);
    assert(c.swingPeak<=peak); // rejected chatter does not erase directional history
    c.rate=-2;c.trackAmplitude(.280f,.252f);
    assert(c.loss==LOSS0 && c.energy==e && c.swingSign==-1);
  }
  // Regression: the preceding tip must not mask the smaller opposite tip.
  // Drive the actual turning-point detector through an asymmetric half swing.
  {
    for (int direction = -1; direction <= 1; direction += 2) {
      const float sign = float(direction);
      Controller a; a.swingSign = sign > 0 ? -1 : 1;
      a.swingPeak = sign * rad(150);
      a.rate = -sign * 2; a.trackAmplitude(sign * rad(150), sign * rad(100));
      a.trackAmplitude(sign * rad(100), 0);
      a.trackAmplitude(0, -sign * rad(120));
      a.rate = sign; a.trackAmplitude(-sign * rad(120), -sign * rad(119));
      assert(fabsf(a.energy - (1 - cosf(rad(120)))) < 1e-4f);
    }
  }
  Controller c;
  assert(!c.start(0, 0, 0, true)); // Must explicitly zero first.
  c.zero();
  assert(!c.start(0, 0, 0, false));
  assert(!c.start(0, rad(6), 0, true));
  assert(!c.start(0, 0, rad(11), true));
  assert(c.start(0, 0, 0, true));
  assert(!c.start(100, 0, 0, true)); // Duplicate cannot restart countdown.
  c.tick(2995, 0, 0); assert(c.mode == COUNTDOWN && c.velocity == 0);
  c.tick(3000, 0, 0); assert(c.mode == PUMP && c.u > 0);
  c.stop(KEY); c.tick(6000, 0, 0);
  assert(c.mode == IDLE && c.velocity == 0 && !c.zeroed);
  assert(!c.start(6000, 0, 0, true));
  c.zero(); assert(c.start(6000, 0, 0, true));
  c.tick(6005, 0, 0, false); assert(c.mode == IDLE && c.reason == SENSOR);
  c.zero(); assert(c.start(7000, 0, 0, true));
  c.tick(10000, 0, 0); c.tick(30000, 0, 0);
  assert(c.mode == PARK); // v2.3: timeout parks the arm before ending.
  for (uint32_t t = 30005; c.mode == PARK && t < 30000 + PARK_MS + 10; t += 5) c.tick(t, 0, 0);
  assert(c.mode == IDLE && c.reason == TIMEOUT && c.velocity == 0 && !c.zeroed);

  // PARK: from pump speed and an offset arm, decelerate within U_MAX, return to
  // commanded 0 at <= PARK_V, never catch, and end as TIMEOUT within PARK_MS.
  const float parkSigns[] = {-1.0f, 1.0f};
  for (float sign : parkSigns) {
    Controller k; k.zero(); assert(k.start(0, 0, 0, true));
    k.tick(3000, 0, 0); assert(k.mode == PUMP);
    k.velocity = sign * V_MAX;
    float theta = sign * rad(40), lastV = k.velocity; uint32_t t = 3000 + PUMP_MS;
    float stepAcc = theta * 3200 / (2 * PI_F);
    k.tick(t, theta, PI_F); // upright pendulum: must not catch in PARK
    assert(k.mode == PARK);
    bool reachedSlow = false;
    while (k.mode == PARK) {
      assert(fabsf(k.velocity - lastV) <= U_MAX * DT + 1e-4f);
      if (fabsf(k.velocity) <= PARK_V + 1e-4f) reachedSlow = true;
      else assert(!reachedSlow); // once slow, stays slow
      lastV = k.velocity;
      // Timer accumulates fractional steps; theta is the whole issued step count.
      // Velocities below the 31 steps/s deadband issue no steps (as the sketch).
      if (fabsf(k.velocity) >= MIN_SPEED) stepAcc += k.velocity * DT * 3200 / (2 * PI_F);
      theta = truncf(stepAcc) * (2 * PI_F) / 3200;
      t += 5; k.tick(t, theta, PI_F);
      assert(k.mode != BALANCE);
      assert(uint32_t(t - (3000 + PUMP_MS)) <= PARK_MS + 5);
    }
    assert(k.reason == TIMEOUT && fabsf(theta) < rad(0.2f));
    // Key stop during PARK is immediate and terminal.
    Controller s; s.zero(); s.start(0, 0, 0, true); s.tick(3000, 0, 0);
    s.tick(3000 + PUMP_MS, rad(20), 0); assert(s.mode == PARK);
    s.stop(KEY); assert(s.mode == IDLE && s.velocity == 0);
    s.tick(3005 + PUMP_MS, rad(20), 0); assert(s.mode == IDLE);
  }
  // Mean centring: with an arm offset and a saturated pump, lower-half targets
  // are never clipped, keep the centring bias, and both reversals are equal.
  const float offsets[] = {-60.0f, -20.0f, 0.0f, 20.0f, 60.0f}, alphas[] = {-.4f, .4f};
  for (float offset : offsets) {
    for (float alpha0 : alphas) {
      Controller m; m.mode = PUMP; m.pumpMs = 0; m.energy = 1; m.pumpSpeed = V_MAX;
      m.thetaMean = rad(offset); m.previousValid = true; m.previousAlpha = alpha0; m.rate = 0;
      m.tick(2000, rad(offset), alpha0);
      assert(m.mode == PUMP && fabsf(m.lastTargetRaw - m.lastTarget) < 1e-6f);
      const float bias = clamp(-MEAN_KP * m.thetaMean, -BIAS_MAX, BIAS_MAX);
      assert(fabsf(m.lastTarget - m.lastPump - bias) < 1e-5f);
      assert(fabsf(fabsf(m.lastPump) - fminf(V_MAX, V_MAX - fabsf(bias))) < 1e-5f);
      assert(offset == 0 || bias * offset < 0);
    }
  }
  // v2.4 catch phase: a capture from PUMP uses CATCH limits, never jumps velocity,
  // and returns to the exact balance.ino limits only once speed <= BALANCE_V_MAX.
  {
    Controller g; g.mode = PUMP; g.pumpMs = 0; g.previousValid = true;
    g.previousAlpha = PI_F; g.velocity = 0;
    g.tick(500, 0, PI_F); assert(g.mode == BALANCE && g.catching);
    // Large error: request saturates at the catch limit, not 60.
    float lastV = g.velocity;
    g.tick(505, 0, PI_F - rad(10)); assert(fabsf(g.u) <= CATCH_U_MAX + 1e-3f && fabsf(g.u) > BALANCE_U_MAX);
    assert(fabsf(g.velocity - lastV) <= CATCH_U_MAX * DT + 1e-4f);
    // Fast arm after CATCH_MS: still catching until speed is back inside pi.
    g.velocity = BALANCE_V_MAX + 1; g.rate = 0; g.previousAlpha = PI_F; g.previousD = 0;
    g.tick(500 + CATCH_MS + 5, 0, PI_F);
    assert(g.catching && fabsf(g.velocity) > BALANCE_V_MAX);
    // Even when A keeps pushing outward, speed ramps down at <= 60 rad/s^2.
    uint32_t t = 500 + CATCH_MS + 10;
    for (; g.catching && t < 500 + CATCH_MS + 2000; t += 5) {
      lastV = g.velocity; g.tick(t, 0, PI_F);
      assert(fabsf(g.velocity - lastV) <= BALANCE_U_MAX * DT + 1e-4f);
    }
    assert(t <= 500 + CATCH_MS + 10 + uint32_t(1000 * 1.0f / BALANCE_U_MAX / DT * DT) + 400);
    assert(!g.catching && g.mode == BALANCE && fabsf(g.velocity) <= BALANCE_V_MAX);
    g.tick(500 + CATCH_MS + 2005, 0, PI_F - rad(10));
    assert(fabsf(g.u) <= BALANCE_U_MAX + 1e-4f); // back to proven limits
    // A balance entered without a moving catch (parity path) never uses them.
    Controller h; h.mode = BALANCE; assert(!h.catching);
  }
  // cos kick: the open-loop start's commanded arm motion is centred on zero.
  {
    Controller q; q.zero(); q.start(0, 0, 0, true); q.tick(3000, 0, 0);
    double theta = 0, sum = 0; int n = 0; float lo = 0, hi = 0;
    for (uint32_t t = 3005; t < 3000 + KICK_MS; t += 5) {
      theta += q.velocity * DT; sum += theta; ++n;
      lo = fminf(lo, float(theta)); hi = fmaxf(hi, float(theta));
      q.tick(t, float(theta), 0); // pendulum still: kick continues
    }
    assert(fabs(sum / n) < rad(6) && hi > rad(15) && lo < -rad(15));
  }

  // Wrap crossing must not create a 2*pi/dt derivative impulse.
  Controller wrapTest;
  wrapTest.tick(0, 0, PI_F - .001f);
  wrapTest.tick(5, 0, -PI_F + .001f);
  assert(fabsf(wrapTest.rate) < .5f);

  // Capture uses the angle, pendulum-rate and arm gates. Preserve filter and speed.
  Controller capture;
  capture.mode = PUMP; capture.pumpMs = 0;
  capture.previousValid = true; capture.previousAlpha = PI_F;
  capture.velocity = .5f;
  capture.tick(500, 0, PI_F);
  assert(capture.mode == BALANCE && capture.velocity > .5f && capture.catches == 1);
  capture.tick(505, 0, PI_F - rad(24));
  assert(capture.mode == BALANCE);
  capture.tick(510, 0, PI_F - rad(26));
  assert(capture.mode == IDLE && capture.reason == FELL);
  capture.tick(515, 0, PI_F);
  assert(capture.mode == IDLE); // Failed catch must not restart motion.
  for (int gate = 0; gate < 5; ++gate) {
    Controller q; q.mode = PUMP; q.previousValid = true;
    float a = gate == 0 ? PI_F - rad(21) : PI_F;
    q.previousAlpha = a;
    if (gate == 1) q.rate = 15; // z ~ 8.4 rad/s after this tick: flying through.
    if (gate == 3) q.velocity = BALANCE_V_MAX + .1f;
    if (gate == 4) { a = PI_F + rad(10); q.previousAlpha = a; }
    q.tick(500, gate == 2 ? rad(61) : 0, a);
    assert(q.mode == PUMP);
  }
  // Energy comes from turning-point amplitude, not speed: a speed spike at the
  // bottom (weak-magnet distortion) must not raise the estimate.
  {
    Controller e; e.mode = PUMP; e.pumpMs = 0; e.energy = .5f;
    float a = 0; uint32_t t = 0;
    e.tick(t, 0, a);
    for (int i = 0; i < 40; ++i) {
      t += 5; a += (i == 3 ? .15f : .05f); // one 30 rad/s reading near the bottom
      e.tick(t, 0, a);
      assert(e.energy == .5f); // v1 would read ~3.1 here from the speed.
    }
    for (int i = 0; i < 40; ++i) { t += 5; a -= .05f; e.tick(t, 0, a); }
    assert(e.mode == PUMP && fabsf(e.energy - (1 - cosf(2.1f))) < 1e-3f);
  }
  // Bottom-timed reversal: just after the bottom moving positive, accelerate +.
  {
    Controller b; b.mode = PUMP; b.pumpMs = 0; b.energy = 1; b.pumpSpeed = 2;
    b.previousValid = true; b.previousAlpha = -.05f; b.rate = 10; b.velocity = -1;
    b.tick(2000, 0, 0); assert(b.u > 0);
    Controller n; n.mode = PUMP; n.pumpMs = 0; n.energy = 1; n.pumpSpeed = 2;
    n.previousValid = true; n.previousAlpha = .05f; n.rate = -10; n.velocity = 1;
    n.tick(2000, 0, 0); assert(n.u < 0);
    Controller full = n; full.energy = E_TARGET; full.pumpSpeed = 0; full.velocity = 0;
    full.tick(2005, 0, -.05f); assert(fabsf(full.u) < 1e-3f); // No pumping at target.
  }
  // Deadbeat speed: at a turning point, Vp = (2 - E + loss)/gain, capped at V_MAX.
  {
    Controller p; p.mode = PUMP; p.pumpMs = 0; p.loss = .3f; p.swingSign = 1;
    p.rawSwingSign=1;p.rawPeak=2.4f;
    p.swingPeak = 2.4f; p.previousValid = true; p.previousAlpha = 2.4f; p.rate = 1;
    p.tick(3000, 0, 2.39f); p.tick(3005, 0, 2.37f); p.tick(3010, 0, 2.34f);
    const float e = 1 - cosf(2.4f);
    assert(fabsf(p.energy - e) < 1e-3f);
    assert(fabsf(p.pumpSpeed - (E_TARGET - e + .3f) / PUMP_GAIN) < 1e-3f);
  }
  // Going over the top: marked over-energised, no pumping, braking in the top zone.
  {
    Controller o; o.mode = PUMP; o.pumpMs = 0; o.energy = 1.9f; o.pumpSpeed = 1;
    o.loss = .8f;
    o.previousValid = true; o.previousAlpha = PI_F - .02f; o.rate = 8;
    o.tick(4000, 0, -PI_F + .02f);
    assert(o.energy >= E_OVER && o.pumpSpeed == 0 && o.u > 0);
    assert(fabsf(o.loss - .4f) < 1e-6f); // Overshoot must back off compensation.
  }
  const float signs[] = {-1.0f, 1.0f};
  for (float sign : signs) {
    Controller q; q.mode = PUMP; q.velocity = sign * V_MAX;
    q.tick(500, sign * rad(73), 0);
    assert(q.braking && q.u * sign == -U_MAX);
    q.tick(505, sign * HARD_ARM, 0);
    assert(q.mode == IDLE && q.reason == ARM && q.velocity == 0);
  }
  // uint32 millisecond wrap must not prematurely finish a countdown.
  Controller rollover; rollover.zero();
  uint32_t start = UINT32_MAX - 1000;
  assert(rollover.start(start, 0, 0, true));
  rollover.tick(start + 2995u, 0, 0); assert(rollover.mode == COUNTDOWN);
  rollover.tick(start + 3000u, 0, 0); assert(rollover.mode == PUMP);
  Controller invalid; invalid.mode = PUMP;
  invalid.tick(0, 0, NAN); assert(invalid.mode == IDLE && invalid.reason == SENSOR);
  puts("PASS: transitions, cancellation, faults, timeout+park, park limits/no catch/key stop, mean centring headroom, centred kick, wrap, five capture gates, terminal failed catch, amplitude energy, reversal, braking, rollover, NaN");
}

struct Scenario { const char *name; float zeta, friction, couplingScale, bias, sin1, sin2; bool mustHold, stiction; float topFriction; };
struct Result { bool captured, held; float catchSeconds, maxArm, peakSwing; Reason stop; };
Result simulate(Scenario scenario, const char *csvPath = nullptr) {
  Controller c; c.zero(); c.start(0, 0, 0, true);
  float alpha = 0, omega = 0, theta = 0, actualVelocity = 0, stepPhase = 0;
  const float countsPerRad = 4096 / (2 * PI_F), stepsPerRad = 3200 / (2 * PI_F);
  Result result = {false, false, -1, 0, 0, NONE};
  FILE *csv = csvPath ? fopen(csvPath, "w") : nullptr;
  if (csvPath && !csv) { perror(csvPath); exit(2); }
  if (csv) fputs("t_s,theta_deg,alpha_deg,alpha_dot_rad_s,u_rad_s2,v_cmd_rad_s,energy,mode,braking\n", csv);
  for (unsigned n = 0; n < 7000; ++n) {
    const uint32_t ms = n * 5;
    // Calibration removes bias at hanging down; bias here represents angle-dependent
    // error near upright, smoothly zero at down. Re-quantise the resulting sample.
    // sin1/sin2: weak-magnet nonlinearity (zero at the calibrated hanging position).
    float measured = roundf((alpha + scenario.bias * (1 - cosf(alpha)) / 2
        + scenario.sin1 * sinf(alpha) + scenario.sin2 * sinf(2 * alpha)) * countsPerRad) / countsPerRad;
    const Mode oldMode = c.mode;
    c.tick(ms, theta, wrap(measured));
    if (c.mode == BALANCE && oldMode != BALANCE) {
      result.captured = true; result.catchSeconds = ms / 1000.0f - 3;
    }
    const bool active = c.mode == PUMP || c.mode == BALANCE || c.mode == PARK;
    const float nextVelocity = active && fabsf(c.velocity) >= MIN_SPEED ? c.velocity : 0;
    const float acceleration = (nextVelocity - actualVelocity) / DT;
    // Semi-implicit integration in ten substeps, with the centrifugal term and
    // optional approximate Coulomb/stiction model. Does not simulate motor torque.
    for (int k = 0; k < 10; ++k) {
      const float h = DT / 10;
      float torque = -2 * scenario.zeta * WN * omega - WN * WN * sinf(alpha)
          + COUPLING * scenario.couplingScale * cosf(alpha) * acceleration
          + actualVelocity * actualVelocity * sinf(alpha) * cosf(alpha);
      const float friction = scenario.friction * WN * WN;
      if (scenario.stiction) {
        if (fabsf(omega) < .02f && fabsf(torque) < friction) omega = 0;
        else {
          float direction = fabsf(omega) > .001f ? omega : torque;
          omega += (torque - copysignf(friction, direction)) * h;
        }
      } else {
        // Calibrated to hardware: decay tests (motor off, large swings) give
        // T_c ~ 0.10 m g l_p, but balance runs A (2026-11-04) held about +-1 deg,
        // which this model only reproduces with <= ~0.03 near upright (checked
        // with balance-only runs). So: full dry friction while swinging, the
        // balance-compatible level within 30 deg of upright, no stick band
        // (stepper vibration dithers the hinge). Strict stiction is a stress case.
        const float level = fabsf(wrap(alpha - PI_F)) < PI_F / 6
            ? fminf(friction, scenario.topFriction * WN * WN) : friction;
        omega += (torque - level * tanhf(omega / .3f)) * h;
      }
      alpha += omega * h;
    }
    actualVelocity = nextVelocity;
    stepPhase += actualVelocity * DT * stepsPerRad;
    // Mirror the sketch's ISR guard: the pulse that would cross 791 is refused.
    // Hard travel can terminate BALANCE; only PUMP has predictive braking.
    if (fabsf(truncf(stepPhase)) > 791) {
      stepPhase = copysignf(791, stepPhase);
      c.stop(ARM);
    }
    theta = truncf(stepPhase) / stepsPerRad;
    result.maxArm = fmaxf(result.maxArm, fabsf(theta));
    result.peakSwing = fmaxf(result.peakSwing, fabsf(wrap(alpha)));
    assert(fabsf(c.u) <= U_MAX + .001f && fabsf(c.velocity) <= V_MAX + .001f);
    assert(fabsf(theta) < HARD_ARM); // Main-loop brake should stay inside ISR cutoff.
    if (csv && n % 4 == 0)
      fprintf(csv, "%.3f,%.3f,%.3f,%.4f,%.2f,%.4f,%.4f,%u,%u\n",
        ms / 1000.0, theta * 180 / PI_F, wrap(alpha) * 180 / PI_F, omega,
        c.u, c.velocity, c.energy, unsigned(c.mode), unsigned(c.braking));
    if (c.mode == IDLE) { result.stop = c.reason; break; }
    if (c.mode == BALANCE && ms / 1000.0f - 3 - result.catchSeconds > 5) {
      result.held = true; break;
    }
  }
  if (csv) fclose(csv);
  printf("%s: capture=%s, held_5s=%s, catch_s=%.3f, max_arm_deg=%.2f, peak_swing_deg=%.2f, stop_reason=%u\n",
    scenario.name, result.captured ? "yes" : "no", result.held ? "yes" : "no",
    result.catchSeconds, result.maxArm * 180 / PI_F, result.peakSwing * 180 / PI_F, unsigned(result.stop));
  return result;
}
int main(int argc, char **argv) {
  tests();
  const float D5 = 5 * PI_F / 180;
  // Identified hinge: dry friction T_c ~ 0.10 m g l_p (two decay runs), encoder
  // nonlinearity ~ +-5 deg. Viscous 0.06 is the old first-cycle equivalent only.
  const Scenario scenarios[] = {
    {"lossless", 0, 0, 1, 0, 0, 0, true, false, .02f},
    {"identified_dry_0.10", 0, .10f, 1, 0, 0, 0, true, false, .02f},
    {"dry_0.12", 0, .12f, 1, 0, 0, 0, true, false, .02f},
    {"dry_0.10_plus_viscous_0.02", .02f, .10f, 1, 0, 0, 0, true, false, .02f},
    {"dry_0.10_coupling_minus15", 0, .10f, .85f, 0, 0, 0, true, false, .02f},
    {"dry_0.10_coupling_plus15", 0, .10f, 1.15f, 0, 0, 0, true, false, .02f},
    {"dry_0.10_encoder_sin1_5deg", 0, .10f, 1, 0, -D5, 0, true, false, .02f},
    {"dry_0.10_encoder_sin2_5deg", 0, .10f, 1, 0, 0, D5, true, false, .02f},
    {"dry_0.10_upright_bias1deg", 0, .10f, 1, PI_F / 180, 0, 0, true, false, .02f},
    {"viscous_0.06_pessimistic", .06f, 0, 1, 0, 0, 0, false, false, .02f},
    {"dry_0.10_strict_stiction", 0, .10f, 1, 0, 0, 0, false, true, .02f},
  };
  int failures = 0;
  for (auto scenario : scenarios) {
    char path[1024];
    if (argc > 1) snprintf(path, sizeof(path), "%s/%s.csv", argv[1], scenario.name);
    Result r = simulate(scenario, argc > 1 ? path : nullptr);
    // Every run must either hold or terminate with a reason (never hang/repump forever).
    assert(r.held || r.stop != NONE);
    if (scenario.mustHold && !(r.captured && r.held)) {
      printf("FAIL: %s did not swing up and hold 5 s\n", scenario.name); ++failures;
    }
  }
  if (failures) return 1;
  puts("PASS: all required swing-up scenarios captured and held 5 s");
}

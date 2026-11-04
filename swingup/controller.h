#pragma once
#include <math.h>
#include <stdint.h>

// Swing-up v2.2 core. No Arduino dependencies: the host simulator and regression
// tests execute exactly this control/state-machine code.
//
// Why v2 (hardware logs 2026-11-04 15:09 and 15:12): v1 estimated energy from the
// measured pendulum speed. The 15:12 log stalled at 94.32 degrees yet estimated
// E > 2 at some bottom passes. Weak-magnet derivative distortion is a suspected
// contributor; the true speed is not independently measured. Avoid using that
// instantaneous speed estimate to decide whether to add bottom-pass energy.
// v2:
//  (a) energy only from turning-point amplitudes, E = 1 - cos(peak) (no speed);
//  (b) pumps with arm-velocity reversals timed at the bottom, v = Vp*sign(sin a),
//      so u = dv/dt is an impulse at the bottom with the sign of alpha_dot;
//  (c) Vp is set once per half swing to land exactly on E = 2 (upright), using an
//      online estimate of the per-half-swing loss (dry hinge friction);
//  (d) brakes if the pendulum is going over the top;
//  (e) catches only when the unstable upright mode is small, then runs the
//      proven balance.ino A law; a failed catch is a terminal stop.
namespace Swing {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float DT = 0.005f;
constexpr float WN = 12.083f, COUPLING = 1.042f;
// Experimental pump limits (v2.3: raised from 100 rad/s^2 / 250 deg/s; the 16:31 run held Vp at the 250 deg/s cap for 20 s).
// Old A reached 100 rad/s^2; current proven balance uses 60 and 180 deg/s.
#ifndef SWING_U_MAX
#define SWING_U_MAX 150.0f
#endif
#ifndef SWING_V_MAX_DEG
#define SWING_V_MAX_DEG 300.0f
#endif
constexpr float U_MAX = SWING_U_MAX;                     // rad/s^2, PUMP ONLY
constexpr float V_MAX = SWING_V_MAX_DEG * PI_F / 180.0f; // rad/s arm speed, PUMP ONLY
// Catch limits are the CURRENT hardware-tested balance/balance.ino values.
constexpr float BALANCE_U_MAX = 60.0f, BALANCE_V_MAX = PI_F;
#ifndef SWING_CATCH_U_MAX
#define SWING_CATCH_U_MAX 100.0f
#endif
#ifndef SWING_CATCH_V_MAX_DEG
#define SWING_CATCH_V_MAX_DEG 360.0f
#endif
#ifndef SWING_CATCH_MS
#define SWING_CATCH_MS 1000u
#endif
constexpr float CATCH_U_MAX = SWING_CATCH_U_MAX;
constexpr float CATCH_V_MAX = SWING_CATCH_V_MAX_DEG * PI_F / 180.0f;
constexpr uint32_t CATCH_MS = SWING_CATCH_MS;
constexpr float SOFT_ARM = 75.0f * PI_F / 180.0f;
constexpr float HARD_ARM = 89.0f * PI_F / 180.0f;

#ifndef SWING_CAPTURE_DEG
#define SWING_CAPTURE_DEG 18.0f
#endif
#ifndef SWING_CAPTURE_Z
#define SWING_CAPTURE_Z 3.0f
#endif
#ifndef SWING_LOSS0
#define SWING_LOSS0 0.3f
#endif
#ifndef SWING_CENTER_KP
#define SWING_CENTER_KP 2.0f
#endif
constexpr float CAPTURE_D = SWING_CAPTURE_DEG * PI_F / 180.0f;
// Catch only when the unstable upright mode z = wn*d + d_dot is small, i.e. the
// pendulum is creeping into the top, not flying through or falling away.
constexpr float CAPTURE_Z = SWING_CAPTURE_Z, CAPTURE_ARM = 60.0f * PI_F / 180.0f;
constexpr float DROP_D = 25.0f * PI_F / 180.0f;
constexpr float TRIM_MAX = 3.0f * PI_F / 180.0f;
constexpr float MIN_SPEED = 31.0f / (3200.0f / (2.0f * PI_F));
constexpr uint32_t COUNTDOWN_MS = 3000, PUMP_MS = 20000, KICK_MS = 1500;
#ifndef SWING_E_TARGET
#define SWING_E_TARGET 2.0f
#endif
constexpr float E_TARGET = SWING_E_TARGET; // E/(J wn^2): 0 hanging, 2 upright at rest
constexpr float E_OVER = 2.2f;           // speed-based, near the top only
constexpr float KICK_AMPLITUDE = 0.01f;  // 1-cos(8 deg): leave open-loop start
constexpr float PHASE_HYST = 5.0f * PI_F / 180.0f; // opposite-lobe phase qualification
constexpr float TURN_RATE = 0.5f;        // rad/s hysteresis for turning points
constexpr float CENTER_KP = SWING_CENTER_KP; // arm recentering, 1/s
// v2.3: hardware runs 16:06-16:31 pumped about a +17..+22 deg commanded arm
// offset; the instantaneous centering term was clipped at V_MAX on one reversal
// and could not remove it. Lower-half pumping now uses a bias from the
// low-passed arm angle and reserves its speed, so both reversals are 2*Vp.
#ifndef SWING_MEAN_CENTER
#define SWING_MEAN_CENTER 1
#endif
constexpr float MEAN_TAU = 0.5f;          // s, about one pendulum period
constexpr float MEAN_KP = 2.0f;           // 1/s on the arm oscillation centre
constexpr float BIAS_MAX = 1.0f;          // rad/s reserved for centering
#ifndef SWING_KICK_COS
#define SWING_KICK_COS 1
#endif
// Energy added by one bottom reversal -Vp -> +Vp near E = 2:
// dE = K/wn^2 * alpha_dot*cos(a)*dv, alpha_dot ~ 2 wn  ->  dE ~ 4K/wn * Vp.
constexpr float PUMP_GAIN = 0.9f * 4.0f * COUPLING / WN; // per rad/s of Vp
constexpr float LOSS0 = SWING_LOSS0, LOSS_MAX = 1.2f;

inline float clamp(float x, float lo, float hi) { return fmaxf(lo, fminf(hi, x)); }
inline float wrap(float x) {
  // Callers pass one-turn angles/increments; avoid trig in the 200 Hz path.
  if (!isfinite(x)) return x;
  while (x > PI_F) x -= 2.0f * PI_F;
  while (x < -PI_F) x += 2.0f * PI_F;
  return x;
}
enum Mode : uint8_t { IDLE = 0, COUNTDOWN = 1, BALANCE = 2, PUMP = 3, PARK = 4 };
// PARK (v2.3, pump timeout only): driver stays enabled and the arm returns
// gently to commanded 0, then the attempt ends with reason TIMEOUT. If the arm
// is then not on its physical 0 mark, steps were lost during pumping.
constexpr float PARK_KP = 3.0f, PARK_V = 1.0f, PARK_U = 40.0f; // 1/s, rad/s, rad/s^2
constexpr uint32_t PARK_MS = 4000;
constexpr float PARK_DONE = 0.5f * 2.0f * PI_F / 3200.0f; // half a microstep
enum Reason : uint8_t { NONE, KEY, SENSOR, TIMING, ARM, TIMEOUT, FELL };

enum TurnEvent : uint8_t { TURN_NONE, TURN_SEED, TURN_ACCEPT,
  TURN_REJECT_PHASE, TURN_REJECT_LOBE, TURN_REJECT_SMALL, TURN_ROTATION };

struct Controller {
  Mode mode = IDLE;
  Reason reason = NONE;
  bool zeroed = false, previousValid = false, braking = false, catching = false;
  float previousAlpha = 0, rate = 0, trim = 0, velocity = 0;
  float previousD = 0;
  float u = 0, d = 0, energy = 0;
  float thetaMean = 0;                       // low-passed arm angle while pumping
  // Observation only (diagnostics): last pump component and target before/after clip.
  float lastPump = 0, lastTargetRaw = 0, lastTarget = 0;
  // Pump state, updated once per half swing.
  float swingPeak = 0, pumpSpeed = 0, loss = LOSS0; // signed directional extreme
  bool lastEnergyValid = false, pumpedLastHalf = false;
  float lastEnergy = 0, lastPumpSpeed = 0;
  int8_t swingSign = 0;
  // Raw candidates are independent of accepted half-cycle history.
  int8_t rawSwingSign = 0, phaseSide = 0, acceptedLobe = 0;
  float rawPeak = 0, candidatePeak = 0;
  bool seeded = false, bottomQualified = false;
  TurnEvent turnEvent = TURN_NONE;
  uint8_t catches = 0;
  uint32_t requestedMs = 0, pumpMs = 0, balanceMs = 0, parkMs = 0;

  void stop(Reason why) {
    mode = IDLE; reason = why; velocity = u = 0; braking = false;
    zeroed = false; // Disabled motor may move: require a new physical zero.
    seeded = bottomQualified = false; rawSwingSign = phaseSide = acceptedLobe = 0;
    turnEvent = TURN_NONE;
  }
  void zero() { if (mode == IDLE) zeroed = true; }
  void resetMeasurement() { previousValid = false; rate = 0; }
  void enterPump(uint32_t now, float alpha) {
    mode = PUMP; pumpMs = now;
    swingSign = 0; swingPeak = alpha;
    rawSwingSign = phaseSide = acceptedLobe = 0;
    rawPeak = candidatePeak = alpha;
    seeded = bottomQualified = false; turnEvent = TURN_NONE;
    energy = 1.0f - cosf(alpha); // Lower bound until the next turning point.
    lastEnergyValid = pumpedLastHalf = false;
    pumpSpeed = 0; thetaMean = 0;
    lastPump = lastTargetRaw = lastTarget = 0;
  }

  bool start(uint32_t now, float theta, float alpha, bool sensorReady) {
    if (mode != IDLE || !zeroed || !sensorReady || !isfinite(theta) ||
        !isfinite(alpha) || fabsf(theta) > PI_F / 36 ||
        fabsf(wrap(alpha)) > PI_F / 18 || fabsf(rate) > 0.5f) return false;
    requestedMs = now; mode = COUNTDOWN; reason = NONE;
    trim = velocity = u = energy = 0; braking = false; catches = 0; loss = LOSS0;
    return true;
  }

  // Rate sign changes are raw candidates. Only a bottom traversal into the
  // opposite >=5-degree lobe qualifies the next energy/loss observation.
  void trackAmplitude(float lastAlpha, float alpha) {
    turnEvent = TURN_NONE;
    if (fabsf(lastAlpha)>2.8f && fabsf(alpha)>2.8f &&
        (lastAlpha>0)!=(alpha>0)) {
      loss *= .5f;
      energy = E_OVER; lastEnergyValid = false; pumpSpeed = 0;
      swingSign = rawSwingSign = acceptedLobe = phaseSide = 0;
      swingPeak = rawPeak = alpha; candidatePeak = alpha;
      seeded = bottomQualified = false; turnEvent = TURN_ROTATION;
      return;
    }
    const int8_t side=alpha>PHASE_HYST?1:(alpha<-PHASE_HYST?-1:0);
    if(side && side!=phaseSide) {
      if(seeded && phaseSide==acceptedLobe && side==-acceptedLobe)
        bottomQualified=true;
      phaseSide=side;
    }
    // Never reset accepted directional extreme on rejected candidates.
    if(seeded) swingPeak=swingSign>0?fmaxf(swingPeak,alpha):fminf(swingPeak,alpha);
    if(rawSwingSign>0) rawPeak=fmaxf(rawPeak,alpha);
    else if(rawSwingSign<0) rawPeak=fminf(rawPeak,alpha);
    else rawPeak=alpha;
    const int8_t sign=rate>TURN_RATE?1:(rate<-TURN_RATE?-1:0);
    if(!sign || sign==rawSwingSign) return;
    if(!rawSwingSign) {rawSwingSign=sign;rawPeak=alpha;return;}
    const int8_t oldRawSign=rawSwingSign;
    candidatePeak=seeded?swingPeak:rawPeak;
    const float localPeak=rawPeak;
    rawSwingSign=sign;rawPeak=alpha; // raw diagnostics may change freely
    if(!seeded) {
      candidatePeak=localPeak;
      if(fabsf(candidatePeak)<PHASE_HYST) {turnEvent=TURN_REJECT_SMALL;return;}
      const int8_t lobe=candidatePeak>0?1:-1;
      if(oldRawSign!=lobe || sign!=-lobe) {turnEvent=TURN_REJECT_LOBE;return;}
      turnEvent=TURN_SEED;
    } else {
      if(!bottomQualified) {candidatePeak=localPeak;turnEvent=TURN_REJECT_PHASE;return;}
      if(side!=-acceptedLobe || oldRawSign!=swingSign || sign!=-swingSign) {
        candidatePeak=localPeak;turnEvent=TURN_REJECT_LOBE;return;
      }
      turnEvent=TURN_ACCEPT;
    }
    energy=1.0f-cosf(candidatePeak);
    if(lastEnergyValid && pumpedLastHalf)
      loss=clamp(loss+.5f*(lastEnergy+PUMP_GAIN*lastPumpSpeed-energy-loss),0.0f,LOSS_MAX);
    lastEnergy=energy;lastEnergyValid=true;
    pumpSpeed=energy>=E_TARGET?0.0f:clamp((E_TARGET-energy+loss)/PUMP_GAIN,0.0f,V_MAX);
    lastPumpSpeed=pumpSpeed;pumpedLastHalf=false;
    seeded=true;acceptedLobe=candidatePeak>0?1:-1;
    bottomQualified=false;swingSign=-acceptedLobe;swingPeak=alpha;
  }

  void tick(uint32_t now, float theta, float alpha, bool sensorOK = true) {
    turnEvent = TURN_NONE;
    if (!sensorOK || !isfinite(theta) || !isfinite(alpha)) {
      resetMeasurement();
      if (mode != IDLE) stop(SENSOR);
      return;
    }
    // Differentiate the wrapped increment, never the upright-angle discontinuity.
    d = wrap(alpha - PI_F) - trim;
    float rawRate = previousValid ? wrap(alpha - previousAlpha) / DT : 0;
    // In BALANCE reproduce the proven sketch's trimmed derivative and glitch
    // rejection. Pumping needs wrapped increments for complete rotations.
    if (mode == BALANCE) {
      rawRate = previousValid ? (d - previousD) / DT : 0;
      if (fabsf(rawRate) > 100) rawRate = 0;
    }
    const float lastAlpha = previousValid ? previousAlpha : alpha;
    previousAlpha = alpha; previousValid = true;
    previousD = d;
    rate += (DT / (1.0f / (2.0f * PI_F * 25.0f) + DT)) * (rawRate - rate);
    u = 0; braking = false;
    if (mode == IDLE) return;
    if (fabsf(theta) >= HARD_ARM) { stop(ARM); return; }
    if (mode == COUNTDOWN) {
      // Arm must remain centered, pendulum down and still until motion begins.
      if (fabsf(theta) > PI_F / 36 || fabsf(wrap(alpha)) > PI_F / 18 ||
          fabsf(rate) > 0.5f) { stop(SENSOR); return; }
      if (uint32_t(now - requestedMs) < COUNTDOWN_MS) return;
      enterPump(now, alpha);
    }
    if (mode == BALANCE && fabsf(d) > DROP_D) {
      stop(FELL); return; // Same as balance.ino: driver off on a drop.
    }
    if (mode == PUMP && uint32_t(now - pumpMs) >= PUMP_MS) {
      mode = PARK; parkMs = now; pumpSpeed = 0;
    }
    if (mode == PARK) {
      // Never catches or pumps. Ends as TIMEOUT when parked or after PARK_MS.
      const bool atZero = fabsf(theta) < PARK_DONE;
      if ((atZero && fabsf(velocity) <= 2.0f * MIN_SPEED) ||
          uint32_t(now - parkMs) >= PARK_MS) { stop(TIMEOUT); return; }
      float target = clamp(-PARK_KP * theta, -PARK_V, PARK_V);
      // Below the step generator's 31 steps/s deadband creep at 1.5x that speed.
      if (!atZero && fabsf(target) < 1.5f * MIN_SPEED)
        target = theta > 0 ? -1.5f * MIN_SPEED : 1.5f * MIN_SPEED;
      if (atZero) target = 0;
      // Leave pump speed at the proven pump deceleration, then move gently.
      const float limit = fabsf(velocity) > PARK_V ? U_MAX : PARK_U;
      u = clamp((target - velocity) / DT, -limit, limit);
      // Falls through to the shared pump clamp + predictive travel braking.
    }
    if (mode == PUMP) {
      trackAmplitude(lastAlpha, alpha);
      const float catchU = 27.5553f * theta + 600.4973f * d
          + 14.2170f * velocity + 44.9091f * rate;
      // Reject a heavily saturated first command. A 25% allowance admits mild
      // clipping; this is a simulated capture heuristic, not a basin proof.
      if (fabsf(d) < CAPTURE_D && fabsf(WN * d + rate) < CAPTURE_Z &&
          fabsf(theta) < CAPTURE_ARM && fabsf(velocity) <= BALANCE_V_MAX &&
          fabsf(catchU) <= BALANCE_U_MAX * 1.25f) {
        mode = BALANCE; balanceMs = now; ++catches; catching = CATCH_MS > 0;
        // Keep velocity/filter state: no artificial reset at a moving catch.
      } else {
        float pump = 0, center = -CENTER_KP * theta;
        thetaMean += (DT / (MEAN_TAU + DT)) * (theta - thetaMean);
        const float eTop = 0.5f * (rate / WN) * (rate / WN) + 1.0f - cosf(alpha);
        if (energy < KICK_AMPLITUDE && uint32_t(now - pumpMs) < KICK_MS) {
          // Open-loop start at the natural frequency breaks hinge stiction.
          // cos start (v2.3): first quarter period positive, so the square-wave
          // arm motion is centred on zero instead of on +V_MAX*T/4.
          const float phase = WN * 1e-3f * float(uint32_t(now - pumpMs));
          pump = (SWING_KICK_COS ? cosf(phase) : sinf(phase)) >= 0 ? V_MAX : -V_MAX;
        } else if (fabsf(alpha) > 2.0f * PI_F / 3.0f &&
                   eTop > (energy >= E_OVER ? 2.02f : E_OVER)) {
          // After a detected full rotation, keep braking down toward E=2.02;
          // stopping at 2.2 leaves enough speed to miss the capture gate forever.
          // Going over the top (low speeds there, so the weak-magnet speed error
          // is small): u with the sign of alpha_dot removes energy, cos(a) < 0.
          pump = rate >= 0 ? V_MAX : -V_MAX;
        } else if (pumpSpeed > 0 && fabsf(alpha) < PI_F / 2) {
          // Lower half only: the arm is near rest while the pendulum approaches
          // the top, so the catch does not start with the arm at pump speed.
          float vp = pumpSpeed;
          if (SWING_MEAN_CENTER) {
            // Constant-per-cycle bias on the oscillation centre, with reserved
            // headroom: the final clamp never removes centering or pump speed.
            center = clamp(-MEAN_KP * thetaMean, -BIAS_MAX, BIAS_MAX);
            vp = fminf(pumpSpeed, V_MAX - fabsf(center));
          }
          // Lead centres the reversal (2vp/U long) on the bottom, + filter delay.
          const float lead = vp / U_MAX + 2.0f * DT;
          pump = sinf(alpha + rate * lead) >= 0 ? vp : -vp;
          pumpedLastHalf = true;
        }
        lastPump = pump; lastTargetRaw = pump + center;
        const float target = clamp(lastTargetRaw, -V_MAX, V_MAX);
        lastTarget = target;
        u = (target - velocity) / DT;
      }
    }
    if (mode == BALANCE) {
      u = 27.5553f * theta + 600.4973f * d + 14.2170f * velocity + 44.9091f * rate;
      if (uint32_t(now - balanceMs) > 1000)
        trim = clamp(trim - 0.023f * theta * DT, -TRIM_MAX, TRIM_MAX);
    }
    if (mode == BALANCE) {
      // Same gains, integration and trim as balance/balance.ino.
      // Do not zero velocity/rate at a moving catch: those are real states.
      // v2.4 catch phase: right after a moving catch use the older A limits
      // (100 rad/s^2, 360 deg/s; A balanced on hardware with them). Leave the
      // phase only once speed is back inside the proven 180 deg/s, so the
      // velocity limit never jumps. Then exactly the balance.ino limits.
      // After CATCH_MS the speed limit ramps down at the balance acceleration,
      // so the phase always ends within (CATCH_V_MAX - pi) / 60 s.
      float uLim = CATCH_U_MAX, vLim = CATCH_V_MAX;
      if (catching && uint32_t(now - balanceMs) >= CATCH_MS) {
        if (fabsf(velocity) <= BALANCE_V_MAX) catching = false;
        else { uLim = BALANCE_U_MAX; vLim = fabsf(velocity) - BALANCE_U_MAX * DT; }
      }
      if (!catching) { uLim = BALANCE_U_MAX; vLim = BALANCE_V_MAX; }
      vLim = fmaxf(vLim, BALANCE_V_MAX);
      u = clamp(u, -uLim, uLim);
      velocity = clamp(velocity + u * DT, -vLim, vLim);
      return; // Pump centering/braking must not override the proven A law.
    }
    u = clamp(u, -U_MAX, U_MAX);
    // One tick of latency plus stopping distance at the acceleration cap.
    // This constrains commanded motion; it cannot detect a stalled stepper.
    const float positiveV = fmaxf(velocity, 0), negativeV = fminf(velocity, 0);
    if (theta + positiveV * DT + positiveV * positiveV / (2 * U_MAX) > SOFT_ARM) {
      u = -U_MAX; braking = true;
    }
    if (theta + negativeV * DT - negativeV * negativeV / (2 * U_MAX) < -SOFT_ARM) {
      u = U_MAX; braking = true;
    }
    velocity = clamp(velocity + u * DT, -V_MAX, V_MAX);
  }
};
} // namespace Swing

#pragma once
#include "controller.h"
#include <stdio.h>
#include <string.h>

// Read-only observer. Never feeds values back to the control law.
namespace SwingDiag {
struct Before {
  Swing::Mode mode;
  int8_t sign;
  float alpha, peak, energy, velocity;
  explicit Before(const Swing::Controller &c)
      : mode(c.mode), sign(c.swingSign), alpha(c.previousAlpha),
        peak(c.swingPeak), energy(c.energy), velocity(c.velocity) {}
};
inline int16_t fixed(float v, float scale = 1000) {
  if (!isfinite(v)) return -32768;
  return int16_t(roundf(Swing::clamp(v * scale, -32767, 32767)));
}
struct Record {
  uint32_t ms;
  char kind;
  int8_t direction;
  uint8_t mask, reason, mode, events;
  int16_t x[22];
};
struct Logger {
  static const uint8_t CAPACITY = 4;
  Record queue[CAPACITY];
  uint8_t head = 0, count = 0;
  uint32_t dropped = 0, lastSample = 0, lastTop = 0;
  char line[240];
  uint16_t length = 0, offset = 0;
  float work[4] = {0,0,0,0};
  float reversalStart = 0;
  int8_t planSign = 0;
  bool completing = false;
  void resetAttempt() {
    for(uint8_t i=0;i<4;++i) work[i]=0;
    planSign=0; completing=false; reversalStart=0;
    lastSample=lastTop=0;
  }

  void emit(Record r, bool priority = false) {
    if (count == CAPACITY) {
      ++dropped;
      if (!priority) return;
      head = (head + 1) % CAPACITY; --count;
    }
    queue[(head + count) % CAPACITY] = r; ++count;
  }
  void message(uint32_t now, const Swing::Controller &c, uint8_t code) {
    Record r = {}; r.ms = now; r.kind = 'M'; r.mode = c.mode;
    r.reason = c.reason; r.x[0] = code; emit(r, true);
  }
  void observe(uint32_t now, float theta, float alpha,
               const Before &b, const Swing::Controller &c, bool enabled) {
    if(b.mode==Swing::IDLE && c.mode==Swing::COUNTDOWN) {
      resetAttempt();
    }
    if (b.mode != Swing::PUMP && c.mode != Swing::PUMP && c.mode!=Swing::BALANCE &&
        c.mode != Swing::PARK) return;
    // v2.3: the controller records its own pump component and targets (read-only).
    const bool pumping = c.mode==Swing::PUMP;
    const float pump = pumping ? c.lastPump : 0;
    const float raw = pumping ? c.lastTargetRaw : 0;
    const float clipped = pumping ? c.lastTarget : 0;
    const float z = Swing::WN*c.d+c.rate;
    const float a = 27.5553f*theta+600.4973f*c.d+14.2170f*b.velocity+44.9091f*c.rate;
    uint8_t mask = 0;
    if(fabsf(c.d)>=Swing::CAPTURE_D) mask|=1;
    if(fabsf(z)>=Swing::CAPTURE_Z) mask|=2;
    if(fabsf(theta)>=Swing::CAPTURE_ARM) mask|=4;
    if(fabsf(b.velocity)>Swing::BALANCE_V_MAX) mask|=8;
    if(fabsf(a)>1.25f*Swing::BALANCE_U_MAX) mask|=16;
    const bool turn = c.turnEvent==Swing::TURN_SEED || c.turnEvent==Swing::TURN_ACCEPT;
    const bool rejected = c.turnEvent>=Swing::TURN_REJECT_PHASE && c.turnEvent<=Swing::TURN_REJECT_SMALL;
    const bool candidate = turn || rejected;
    const bool rotation = c.turnEvent==Swing::TURN_ROTATION;
    const bool bottom = b.alpha*alpha<0 && fabsf(alpha)<Swing::PI_F/2 && fabsf(b.alpha)<Swing::PI_F/2;
    const int8_t sign = pump>0 ? 1 : (pump<0 ? -1 : 0);
    const bool reversal = sign && planSign && sign!=planSign;
    if(reversal) { reversalStart=b.velocity; completing=true; }
    if(sign) planSign=sign;
    const bool complete = completing && c.mode==Swing::PUMP && fabsf(c.velocity-clipped)<.05f;
    if(complete) completing=false;
    // Requested acceleration proxy at 200 Hz. Not mechanical work: encoder rate
    // is distorted, motor acceleration/position are unmeasured.
    if(c.mode==Swing::PUMP) {
      const uint8_t bin=(fabsf(alpha)>=Swing::PI_F/2?2:0)+(c.rate<0?1:0);
      work[bin]+=c.rate*cosf(alpha)*c.u*Swing::DT;
    }
    if(!enabled) {
      if(turn || rotation) for(uint8_t i=0;i<4;++i) work[i]=0;
      return;
    }
    char kind = 0;
    if(turn) kind='T'; else if(rejected) kind='J'; else if(rotation) kind='O'; else if(reversal) kind='R';
    else if(bottom) kind='B'; else if(complete) kind='E';
    else if(fabsf(c.d)<Swing::PI_F/4 && uint32_t(now-lastTop)>=50) {kind='N'; lastTop=now;}
    else if(uint32_t(now-lastSample)>=100) {kind='S'; lastSample=now;}
    if(!kind) return;
    Record r={}; r.ms=now; r.kind=kind; r.direction=c.rate<0?-1:1;
    r.mask=mask; r.mode=c.mode; r.reason=c.reason;
    r.events=(candidate?1:0)|(bottom?2:0)|(reversal?4:0)|(complete?8:0)|(rotation?16:0)|(turn?32:0)|(rejected?64:0);
    const float peak=candidate ? c.candidatePeak : c.swingPeak;
    r.x[0]=fixed(alpha); r.x[1]=fixed(peak);
    r.x[2]=fixed(b.energy); r.x[3]=fixed(c.energy); r.x[4]=fixed(c.loss);
    r.x[5]=fixed(c.pumpSpeed); r.x[6]=fixed(raw); r.x[7]=fixed(clipped);
    r.x[8]=fixed(b.velocity); r.x[9]=fixed(c.velocity);
    r.x[10]=fixed(z,100); r.x[11]=fixed(a,10);
    for(uint8_t i=0;i<4;++i) r.x[12+i]=fixed(work[i],1);
    r.x[16]=fixed(c.velocity-reversalStart); r.x[17]=fixed(theta);
    r.x[18]=fixed(c.rate,100); r.x[19]=fixed(c.trim); r.x[20]=fixed(c.u,10);
    r.x[21]=c.turnEvent;
    emit(r);
    if(turn || rotation) for(uint8_t i=0;i<4;++i) work[i]=0;
  }
  // Queue owns complete lines. Transport sends bounded chunks only when UART
  // has space. No Serial.print/float formatting on the diagnostic path.
  template<class SerialPort> void drain(SerialPort &port, uint32_t slackUs) {
    if(slackUs<1000) return;
    if(offset==length) {
      if(!count) return;
      Record &r=queue[head];
      int n=snprintf(line,sizeof(line),"#D,%lu,%c,%d,%u,%u,%u,%lu,%u",
        (unsigned long)r.ms,r.kind,int(r.direction),r.mode,r.reason,r.mask,(unsigned long)dropped,r.events);
      for(uint8_t i=0;i<22 && n>0 && n<int(sizeof(line));++i)
        n+=snprintf(line+n,sizeof(line)-n,",%d",int(r.x[i]));
      if(n<=0 || n>=int(sizeof(line))-1) { ++dropped; length=offset=0; }
      else {line[n++]='\n'; length=n; offset=0;}
      head=(head+1)%CAPACITY; --count;
    }
    int room=port.availableForWrite();
    uint16_t n=length-offset;
    if(n>16) n=16;
    if(room<0) return;
    if(n>uint16_t(room)) n=room;
    if(n) offset+=port.write((const uint8_t*)line+offset,n);
  }
};
} // namespace SwingDiag

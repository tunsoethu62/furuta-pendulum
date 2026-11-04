#include "../swingup/controller.h"
#include <assert.h>
#include <stdio.h>
using namespace Swing;
void sample(Controller &c,float from,float to,float rate){c.rate=rate;c.trackAmplitude(from,to);}
int main(){
  for(int side=-1;side<=1;side+=2){
    Controller c;
    sample(c,0,side*.283f,side*2); // initial outward motion
    sample(c,side*.283f,side*.263f,-side*1.28f);
    assert(c.turnEvent==TURN_SEED && c.seeded && !c.bottomQualified);
    float e=c.energy;c.pumpedLastHalf=true;
    sample(c,side*.263f,side*.280f,side*.77f);
    assert(c.turnEvent==TURN_REJECT_PHASE && c.energy==e && c.loss==LOSS0);
    sample(c,side*.280f,side*.252f,-side*2);
    assert(c.turnEvent==TURN_REJECT_PHASE && c.loss==LOSS0);
    // Zero-crossing jitter entirely within the hysteresis band is not a traversal.
    sample(c,side*.252f,side*.04f,-side*2);
    sample(c,side*.04f,-side*.04f,side*.7f);
    assert(!c.bottomQualified && c.energy==e);
    sample(c,-side*.04f,-side*.10f,-side*2);
    assert(c.bottomQualified && c.energy==e);
    sample(c,-side*.10f,-side*2.1f,-side*2);
    sample(c,-side*2.1f,-side*2.09f,side*.8f);
    assert(c.turnEvent==TURN_ACCEPT && !c.bottomQualified);
    assert(fabsf(c.energy-(1-cosf(2.1f)))<1e-5f);
    // A rejected local candidate cannot replace the physical signed extreme.
    float last=c.lastEnergy,loss=c.loss;
    sample(c,-side*2.09f,-side*2.11f,-side*.9f);
    assert(c.turnEvent==TURN_REJECT_PHASE && c.lastEnergy==last && c.loss==loss);
    // Full rotation is not a bottom traversal and invalidates accepted history.
    sample(c,side*3.13f,-side*3.13f,side*3);
    assert(c.turnEvent==TURN_ROTATION && !c.seeded && !c.bottomQualified && !c.lastEnergyValid);
    c.mode=PUMP;c.stop(KEY);c.tick(5000,0,0);
    assert(c.mode==IDLE && !c.seeded && !c.bottomQualified && c.turnEvent==TURN_NONE);
    for(int i=0;i<10;++i)c.tick(5005+i*5,0,0);
    c.zero();assert(c.start(5100,0,0,true));c.tick(8100,0,0);
    assert(c.mode==PUMP && !c.seeded && !c.bottomQualified);
  }
  Controller small;sample(small,0,.02f,1);sample(small,.02f,.01f,-1);
  assert(small.turnEvent==TURN_REJECT_SMALL && !small.seeded);
  puts("PASS: two-direction chatter replay, hysteretic traversal, one accepted update per half swing, preserved peaks, rotation/cancel/restart, first seed");
}

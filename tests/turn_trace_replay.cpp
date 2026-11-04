// Replay sampled diagnostic observations and raw-turn peaks. This checks the
// estimator on a fixed trace, not the changed plant's future trajectory.
#include "../swingup/controller.h"
#include <assert.h>
#include <stdio.h>
using namespace Swing;
int main(int argc,char **argv){
  assert(argc==2);FILE *f=fopen(argv[1],"r");assert(f);
  Controller c;float previous=0;unsigned ms;char kind;float alpha,peak,rate;
  int accepted=0,rejected=0,chatter=0;
  while(fscanf(f,"%u %c %f %f %f",&ms,&kind,&alpha,&peak,&rate)==5){
    if(kind=='T') {
      // Supply the stored between-snapshot extreme without inventing a new raw
      // sign change. Then apply the recorded candidate's filtered rate.
      c.rate=c.rawSwingSign ? c.rawSwingSign : (peak>=0?1:-1);
      c.trackAmplitude(previous,peak);previous=peak;
    }
    c.rate=rate;c.trackAmplitude(previous,alpha);previous=alpha;
    if(c.turnEvent==TURN_SEED || c.turnEvent==TURN_ACCEPT)++accepted;
    if(c.turnEvent>=TURN_REJECT_PHASE && c.turnEvent<=TURN_REJECT_SMALL)++rejected;
    if(ms==24134 || ms==24139) {
      assert(c.turnEvent==TURN_REJECT_PHASE && c.loss==LOSS0);
      assert(fabsf(c.energy-.04f)<.002f);++chatter;
    }
    if(c.pumpSpeed>0 && fabsf(alpha)<PI_F/2)c.pumpedLastHalf=true;
  }
  fclose(f);assert(chatter==2 && rejected>=3 && accepted>40);
  printf("PASS: fixed diagnostic trace replay: accepted=%d rejected=%d; both chatter updates rejected at loss=0.3\n",accepted,rejected);
}

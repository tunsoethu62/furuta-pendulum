#include "../swingup/controller.h"
#include <assert.h>
#include <stdio.h>
using namespace Swing;
int main(){
  assert(fabsf(CAPTURE_D-18*PI_F/180)<1e-6f);
  for(int sign=-1;sign<=1;sign+=2){
    for(int outside=0;outside<2;++outside){
      Controller c;c.mode=PUMP;c.previousValid=true;
      float d=sign*(outside?18.5f:17.5f)*PI_F/180;
      c.rate=-WN*d;c.previousAlpha=wrap(PI_F+d-c.rate*DT);
      c.tick(1000,0,wrap(PI_F+d));
      assert(c.mode==(outside?PUMP:BALANCE));
    }
    Controller still;still.mode=PUMP;still.previousValid=true;
    float alpha=wrap(PI_F+sign*17.5f*PI_F/180);
    still.previousAlpha=alpha;still.tick(1000,0,alpha);
    assert(still.mode==PUMP); // wider angle must not bypass motion/effort gates
  }
  puts("PASS: +/-17.5 moving catch, +/-18.5 rejection, other capture gates retained");
}

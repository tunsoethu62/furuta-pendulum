// Offline comparison only. Perfect commanded motor; reduced nonlinear model.
// A sampled successful rollout is NOT a hardware region-of-attraction proof.
#include "../swingup/controller.h"
#include <stdio.h>
using namespace Swing;
static float rad(float d){return d*PI_F/180;}
int main(){
  int eligible=0,held=0,arm=0,fell=0,rejected=0;
  const float angles[]={14,16,17.5f},speeds[]={3,4,5};
  const float arms[]={-15,0,15},velocities[]={-1,0,1},biases[]={-1,0,1};
  for(int side=-1;side<=1;side+=2)
  for(float deg: angles)
  for(float speed: speeds)
  for(float armDeg: arms)
  for(float armV: velocities)
  for(float biasDeg: biases) {
    float alpha=PI_F+side*rad(deg),omega=-side*speed;
    float theta=rad(armDeg),v=armV;
    const float bias=rad(biasDeg);
    Controller c;c.mode=PUMP;c.energy=2;c.previousValid=true;
    c.previousAlpha=wrap(alpha+bias-omega*DT);c.rate=omega;c.velocity=v;
    c.tick(1000,theta,wrap(alpha+bias));
    if(c.mode!=BALANCE){++rejected;continue;}++eligible;
    for(int n=0;n<1000 && c.mode==BALANCE;++n){
      float nv=fabsf(c.velocity)>=MIN_SPEED?c.velocity:0;
      float accel=(nv-v)/DT;
      for(int k=0;k<10;++k){
        float torque=-WN*WN*sinf(alpha)-2*.06f*WN*omega
          +COUPLING*cosf(alpha)*accel+v*v*sinf(alpha)*cosf(alpha)
          -.02f*WN*WN*tanhf(omega/.3f);
        omega+=torque*(DT/10);alpha+=omega*(DT/10);
      }
      v=nv;theta+=v*DT;
      if(fabsf(theta)>791/(3200/(2*PI_F))){c.stop(ARM);break;}
      float measured=roundf((alpha+bias)*4096/(2*PI_F))*(2*PI_F/4096);
      c.tick(1005+n*5,theta,wrap(measured));
    }
    if(c.mode==BALANCE)++held;else if(c.reason==ARM)++arm;else ++fell;
  }
  printf("capture_deg=%.1f grid=486 eligible=%d rejected=%d held5s=%d arm_stop=%d other_stop=%d\n",
    CAPTURE_D*180/PI_F,eligible,rejected,held,arm,fell);
}

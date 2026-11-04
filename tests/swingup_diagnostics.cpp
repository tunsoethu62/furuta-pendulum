#include "../swingup/diagnostics.h"
#include <assert.h>
#include <stdio.h>
struct Port {
  int room=0;
  char text[4096]={}; unsigned used=0;
  int availableForWrite(){return room;}
  unsigned write(const uint8_t *s,unsigned n){
    assert(n<=16 && n<=unsigned(room) && used+n<sizeof(text));
    memcpy(text+used,s,n); used+=n; return n;
  }
};
int main(int argc,char **argv){
  Swing::Controller plain, observed;
  plain.zero();observed.zero();
  assert(plain.start(0,0,0,true));assert(observed.start(0,0,0,true));
  SwingDiag::Logger log;
  for(unsigned t=0;t<24000;t+=5){
    float a=t<3000?0:2.7f*sinf((t-3000)*.012f);
    SwingDiag::Before before(observed);
    float theta=t<=3000?0:.15f;
    plain.tick(t,theta,a);observed.tick(t,theta,a);
    log.observe(t,theta,a,before,observed,true); // Full UART queue must not affect control.
    assert(plain.mode==observed.mode && plain.reason==observed.reason);
    assert(plain.u==observed.u && plain.velocity==observed.velocity);
    assert(plain.loss==observed.loss && plain.energy==observed.energy);
    assert(plain.rate==observed.rate && plain.trim==observed.trim);
  }
  assert(log.dropped>0 && log.count==log.CAPACITY);
  Port p;log.drain(p,5000);assert(p.used==0);
  p.room=3;log.drain(p,0);assert(p.used==0);
  for(int i=0;i<1000 && (log.count || log.offset<log.length);++i)log.drain(p,5000);
  assert(p.used>0 && !log.count && log.offset==log.length);
  assert(strncmp(p.text,"#D,",3)==0 && p.text[p.used-1]=='\n');
  unsigned commas=0;
  for(unsigned i=0;i<p.used;++i) {
    if(p.text[i]==',') ++commas;
    if(p.text[i]=='\n') {assert(commas==30);commas=0;}
  }
  if(argc>1) {FILE *f=fopen(argv[1],"wb");assert(f);assert(fwrite(p.text,1,p.used,f)==p.used);fclose(f);}
  // Full queue keeps terminal status by dropping an older unsent record.
  SwingDiag::Record r={};r.kind='S';for(int i=0;i<4;++i)log.emit(r);
  observed.stop(Swing::KEY);log.message(25000,observed,0);
  auto &last=log.queue[(log.head+log.count-1)%log.CAPACITY];
  assert(last.kind=='M' && last.mode==Swing::IDLE && last.reason==Swing::KEY);
  // Synthetic simultaneous bottom crossing + reversal must retain both flags.
  Swing::Controller c;c.mode=Swing::PUMP;c.pumpMs=0;c.energy=1;c.pumpSpeed=2;
  c.previousValid=true;c.previousAlpha=-.2f;c.rate=10;c.velocity=-1;
  SwingDiag::Logger events;events.planSign=-1;
  SwingDiag::Before b(c);c.tick(2000,0,.02f);
  events.observe(2000,0,.02f,b,c,true);
  assert(events.count && (events.queue[events.head].events&2) && (events.queue[events.head].events&4));
  // Rejected raw turns remain visible, with no accepted energy update.
  SwingDiag::Logger rejection;
  Swing::Controller chatter;chatter.mode=Swing::PUMP;chatter.rate=2;
  chatter.trackAmplitude(0,.283f);chatter.rate=-1.28f;chatter.trackAmplitude(.283f,.263f);
  SwingDiag::Before cb(chatter);chatter.rate=.77f;chatter.trackAmplitude(.263f,.280f);
  rejection.observe(24134,0,.280f,cb,chatter,true);
  auto &j=rejection.queue[rejection.head];
  assert(j.kind=='J' && (j.events&64) && !(j.events&32));
  assert(j.x[21]==Swing::TURN_REJECT_PHASE && j.x[2]==j.x[3]);
  puts("PASS: diagnostics preserve control outputs; overflow/backpressure, bounded writes, terminal priority, event flags");
}

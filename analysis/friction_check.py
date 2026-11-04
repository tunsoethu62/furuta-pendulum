"""Rough robustness check of the balance gains against hinge dry friction (stiction + Coulomb), the Timer1 speed
deadband and an upright bias. Crude plant model; indicative only. Run: uv run --no-project --with numpy --with scipy python analysis/friction_check.py"""
import numpy as np
exec(open("analysis/lqr_design.py",encoding="utf8").read().split("# ---- nonlinear simulation")[0])
q=2*np.pi/4096; TAU=1/(2*np.pi*25); dtc=dt
VMIN=31/(3200/(2*np.pi))  # rad/s
def sim(Kp=K, bias=0.0, Tc_ratio=0.0, vmin=VMIN, T=8.0, d0=np.radians(3)):
    x=np.array([0.0,d0,0.0,0.0]); vcmd=0.0; vprev=0.0; dprev=None; ddf=0.0; c=0.0; log=[]
    for k in range(int(T/dt)):
        dm=np.round((x[1]+bias)/q)*q-c
        dr=0.0 if dprev is None else (dm-dprev)/dt; dprev=dm; ddf+=(dt/(TAU+dt))*(dr-ddf)
        u=float(np.clip(-(Kd@np.array([x[0],dm,vcmd,ddf]))[0],-100,100))
        if k*dt>1: c=float(np.clip(c-0.023*x[0]*dt,-0.05,0.05))
        vcmd=float(np.clip(vcmd+u*dt,-2*np.pi,2*np.pi))
        vact=vcmd if abs(vcmd)>=vmin else 0.0
        for _ in range(10):
            h=dt/10
            thdd=(vact-x[2])/h if True else 0   # arm speed follows the (deadbanded) command at once
            thdd=float(np.clip(thdd,-1e4,1e4))
            a=x[1]+np.pi
            grav=-w_n**2*np.sin(a)+s*Kp*np.cos(a)*thdd-2*zeta*w_n*x[3]
            fr=Tc_ratio*w_n**2  # Tc/(J_p) = ratio*m g l/J_p = ratio*w_n^2
            if abs(x[3])>1e-3: add=-fr*np.sign(x[3])
            else: add=-np.clip(grav,-fr,fr)   # stiction
            add_dd=grav+add
            x=x+h*np.array([vact-x[2]+x[2], x[3], 0, add_dd]); x[2]=vact
        log.append((x[0],x[1],u))
        if abs(x[1])>np.radians(25): return "FELL at %.2fs"%(k*dt)
    L=np.array(log[-int(2/dt):]); 
    return "ok: last 2 s d range %.2f..%.2f deg, theta %.1f..%.1f deg, |u|rms %.0f"%(np.degrees(L[:,1].min()),np.degrees(L[:,1].max()),np.degrees(L[:,0].min()),np.degrees(L[:,0].max()),np.sqrt((L[:,2]**2).mean()))
for name,kw in [("nominal",{}),("no deadband",{"vmin":0}),("Tc 0.10",{"Tc_ratio":0.10}),("Tc 0.10 no deadband",{"Tc_ratio":0.10,"vmin":0}),("Tc 0.10 + bias 1deg",{"Tc_ratio":0.10,"bias":np.radians(1)}),("Tc 0.05",{"Tc_ratio":0.05}),("K x0.8 Tc0.1",{"Kp":K*0.8,"Tc_ratio":0.1})]:
    print(name.ljust(22),sim(**kw))

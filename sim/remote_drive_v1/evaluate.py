"""Declared test trajectories and event/shape/settling metrics. Missing = fail."""
import numpy as np
from model import NAMES


def mj(s):
    s=np.clip(s,0,1)
    return 10*s**3-15*s**4+6*s**5


def scenario(name, dt, reduced=False):
    small = 5 if name == "small5" else 10
    amplitude = small if name.startswith("small") else 45
    if name == "fast90_150ms":
        amplitude=90
    amp=np.array([0,min(amplitude,80),amplitude,min(amplitude*.8,72)]*4+[0,0,0,0],dtype=float)
    spread=np.array([8,2,-3,-8])*min(amplitude/10,1)
    amp[::4]=np.r_[spread, -min(amplitude,30)]
    amp[17:]=[min(amplitude,70),min(amplitude,90),min(amplitude,85)]
    base=np.zeros(20)
    duration=.15 if name == "fast90_150ms" else .25
    if name.startswith("small"):
        duration=.10
    moves=[]
    t0=.15
    # Each entry includes hold endpoint: missed crossings cannot be borrowed from another cycle.
    for a,b in [(base,amp),(amp,base),(base,amp*.6),(amp*.6,base)]:
        moves.append(dict(start=t0,end=t0+duration,hold_end=t0+duration+.20,
                          q0=a.copy(),q1=b.copy()))
        t0+=duration+.20
    if name == "upstream_hold":
        base=np.array([0,0,25,20]*4+[0,0,20,15],dtype=float)
        amp=base.copy()
        amp[1:16:4]=65; amp[:16:4]=spread
        amp[16:18]=[-25,55]
        moves=[];t0=.20
        for a,b in [(base,amp),(amp,base)]:
            moves.append(dict(start=t0,end=t0+.15,hold_end=t0+.45,q0=a.copy(),q1=b.copy()))
            t0+=.45
    if name == "rapid_reversal":
        moves=[];t0=.15
        for a,b in [(base,amp),(amp,base),(base,amp),(amp,base)]:
            moves.append(dict(start=t0,end=t0+.15,hold_end=t0+.15,q0=a.copy(),q1=b.copy()))
            t0+=.15
        moves[-1]["hold_end"]+=.3;t0+=.3
    if reduced:
        for move in moves:
            for k in [3,7,11,15]:
                move["q0"][k]=.8*move["q0"][k-1]
                move["q1"][k]=.8*move["q1"][k-1]
        base=moves[0]["q0"].copy()
    time=np.arange(0,t0+.05+dt/2,dt)
    command=np.repeat(base[None,:],len(time),axis=0)
    for move in moves:
        a,b=np.deg2rad(move["q0"]),np.deg2rad(move["q1"])
        mask=time>=move["start"]
        blend=mj((time[mask]-move["start"])/(move["end"]-move["start"]))
        # command is rad after first initialized segment; initialize separately below.
        command[mask]=a+(b-a)*blend[:,None]
    command[time<moves[0]["start"]]=np.deg2rad(base)
    return time,command,moves


def first_crossing(t,x,threshold,direction):
    indices=np.flatnonzero(direction*(x-threshold)>=0)
    if not len(indices):
        return None
    i=int(indices[0])
    if i==0:
        # Already beyond this target at segment start is not a measured crossing.
        return None
    delta=x[i]-x[i-1]
    return float(t[i-1]+(threshold-x[i-1])/delta*(t[i]-t[i-1])) if delta else float(t[i])


def evaluate(trace, moves, active):
    t=trace["time"]
    cmd=np.rad2deg(trace["command"]); q=np.rad2deg(trace["q"]);v=np.rad2deg(trace["v"])
    dt=t[1]-t[0]
    events=[]; settling=[]
    for mi,move in enumerate(moves):
        start,end,hold=move["start"],move["end"],move["hold_end"]
        for j,name in enumerate(NAMES):
            diff=move["q1"][j]-move["q0"][j]
            if abs(diff)<.1:
                continue
            sign=np.sign(diff)
            for fraction in [.1,.5,.9]:
                threshold=move["q0"][j]+diff*fraction
                mask=(t>=start-1e-10)&(t<=hold+1e-10)
                tc=first_crossing(t[mask],cmd[mask,j],threshold,sign)
                tr=first_crossing(t[mask],q[mask,j],threshold,sign)
                lag=None if tr is None else (tr-tc)*1000
                events.append(dict(move=mi,joint=name,fraction=fraction,direction=int(sign),
                    command_event_s=tc,actual_event_s=tr,lag_ms=lag,
                    status="missing" if tr is None else ("anticipatory" if lag < -10 else "reached")))
            if hold-end >= .04:
                mask=(t>=end)&(t<=hold)
                good=(np.abs(q[mask,j]-move["q1"][j])<=3)&(np.abs(v[mask,j])<=10)
                sustain=max(1,round(.03/dt))
                bad=np.flatnonzero(~good)
                first=int(bad[-1]+1) if len(bad) else 0
                # Must remain inside the band to the end of the observed hold.
                settled=len(good)-first>=sustain
                settling.append(dict(move=mi,joint=name,settle_after_command_end_ms=
                    float((t[mask][first]-end)*1000) if settled else None,
                    status="settled" if settled else "unsettled"))
    lags=np.array([e["lag_ms"] for e in events if e["lag_ms"] is not None])
    missing=sum(e["status"]=="missing" for e in events)
    anticipatory=sum(e["status"]=="anticipatory" for e in events)
    per_joint=[]
    for j,name in enumerate(NAMES):
        errors=q[:,j]-cmd[:,j]
        shifted=[]
        for shift in np.arange(0,.2001,.001):
            mask=t>=shift
            ref=np.interp(t[mask]-shift,t,cmd[:,j])
            shifted.append(np.sqrt(np.mean((q[mask,j]-ref)**2)))
        best=int(np.argmin(shifted))
        per_joint.append(dict(joint=name,raw_rmse_deg=float(np.sqrt(np.mean(errors**2))),
            raw_max_error_deg=float(np.max(np.abs(errors))),
            best_fit_lag_ms=best,delay_aligned_rmse_deg=float(shifted[best]),
            saturation_fraction=float(np.mean(trace["saturation"][:,j])) if j in active else None,
            max_motor_speed_deg_s=float(np.max(np.abs(np.rad2deg(trace["motor_speed"][:,j])))) if j in active else None))
    valid_max=float(np.max(lags)) if len(lags) else None
    limits=np.rad2deg(trace["joint_ranges"])
    range_overshoot=float(np.max(np.maximum(np.maximum(q-limits[:,1],limits[:,0]-q),0)))
    return dict(event_count=len(events),missing_events=missing,anticipatory_events=anticipatory,
        lag_median_ms=float(np.median(lags)) if len(lags) else None,
        lag_p95_ms=float(np.percentile(lags,95)) if len(lags) else None,
        valid_event_max_ms=valid_max,
        maximum_including_missing_ms=None if missing else valid_max,
        simulated_100ms_event_budget_met=bool(len(events)>0 and missing==0 and anticipatory==0 and valid_max<=100),
        simulated_50ms_event_target_met=bool(len(events)>0 and missing==0 and anticipatory==0 and valid_max<=50),
        unsettled_holds=sum(x["status"]=="unsettled" for x in settling),
        raw_max_error_deg=float(np.max(np.abs(q-cmd))),
        soft_joint_limit_max_overshoot_deg=range_overshoot,
        max_tendon_tension_N=float(np.max(trace["peak_tension"])),
        tendon_slack_fraction=float(np.mean(trace["slack"])),
        per_joint=per_joint,events=events,settling=settling,
        metric_notes=["Events include10/50/90% both directions; missing events fail and have no finite maximum.",
          "Best-fit lag is a separate retrospective shape metric, never substituted for raw errors.",
          "Settling: within3deg and10deg/s to the end of the observed hold, for at least30ms; tolerance provisional.",
          "Simulation event budget is not physical validation; historical ranges are not collision-verified."])

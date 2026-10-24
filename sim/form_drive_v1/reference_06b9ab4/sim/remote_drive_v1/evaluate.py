"""Declared test trajectories and event/shape/settling metrics. Missing = fail."""
import numpy as np
from model import NAMES

EARLY_CROSSING_CUTOFF_MS = -10.0
HOLD_QUALITY_PARAMETERS = dict(relative_amplitude_fraction=.1, minimum_tolerance_deg=.1,
    maximum_tolerance_deg=3., maximum_speed_deg_s=10., minimum_sustained_duration_s=.03,
    minimum_observed_hold_s=.04, reference="feasible command", status="provisional, not sensor accuracy")


def project_reference(requested, moves, reduced=False):
    """Preserve the requested task, separately construct a feasible virtual16 reference."""
    command=np.array(requested,copy=True)
    projected=[dict(m,q0=np.array(m["q0"],copy=True),q1=np.array(m["q1"],copy=True)) for m in moves]
    if reduced:
        command[:,[3,7,11,15]]=.8*command[:,[2,6,10,14]]
        for move in projected:
            for endpoint in ["q0","q1"]:
                move[endpoint][[3,7,11,15]]=.8*move[endpoint][[2,6,10,14]]
    return command,projected


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
    if name == "independent_distal":
        base=np.array([0,20,0,0]*4+[0,20,0,0],dtype=float)
        pip=base.copy();pip[[2,6,10,14,18]]=60
        both=pip.copy();both[[3,7,11,15,19]]=60
        dip=base.copy();dip[[3,7,11,15,19]]=60
        moves=[];t0=.20
        for a,b in [(base,pip),(pip,both),(both,dip),(dip,base)]:
            moves.append(dict(start=t0,end=t0+.25,hold_end=t0+.50,q0=a.copy(),q1=b.copy()))
            t0+=.50
    time=np.arange(0,t0+.05+dt/2,dt)
    command=np.repeat(base[None,:],len(time),axis=0)
    for move in moves:
        a,b=np.deg2rad(move["q0"]),np.deg2rad(move["q1"])
        mask=time>=move["start"]
        blend=mj((time[mask]-move["start"])/(move["end"]-move["start"]))
        # command is rad after first initialized segment; initialize separately below.
        command[mask]=a+(b-a)*blend[:,None]
    command[time<moves[0]["start"]]=np.deg2rad(base)
    if reduced:
        command,moves=project_reference(command,moves,True)
    return time,command,moves


def directional_crossing(t,x,threshold,direction):
    """First observed transition in the commanded direction, including a later recross."""
    signed=direction*(x-threshold)
    indices=np.flatnonzero((signed[:-1]<0)&(signed[1:]>=0))+1
    if not len(indices):
        return None
    i=int(indices[0])
    return float(t[i-1]+(threshold-x[i-1])/(x[i]-x[i-1])*(t[i]-t[i-1]))


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


def sustained_settling(t,q,v,target,end,hold,tolerance,dt,legacy_sample_count=False):
    mask=(t>=end)&(t<=hold)
    error=q[mask]-target
    good=(np.abs(error)<=tolerance)&(np.abs(v[mask])<=HOLD_QUALITY_PARAMETERS["maximum_speed_deg_s"])
    sustain=max(1,round(HOLD_QUALITY_PARAMETERS["minimum_sustained_duration_s"]/dt))
    bad=np.flatnonzero(~good)
    first=int(bad[-1]+1) if len(bad) else 0
    settled=(len(good)-first>=sustain) if legacy_sample_count else (
        first<len(good) and t[mask][-1]-t[mask][first]>=HOLD_QUALITY_PARAMETERS["minimum_sustained_duration_s"]-1e-12)
    return dict(settle_after_command_end_ms=float((t[mask][first]-end)*1000) if settled else None,
        status="settled" if settled else "unsettled",max_hold_error_deg=float(np.max(np.abs(error))),
        final_error_deg=float(error[-1]))


def evaluate(trace, moves, active):
    t=trace["time"]
    cmd=np.rad2deg(trace["command"]); q=np.rad2deg(trace["q"]);v=np.rad2deg(trace["v"])
    requested=np.rad2deg(trace.get("requested",trace["command"]))
    dt=t[1]-t[0]
    events=[]; settling=[];hold_quality=[];excursions=[]
    for mi,move in enumerate(moves):
        start,end,hold=move["start"],move["end"],move["hold_end"]
        for j,name in enumerate(NAMES):
            diff=move["q1"][j]-move["q0"][j]
            if hold-end >= HOLD_QUALITY_PARAMETERS["minimum_observed_hold_s"]:
                tolerance=max(HOLD_QUALITY_PARAMETERS["minimum_tolerance_deg"],
                    min(HOLD_QUALITY_PARAMETERS["maximum_tolerance_deg"],abs(diff)*HOLD_QUALITY_PARAMETERS["relative_amplitude_fraction"]))
                quality=sustained_settling(t,q[:,j],v[:,j],move["q1"][j],end,hold,tolerance,dt)
                hold_quality.append(dict(move=mi,joint=name,motion_amplitude_deg=float(abs(diff)),
                    tolerance_deg=float(tolerance),**quality))
            if abs(diff)<.1:
                continue
            sign=np.sign(diff)
            mask=(t>=start-1e-10)&(t<=hold+1e-10)
            displacement=sign*(q[mask,j]-move["q0"][j])
            excursions.append(dict(move=mi,joint=name,direction=int(sign),motion_amplitude_deg=float(abs(diff)),
                window_start_s=start,window_end_s=hold,
                endpoint_overshoot_deg=float(np.maximum(displacement-abs(diff),0).max()),
                wrong_way_excursion_deg=float(np.maximum(-displacement,0).max())))
            for fraction in [.1,.5,.9]:
                threshold=move["q0"][j]+diff*fraction
                tc=first_crossing(t[mask],cmd[mask,j],threshold,sign)
                tr=first_crossing(t[mask],q[mask,j],threshold,sign)
                if tc is None:
                    raise ValueError("Reference event has no directional crossing in its declared segment")
                initially_past=bool(sign*(q[mask,j][0]-threshold)>=0)
                later=directional_crossing(t[mask],q[mask,j],threshold,sign) if initially_past else None
                lag=None if tr is None else (tr-tc)*1000
                classification=("preexisting_level" if initially_past else ("never_reached" if tr is None else
                    ("early_crossing" if lag<EARLY_CROSSING_CUTOFF_MS else "reached")))
                events.append(dict(move=mi,joint=name,fraction=fraction,direction=int(sign),
                    command_event_s=tc,actual_event_s=tr,lag_ms=lag,
                    status="missing" if tr is None else ("anticipatory" if lag < EARLY_CROSSING_CUTOFF_MS else "reached"),
                    classification=classification,threshold_deg=float(threshold),motion_amplitude_deg=float(abs(diff)),
                    command_start_deg=float(move["q0"][j]),command_end_deg=float(move["q1"][j]),
                    actual_start_deg=float(q[mask,j][0]),actual_start_error_deg=float(q[mask,j][0]-move["q0"][j]),
                    initially_past_threshold=initially_past,timing_identifiable=tr is not None,
                    later_directional_crossing_s=later))
            if hold-end >= .04:
                legacy=sustained_settling(t,q[:,j],v[:,j],move["q1"][j],end,hold,3.,dt,legacy_sample_count=True)
                settling.append(dict(move=mi,joint=name,settle_after_command_end_ms=legacy["settle_after_command_end_ms"],
                    status=legacy["status"]))
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
            projection_loss_max_deg=float(np.max(np.abs(cmd[:,j]-requested[:,j]))),
            projection_loss_rmse_deg=float(np.sqrt(np.mean((cmd[:,j]-requested[:,j])**2))),
            task_raw_max_error_deg=float(np.max(np.abs(q[:,j]-requested[:,j]))),
            task_raw_rmse_deg=float(np.sqrt(np.mean((q[:,j]-requested[:,j])**2))),
            best_fit_lag_ms=best,delay_aligned_rmse_deg=float(shifted[best]),
            saturation_fraction=float(np.mean(trace["saturation"][:,j])) if j in active else None,
            max_motor_speed_deg_s=float(np.max(np.abs(np.rad2deg(trace["motor_speed"][:,j])))) if j in active else None))
    valid_max=float(np.max(lags)) if len(lags) else None
    limits=np.rad2deg(trace["joint_ranges"])
    range_overshoot=float(np.max(np.maximum(np.maximum(q-limits[:,1],limits[:,0]-q),0)))
    counts={name:sum(e["classification"]==name for e in events) for name in
            ["reached","early_crossing","preexisting_level","never_reached"]}
    moving=[name for j,name in enumerate(NAMES) if any(abs(m["q1"][j]-m["q0"][j])>=.1 for m in moves)]
    return dict(event_count=len(events),missing_events=missing,anticipatory_events=anticipatory,
        event_classification_counts=counts,indeterminate_events=counts["preexisting_level"]+counts["never_reached"],
        early_crossing_cutoff_ms=EARLY_CROSSING_CUTOFF_MS,commanded_joint_names=moving,
        held_joint_names=[name for name in NAMES if name not in moving],
        lag_median_ms=float(np.median(lags)) if len(lags) else None,
        lag_p95_ms=float(np.percentile(lags,95)) if len(lags) else None,
        valid_event_max_ms=valid_max,
        maximum_including_missing_ms=None if missing else valid_max,
        simulated_100ms_event_budget_met=bool(len(events)>0 and missing==0 and anticipatory==0 and valid_max<=100),
        simulated_50ms_event_target_met=bool(len(events)>0 and missing==0 and anticipatory==0 and valid_max<=50),
        unsettled_holds=sum(x["status"]=="unsettled" for x in settling),
        unsettled_quality_holds=sum(x["status"]=="unsettled" for x in hold_quality),
        hold_quality_parameters=HOLD_QUALITY_PARAMETERS,hold_quality=hold_quality,
        raw_max_error_deg=float(np.max(np.abs(q-cmd))),
        dynamic_raw_max_error_deg=float(np.max(np.abs(q-cmd))),
        dynamic_raw_rmse_deg=float(np.sqrt(np.mean((q-cmd)**2))),
        task_raw_max_error_deg=float(np.max(np.abs(q-requested))),
        task_raw_rmse_deg=float(np.sqrt(np.mean((q-requested)**2))),
        projection_loss_max_deg=float(np.max(np.abs(cmd-requested))),
        projection_loss_rmse_deg=float(np.sqrt(np.mean((cmd-requested)**2))),
        reference_policy=trace.get("reference_policy","independent20"),
        motion_excursions=excursions,
        max_endpoint_overshoot_deg=max((x["endpoint_overshoot_deg"] for x in excursions),default=0.),
        max_wrong_way_excursion_deg=max((x["wrong_way_excursion_deg"] for x in excursions),default=0.),
        soft_joint_limit_max_overshoot_deg=range_overshoot,
        max_tendon_tension_N=float(np.max(trace["peak_tension"])),
        tendon_slack_fraction=float(np.mean(trace["slack"])),
        per_joint=per_joint,events=events,settling=settling,
        metric_notes=["Legacy event fields keep CP004 semantics: preexisting levels and never-reached events are missing and fail the event budget.",
          "classification separates preexisting_level, never_reached, early_crossing and reached; later recrosses do not restore an identifiable first response.",
          "The -10ms early-crossing cutoff is a provisional phase-quality criterion, not the user's 100ms upper delay requirement.",
          "Threshold lag mixes phase delay, bias and shape; best-fit lag is not ground-truth transport delay.",
          "Best-fit lag is a separate retrospective shape metric, never substituted for raw errors.",
          "Legacy settling: moving axes within3deg and10deg/s through hold end for at least30ms. New hold_quality includes stationary axes with max(.1,min(3,.1*amplitude))deg, provisional.",
          "Projection loss compares requested and feasible commands; dynamic error compares actual and feasible; task error compares actual and requested.",
          "Endpoint overshoot and wrong-way/start excursion are separate segment-displacement metrics; neither is soft joint-limit overshoot.",
          "Simulation event budget is not physical validation; declared candidate ranges are not collision-verified."])

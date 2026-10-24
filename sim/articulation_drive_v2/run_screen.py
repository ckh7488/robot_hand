"""Four bounded finite-actuator diagnostics, with all misses and numerical failures.

No CAD generation, subprocesses, hardware, gain search or collision simulation.
The identity transmission is an unrealized decoupling reference, not a design.
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
import time as clock
from pathlib import Path

import numpy as np

import model_adapter as adapter

ROOT = adapter.ROOT
HERE = Path(__file__).resolve().parent
DT = (.000125, .0000625)
CASES = ("small5", "cad_fist")
CONVERGENCE = dict(small5_angle_limit_deg=.02, cad_fist_angle_limit_deg=1.,
    corresponding_lag_limit_ms=2., require_all_event_identities_and_classifications=True,
    require_all_hold_identities_and_statuses=True,
    interpretation="Predeclared numerical confidence only; fist 1 degree does not validate 0.1 degree stationary hold precision.")
FIST_TIMING = dict(initial_hold_s=.15, close_s=.6, closed_hold_s=.3, open_s=.6, open_hold_s=.3,
    interpolation="minimum jerk separately between each CAD fraction waypoint; reverse the waypoint sequence for opening",
    interpretation="Declared benchmark timing, not measured human motion; intermediate event windows stop at their waypoint.")


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {k:jsonable(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    return value


def scenario(legacy, design, name, dt):
    if name == "small5":
        t, q, moves = adapter.scenario(legacy, name, dt)
        return t, q, moves, dict(source="Pinned CP006 small5, unmodified angle signs and timings", moves=jsonable(moves))
    if name != "cad_fist":
        raise ValueError("Unknown bounded diagnostic case")
    path = design["fist_path"]
    fractions = np.asarray([p["fraction"] for p in path], dtype=float)
    angles = np.asarray([p["angles_deg"] for p in path], dtype=float)
    if (len(path)<2 or angles.shape!=(len(path),20) or not np.isfinite(angles).all()
            or not np.isfinite(fractions).all() or fractions[0]!=0 or fractions[-1]!=1
            or np.any(np.diff(fractions)<=0)):
        raise ValueError("Finite ordered CAD fist fractions 0..1 and explicit 20-axis poses required")
    if not np.allclose(angles[-1], design["fist_deg"], rtol=0, atol=1e-12):
        raise ValueError("CAD fist endpoint differs from fist_path")
    moves=[]
    close_start=FIST_TIMING["initial_hold_s"]
    close_end=close_start+FIST_TIMING["close_s"]
    open_start=close_end+FIST_TIMING["closed_hold_s"]
    end=open_start+FIST_TIMING["open_s"]+FIST_TIMING["open_hold_s"]
    for i in range(len(path)-1):
        start=close_start+fractions[i]*FIST_TIMING["close_s"]
        stop=close_start+fractions[i+1]*FIST_TIMING["close_s"]
        moves.append(dict(start=float(start), end=float(stop), hold_end=float(open_start if i==len(path)-2 else stop),
            q0=angles[i].copy(), q1=angles[i+1].copy(), phase="closing", waypoint_from=i, waypoint_to=i+1))
    for i in range(len(path)-1,0,-1):
        start=open_start+(1-fractions[i])*FIST_TIMING["open_s"]
        stop=open_start+(1-fractions[i-1])*FIST_TIMING["open_s"]
        moves.append(dict(start=float(start),end=float(stop),hold_end=float(end if i==1 else stop),
            q0=angles[i].copy(),q1=angles[i-1].copy(),phase="opening",waypoint_from=i,waypoint_to=i-1))
    t=np.arange(round(end/dt)+1,dtype=float)*dt
    q=np.repeat(np.deg2rad(angles[0])[None,:],len(t),axis=0)
    for move in moves:
        mask=t>=move["start"]-1e-12
        a,b=np.deg2rad(move["q0"]),np.deg2rad(move["q1"])
        blend=legacy["evaluate"].mj((t[mask]-move["start"])/(move["end"]-move["start"]))
        q[mask]=a+(b-a)*blend[:,None]
    return t,q,moves,dict(source="New CAD fist_path and fist_deg",timing=FIST_TIMING,
        waypoints=jsonable(path),moves=jsonable(moves),
        event_window_note="Intermediate waypoint misses remain missing. Later movement/holds cannot rescue a missed waypoint event; this is not a proof that a 100 ms delayed path is impossible.")


def check_command_ranges(design, command):
    limits=np.deg2rad([j["range_deg"] for j in adapter.kinematic_blueprint(design,[])["joints"]])
    if command.ndim!=2 or command.shape[1]!=20 or not np.isfinite(command).all():
        raise ValueError("Expected finite Nx20 command")
    if np.any(command<limits[:,0]-1e-12) or np.any(command>limits[:,1]+1e-12):
        raise ValueError("Benchmark exceeds new CAD ranges; no clipping or substitution permitted")


def compare_resolution(coarse, fine):
    if coarse is None or fine is None or not coarse[0].get("finite") or not fine[0].get("finite"):
        return dict(passed=False,status="missing_or_failed_run")
    ra,ta,qa=coarse;rb,tb,qb=fine
    name=ra["scenario"]
    sampled=np.column_stack([np.interp(ta,tb,qb[:,j]) for j in range(20)])
    per_joint=np.max(np.abs(np.rad2deg(qa-sampled)),axis=0)
    ea,eb=ra["metrics"]["events"],rb["metrics"]["events"]
    key=lambda e:(e["move"],e["joint"],e["fraction"])
    identity=[key(e) for e in ea]==[key(e) for e in eb]
    classifications=identity and all(a["classification"]==b["classification"] for a,b in zip(ea,eb))
    lag_pairs=[(a["lag_ms"],b["lag_ms"]) for a,b in zip(ea,eb)] if identity else []
    availability=identity and all((a is None)==(b is None) for a,b in lag_pairs)
    differences=[abs(a-b) for a,b in lag_pairs if a is not None and b is not None]
    lag=max(differences,default=None)
    ha,hb=ra["metrics"]["hold_quality"],rb["metrics"]["hold_quality"]
    holds=[(h["move"],h["joint"],h["status"]) for h in ha]==[(h["move"],h["joint"],h["status"]) for h in hb]
    limit=CONVERGENCE[name+"_angle_limit_deg"]
    passed=bool(per_joint.max()<=limit and identity and classifications and availability and holds
        and lag is not None and lag<=CONVERGENCE["corresponding_lag_limit_ms"])
    return dict(scenario=name,passed=passed,status="converged_under_declared_test" if passed else "numerical_confidence_failed",
        max_angle_difference_deg=float(per_joint.max()),per_joint_max_angle_difference_deg=per_joint.tolist(),
        max_event_lag_difference_ms=lag,angle_limit_deg=limit,event_identity_identical=identity,
        event_classifications_identical=classifications,lag_availability_identical=availability,
        hold_quality_classifications_identical=holds)


def actuator_summary(trace, cfg):
    diag=trace["diagnostics"]
    cap=cfg["stall_torque_Nm"]*cfg["current_limit_A"]/cfg["stall_current_A"]
    return dict(current_torque_proxy_cap_Nm=cap,
        current_definition="abs(stall_torque*(post-slew PWM - speed/no_load_speed)) > proxy torque cap; not measured phase current",
        voltage_definition="abs(raw held PWM request) > 1; separate from current and post-slew PWM",
        legacy_saturation_definition="motor envelope >= proxy cap OR abs(clipped held PWM target) >= .999",
        per_motor=[dict(joint=n,current_clipped_fraction=float(diag["current_clipped"][:,i].mean()),
            voltage_clipped_fraction=float(diag["voltage_clipped"][:,i].mean()),
            slew_limited_fraction=float(diag["slew_limited"][:,i].mean()),
            legacy_saturation_fraction=float(trace["saturation"][:,i].mean()),
            max_abs_electrical_torque_Nm=float(np.abs(diag["electrical_torque_applied_Nm"][:,i]).max()),
            max_abs_net_shaft_torque_Nm=float(np.abs(trace["torque"][:,i]).max()),
            max_abs_motor_speed_rpm=float(np.abs(trace["motor_speed"][:,i]).max()*60/(2*np.pi)))
            for i,n in enumerate(adapter.NAMES)])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--contract",type=Path,required=True)
    p.add_argument("--parameters",type=Path,default=HERE/"parameters.json")
    p.add_argument("--reference-root",type=Path,default=ROOT/"sim/form_drive_v1/reference_06b9ab4")
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--traces",type=Path,required=True)
    a=p.parse_args()
    for directory in (a.output,a.traces):
        if directory.exists():
            raise ValueError("Use a fresh output/trace directory; previous evidence is preserved")
    a.output.mkdir(parents=True);a.traces.mkdir(parents=True)
    contract=adapter.validate_contract(json.loads(a.contract.read_text(encoding="utf-8")),repo_root=ROOT)
    params=json.loads(a.parameters.read_text(encoding="utf-8"));cfg=params["parameters"]
    origin=params["source_provenance"]
    origin_path=ROOT/origin["path"]
    if adapter.digest(origin_path)!=origin["sha256"] or cfg!=json.loads(origin_path.read_text(encoding="utf-8"))["parameters"]:
        raise ValueError("Declared unchanged parameter provenance does not match")
    if not np.array_equal(contract["transmission"]["motor_angle_jacobian"],np.eye(20)) or contract["transmission"]["cable_actuator_indices"]!=list(range(20)):
        raise ValueError("This bounded screen requires the explicitly unrealized identity, cable20 reference")
    legacy=adapter.load_legacy(a.reference_root)
    paths={str((ROOT/rel).resolve()):expected for rel,expected in contract["source_sha256"].items()}
    for path in (a.contract,a.parameters,Path(__file__),HERE/"test_adapter.py",HERE/"test_screen.py",ROOT/adapter.SHARED_REL,origin_path):
        paths[str(path.resolve())]=adapter.digest(path)
    for rel in ("provenance.json","hand_follow/cad_hand.py","hand_follow/retarget.py","hand_follow/__init__.py"):
        path=HERE/"reference_webcam_31ba43f"/rel
        paths[str(path.resolve())]=adapter.digest(path)
    for rel,expected in legacy["source_sha256"].items():
        paths[str((a.reference_root/rel).resolve())]=expected
    manifest=[dict(path=Path(path).relative_to(ROOT).as_posix() if Path(path).is_relative_to(ROOT) else path,
        sha256=expected) for path,expected in paths.items()]
    guard=lambda:all(adapter.digest(path)==expected for path,expected in paths.items())
    if not guard():raise ValueError("Input hashes already stale")
    write_json(a.output/"input_manifest.json",manifest)
    write_json(a.output/"parameters.json",params)
    write_json(a.output/"declared_protocol.json",dict(cases=CASES,dt_s=DT,fist_timing=FIST_TIMING,
        convergence=CONVERGENCE,transmission=contract["transmission"],force_path="Nominal finite CP006; no overrides",
        command_received_to_servo_s=cfg["received_to_servo_s"],hardware_100ms_proven=False))
    records=[];runs={}
    for name in CASES:
        for dt in DT:
            label=f"{name}_{dt*1e6:g}us";started=clock.monotonic()
            record=dict(label=label,scenario=name,dt_s=dt,finite=False)
            try:
                if not guard():raise ValueError("Input changed before case")
                parameters={**cfg,"dt_s":dt}
                system=adapter.build_system(a.contract,a.reference_root,parameters=parameters,allow_conceptual=True,repo_root=ROOT)
                t,command,moves,generation=scenario(legacy,contract["design"],name,dt)
                check_command_ranges(contract["design"],command)
                trace=adapter.run_simulation(system,t,command,record_diagnostics=True)
                metrics=legacy["evaluate"].evaluate(trace,moves,system["active"])
                if not all(np.isfinite(value).all() for value in trace.values() if isinstance(value,np.ndarray)):
                    raise ValueError("Nonfinite trace")
                metrics["command_generation"]=generation
                arrays={k:v for k,v in trace.items() if isinstance(v,np.ndarray)}
                arrays.update({"diag_"+k:v for k,v in trace["diagnostics"].items()})
                trace_path=a.traces/(label+".npz")
                np.savez_compressed(trace_path,**arrays)
                stride=max(1,round(.002/dt));indices=np.unique(np.r_[np.arange(0,len(t),stride),len(t)-1])
                sampled=dict(case=name,variant="articulation20_ideal_decoupling_reference",joint_names=adapter.NAMES,
                    time_s=t[indices].tolist(),requested_deg=np.rad2deg(command[indices]).tolist(),
                    command_deg=np.rad2deg(command[indices]).tolist(),actual_deg=np.rad2deg(trace["q"][indices]).tolist(),
                    metadata=dict(full_rate_trace=str(trace_path),dt_s=dt,plot_sample_s=stride*dt,
                        hardware_performance_validated=False,collisions_modeled=False))
                write_json(a.output/(label+"_trajectory.json"),sampled)
                record.update(finite=True,status="completed_diagnostic_not_hardware_approval",parameters=parameters,
                    contract_sha256=system["contract_sha256"],provenance=system["provenance"],metrics=metrics,
                    actuators=actuator_summary(trace,parameters),diagnostic_options=trace["diagnostic_options"],
                    trace=dict(path=str(trace_path),sha256=adapter.digest(trace_path),samples=len(t),
                        format="lossless compressed NumPy arrays; rad, s, Nm, N; diag_ fields retain CP006 definitions"))
                runs[(name,dt)]=(record,t,trace["q"])
                del trace,arrays,system
            except Exception as exc:
                record.update(status="execution_failed",error=type(exc).__name__+": "+str(exc))
            record["elapsed_s"]=clock.monotonic()-started
            records.append(record);write_json(a.output/(label+".json"),record)
            print(json.dumps({k:record[k] for k in ("label","status","finite","elapsed_s")}),flush=True)
    convergence=[compare_resolution(runs.get((name,DT[0])),runs.get((name,DT[1])))|dict(scenario=name) for name in CASES]
    unchanged=guard()
    fields=["event_count","event_classification_counts","missing_events","anticipatory_events","valid_event_max_ms",
        "raw_max_error_deg","unsettled_quality_holds","simulated_100ms_event_budget_met","max_tendon_tension_N"]
    summary=dict(schema_version="articulation_drive_v2.screen.v1",status="conceptual_diagnostic_only" if unchanged else "invalid_inputs_changed",
        inputs_unchanged=unchanged,hardware_100ms_proven=False,collision_free_motion_proven=False,
        convergence_criteria=CONVERGENCE,convergence=convergence,passed_convergence=sum(c["passed"] for c in convergence),
        cases=[{k:r[k] for k in ("label","scenario","dt_s","finite","status")}|({k:r["metrics"][k] for k in fields} if r["finite"] else {"error":r["error"]}) for r in records],
        limitations=contract["limitations"]+contract["transmission"]["assumptions"]+[
            "Unconverged cases cannot support numerical timing/accuracy conclusions.",
            "Event budgets are separate from raw task errors, stationary hold quality, and actual assembly collisions.",
            "The 1 degree fist convergence bound does not demonstrate sub-degree or 0.1 degree hold numerical accuracy.",
            "The same-time maximum error is retained; retimed or delay-aligned error cannot replace it."],
        runtime=dict(python=sys.version,numpy=np.__version__,mujoco=legacy["model"].mujoco.__version__))
    write_json(a.output/"summary.json",summary)
    if not unchanged or not all(r["finite"] for r in records):raise SystemExit(2)


if __name__=="__main__":
    main()

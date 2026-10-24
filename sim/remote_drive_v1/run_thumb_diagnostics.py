"""Six bounded opposition diagnostics, never a physical controller/motor claim."""
import argparse
import csv
import gzip
import hashlib
import io
import json
from pathlib import Path
import platform

import mujoco
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from model import HERE, ROOT, NAMES, DEFAULTS, build_model, run_simulation
from evaluate import scenario, evaluate
from run_suite import command_metadata, write_json

AXIS=17
CASES=["whole_hand_nominal","opposition_only_nominal","opposition_only_zero_gravity",
       "opposition_only_no_pwm_slew","opposition_only_ideal_torque",
       "opposition_only_ideal_torque_critical_damping"]


def diagnostic_command(dt,isolated):
    time,commands,moves=scenario("rapid_reversal",dt)
    if isolated:
        mask=np.arange(20)!=AXIS
        commands[:,mask]=0
        for move in moves:
            move["q0"][mask]=0;move["q1"][mask]=0
    return time,commands,moves


def damping_rule(system):
    """Predeclared scalar diagnostic: max locked-other-axis inertia at0:5:45deg.

    This is not a closed-loop critical-damping calculation for the whole hand.
    Free-other-axis instantaneous inertia is also recorded to expose the distinction.
    """
    m,d=system["model"],system["data"]
    samples=[]
    for angle in np.arange(0,46,5):
        mujoco.mj_resetData(m,d)
        d.qpos[AXIS]=np.deg2rad(angle)
        for k,ix in system["rotor_index"].items():d.qpos[ix]=(system["A"]@d.qpos[:20])[k]
        mujoco.mj_forward(m,d)
        mass=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,mass,d.qM)
        samples.append(dict(opposition_deg=float(angle),joint_angles_deg=np.rad2deg(d.qpos[:20]).tolist(),
            locked_other_coordinates_inertia_kg_m2=float(mass[AXIS,AXIS]),
            free_other_coordinates_inertia_kg_m2=float(1/np.linalg.inv(mass)[AXIS,AXIS])))
    selected=max(x["locked_other_coordinates_inertia_kg_m2"] for x in samples)
    kp=system["config"]["kp_Nm_per_rad"]
    return dict(rule="Kd=2*sqrt(Kp*max(M[17,17])) at opposition0:5:45deg, all other joints0deg",
        declared_before_trajectory_runs=True,sampled_inertia_definition="M[17,17]: all other generalized accelerations constrained to zero",
        free_inertia_definition="1/inv(M)[17,17]: all other generalized forces zero, instantaneous free-coordinate comparison only",
        samples=samples,selected_inertia_kg_m2=selected,kp_Nm_per_rad=kp,
        kd_override_Nm_s_per_rad=float(2*np.sqrt(kp*selected)),
        scope="Frozen-pose scalar diagnostic, not a calibrated or optimal real-servo gain; nominal gains remain unchanged")


def summarize(trace,metrics):
    log=trace["diagnostics"];dt=float(trace["time"][1]-trace["time"][0])
    power=log["net_shaft_power_W"][:,AXIS]
    events=[e for e in metrics["events"] if e["joint"]==NAMES[AXIS]]
    classifications={label:sum(e["classification"]==label for e in events) for label in
                     ["reached","early_crossing","preexisting_level","never_reached"]}
    fractions={key+"_fraction":float(np.mean(log[key][:,AXIS])) for key in
        ["current_clipped","voltage_clipped","slew_limited","applied_current_clipped",
         "applied_voltage_clipped","applied_slew_limited","braking"]}
    axis_metrics=next(row for row in metrics["per_joint"] if row["joint"]==NAMES[AXIS])
    return dict(axis=AXIS,joint=NAMES[AXIS],**fractions,axis_metrics=axis_metrics,
        axis_event_classification_counts=classifications,axis_event_count=len(events),
        maximum_abs_pd_request_Nm=float(np.max(np.abs(log["pd_request_Nm"][:,AXIS]))),
        maximum_abs_net_shaft_torque_Nm=float(np.max(np.abs(trace["torque"][:,AXIS]))),
        peak_commanded_motor_speed_rpm=float(np.max(np.abs(log["motor_velocity_target_rad_s"][:,AXIS]))*60/(2*np.pi)),
        positive_shaft_work_J=float(np.maximum(power[:-1],0).sum()*dt),
        signed_negative_shaft_work_J=float(np.minimum(power[:-1],0).sum()*dt),
        power_integral_method="Left rectangle over[time[0],time[-1]); final logged force excluded",
        power_scope="Net shaft mechanical power; negative work is not evidence of electrical regeneration")


def write_curve(path,trace):
    """Lossless full-rate axis17 CSV; deterministic gzip header for reproduction."""
    columns=dict(time_s=trace["time"],requested_joint_angle_rad=trace["command"][:,AXIS],
        actual_joint_angle_rad=trace["q"][:,AXIS],actual_joint_velocity_rad_s=trace["v"][:,AXIS],
        motor_position_rad=trace["motor_pos"][:,AXIS],motor_speed_rad_s=trace["motor_speed"][:,AXIS],
        legacy_saturation=trace["saturation"][:,AXIS])
    columns.update({key:value[:,AXIS] for key,value in trace["diagnostics"].items()})
    with Path(path).open("wb") as raw:
        with gzip.GzipFile(filename="",mode="wb",fileobj=raw,mtime=0) as compressed:
            with io.TextIOWrapper(compressed,encoding="utf-8",newline="") as text:
                writer=csv.writer(text);writer.writerow(columns)
                for row in zip(*columns.values()):
                    writer.writerow([int(x) if isinstance(x,(bool,np.bool_)) else float(x) for x in row])
    return dict(path=Path(path).name,format="gzip CSV, full physics rate, float64 round-trip values",
        samples=len(trace["time"]),columns=list(columns),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


def plot_diagnostics(traces,summaries,output):
    fig,axes=plt.subplots(3,2,figsize=(14,12),sharex=True)
    for ax,(name,trace) in zip(axes.ravel(),traces.items()):
        ax.plot(trace["time"],np.rad2deg(trace["command"][:,AXIS]),"k--",lw=1,label="Received target")
        ax.plot(trace["time"],np.rad2deg(trace["q"][:,AXIS]),color="#007f89",label="Actual opposition")
        ax.set_title(name.replace("opposition_only_","isolated ").replace("_"," "),fontsize=10)
        ax.set_ylabel("Angle (deg)");ax.grid(alpha=.25)
        summary=summaries[name]
        ax.text(.02,.96,f"Raw max error {summary['axis_metrics']['raw_max_error_deg']:.2f} deg\n"
            f"Applied current/voltage/slew limits: {summary['applied_current_clipped_fraction']:.0%} / "
            f"{summary['applied_voltage_clipped_fraction']:.0%} / {summary['applied_slew_limited_fraction']:.0%}",
            transform=ax.transAxes,va="top",fontsize=8)
    for ax in axes[-1]:ax.set_xlabel("Time (s)")
    axes[0,0].legend(fontsize=8,loc="lower left")
    fig.suptitle("Opposition diagnostics | ideal branches are NOT hardware candidates",fontsize=14)
    fig.tight_layout();fig.savefig(output/"opposition_comparison.png",dpi=140);plt.close(fig)
    trace=traces["whole_hand_nominal"];log=trace["diagnostics"];t=trace["time"]
    fig,axes=plt.subplots(3,1,figsize=(13,10),sharex=True)
    for key in ["pd_request_Nm","electrical_torque_applied_Nm","net_shaft_torque_Nm"]:
        axes[0].plot(t,log[key][:,AXIS],label=key,lw=1)
    for key in ["pwm_request_raw","pwm_request_clipped","pwm_applied"]:
        axes[1].plot(t,log[key][:,AXIS],label=key,lw=1)
    for offset,key in enumerate(["current_clipped","voltage_clipped","slew_limited"]):
        axes[2].step(t,log[key][:,AXIS].astype(float)+offset*1.3,label=key,lw=.8,where="post")
    for ax,label in zip(axes,["Torque (N m)","PWM duty","Separate limiter flags"]):
        ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend(fontsize=8)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Whole-hand nominal opposition: held PD/PWM request and applied limits")
    fig.tight_layout();fig.savefig(output/"nominal_actuator_signals.png",dpi=140);plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,default=HERE/"results/thumb_diagnostics")
    parser.add_argument("--dt",type=float,default=DEFAULTS["dt_s"])
    parser.add_argument("--input-label",default="Current declared CAD inputs")
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    paths=[HERE/name for name in ["model.py","evaluate.py","run_suite.py","run_thumb_diagnostics.py"]]
    paths += [ROOT/name for name in ["cad/modular_roots_v9/design.json","cad/modular_roots_v9/mass_screen.json",
        "cad/modular_roots_v9/scene.json","cad/modular_roots_v9/cad_manifest.json","cad/modular_roots_v9/kinematics_numpy.py",
        "cad/remote_drive_v1/design.json","cad/remote_drive_v1/simulation_contract.json"]]
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    rule=damping_rule(build_model("remote20",dict(dt_s=args.dt)))
    results={};traces={};summaries={}
    for name in CASES:
        overrides=dict(dt_s=args.dt)
        if name.endswith("zero_gravity"):overrides["gravity_m_s2"]=[0.,0.,0.]
        system=build_model("remote20",overrides)
        isolated=name!="whole_hand_nominal"
        t,command,moves=diagnostic_command(args.dt,isolated)
        options=dict(axis=AXIS,actuator_mode="nominal")
        if name.endswith("no_pwm_slew"):options["actuator_mode"]="no_pwm_slew"
        if "ideal_torque" in name:options["actuator_mode"]="ideal_pd_torque"
        if name.endswith("critical_damping"):options["kd_override_Nm_s_per_rad"]=rule["kd_override_Nm_s_per_rad"]
        trace=run_simulation(system,t,command,record_diagnostics=True,diagnostic=options)
        metrics=evaluate(trace,moves,system["active"])
        metrics["command_generation"]=command_metadata(t,moves,moves,"independent20")
        summary=summarize(trace,metrics)
        curve=write_curve(args.output/(name+"_opposition.csv.gz"),trace)
        results[name]=dict(diagnostic_only=True,physical_acceptance_eligible=False,
            command_scope="Only opposition commanded to move; other joints remain dynamically free under their nominal holding servos" if isolated else "Original full-hand CP005 rapid reversal",
            parameters=system["config"],parameter_overrides=overrides,
            actuator_options=trace["diagnostic_options"],metrics=metrics,summary=summary,curve=curve)
        traces[name]=trace;summaries[name]=summary
        print(json.dumps(dict(case=name,max_error_deg=summary["axis_metrics"]["raw_max_error_deg"],
            current=summary["applied_current_clipped_fraction"],voltage=summary["applied_voltage_clipped_fraction"],
            slew=summary["applied_slew_limited_fraction"])),flush=True)
    for path in paths:
        if hashlib.sha256(path.read_bytes()).hexdigest()!=hashes[path.relative_to(ROOT).as_posix()]:
            raise RuntimeError("Input changed during diagnostics: "+str(path))
    report=dict(schema_version=1,status="Fault-isolation simulations; ideal branches are not motor/controller proposals or hardware performance",
        engine="MuJoCo "+mujoco.__version__,python=platform.python_version(),input_label=args.input_label,
        source_hashes=hashes,axis=AXIS,joint=NAMES[AXIS],joint_names=NAMES,case_count=len(results),damping_rule=rule,cases=results,
        signal_definitions={
            "timestamp":"Position/velocity are logged at t before the step; torque/PWM at t act over[t,t+dt). Last logged force is outside the observed trajectory window.",
            "held_signals":"Motor targets, PD request and raw/clipped PWM targets update only at the controller period and are held between updates.",
            "current_clipped":"abs(stall_torque*(pwm_applied-speed/w0)) > current-proxy torque cap; strict inequality.",
            "voltage_clipped":"abs(held raw PWM request)>1; strict inequality, distinct from the legacy near-limit flag.",
            "slew_limited":"abs(clipped PWM target-pwm_applied)>1e-12.",
            "legacy_saturation":"Unchanged CP005: abs(unlimited electrical torque)>=cap OR abs(clipped PWM target)>=.999.",
            "ideal_branch":"Selected axis PD torque bypasses voltage/current/PWM limits; envelope/PWM are counterfactual there. Applied limiter flags are false. Other actuators stay nominal.",
            "torques":"Electrical torque before viscous motor loss; net shaft torque=electrical torque-B*measured motor speed. Ideal branch retains B.",
            "braking":"Net shaft torque*measured motor speed < -1e-12 W; negative mechanical work does not prove regenerated electrical energy."},
        limitations=["No radius, nominal gain, motor rating, preload or friction changes in the nominal branches.",
            "Ideal torque and damping branches isolate model causes; their favorable metrics cannot justify physical50/100ms claims.",
            "The damping rule is a scalar frozen-pose proxy, not the critical damping of the coupled closed-loop system.",
            "Motor current/torque/PWM and cable mechanics remain uncalibrated; CAD collision contacts are disabled."])
    write_json(args.output/"report.json",report)
    with (args.output/"summary.csv").open("w",encoding="utf-8",newline="") as output:
        writer=csv.writer(output);writer.writerow(["case","axis_max_error_deg","applied_current_clip_fraction","applied_voltage_clip_fraction","applied_slew_limit_fraction","braking_fraction","positive_shaft_work_J","signed_negative_shaft_work_J"])
        for name,row in summaries.items():writer.writerow([name,row["axis_metrics"]["raw_max_error_deg"],row["applied_current_clipped_fraction"],row["applied_voltage_clipped_fraction"],row["applied_slew_limited_fraction"],row["braking_fraction"],row["positive_shaft_work_J"],row["signed_negative_shaft_work_J"]])
    plot_diagnostics(traces,summaries,args.output)
    print(str(args.output/"report.json"),flush=True)


if __name__=="__main__":main()

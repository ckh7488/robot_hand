"""Reproduce model cases, sensitivities, plots and compact user-review traces."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import sys

import mujoco
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from model import HERE, ROOT, NAMES, DEFAULTS, build_model, run_simulation
from evaluate import scenario, evaluate


def write_json(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")


def plot_trace(traces,output):
    fig,axes=plt.subplots(4,2,figsize=(14,12),sharex=True)
    selected=[4,5,6,7,16,17,18,19]
    for ax,j in zip(axes.ravel(),selected):
        ref=next(iter(traces.values()))
        ax.plot(ref["time"],np.rad2deg(ref["command"][:,j]),"k--",lw=1.3,label="Received target")
        for variant,trace in traces.items():
            ax.plot(trace["time"],np.rad2deg(trace["q"][:,j]),label=variant,lw=1)
        ax.set_title(NAMES[j]);ax.set_ylabel("Joint angle (deg)");ax.grid(alpha=.25)
    for ax in axes[-1]:ax.set_xlabel("Time since command stream start (s)")
    axes[0,0].legend(fontsize=8)
    fig.suptitle("Full-hand coupled dynamics | uncalibrated model, not hardware performance")
    fig.tight_layout();fig.savefig(output,dpi=140);plt.close(fig)


def compact_trace(trace,variant,case):
    skip=max(1,round(.01/(trace["time"][1]-trace["time"][0])))
    return dict(joint_names=NAMES,time_s=np.round(trace["time"][::skip],6).tolist(),
        command_deg=np.round(np.rad2deg(trace["command"][::skip]),4).tolist(),
        actual_deg=np.round(np.rad2deg(trace["q"][::skip]),4).tolist(),variant=variant,case=case,
        metadata=f"100Hz display samples; physics timestep is{trace['time'][1]-trace['time'][0]}s. Not physical or collision validation.")


def main():
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,default=HERE/"results")
    p.add_argument("--quick",action="store_true");p.add_argument("--dt",type=float,default=DEFAULTS["dt_s"])
    p.add_argument("--variants",nargs="+",default=["baseline20","remote20","remote16"])
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    files=[HERE/"model.py",HERE/"evaluate.py",HERE/"run_suite.py",ROOT/"cad/modular_roots_v9/design.json",
           ROOT/"cad/modular_roots_v9/mass_screen.json",ROOT/"cad/modular_roots_v9/scene.json",
           ROOT/"cad/modular_roots_v9/cad_manifest.json",ROOT/"cad/modular_roots_v9/kinematics_numpy.py",
           ROOT/"cad/remote_drive_v1/design.json",ROOT/"robot_config.json",ROOT/"cad/remote_drive_v1/simulation_contract.json"]
    source_hashes={str(f.relative_to(ROOT)).replace("\\","/"):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    cases=["small5","small10","large45","fast90_150ms","upstream_hold","rapid_reversal"]
    if a.quick:cases=["large45","rapid_reversal"]
    results={}; plotting={}; provenance={}
    for variant in a.variants:
        system=build_model(variant,dict(dt_s=a.dt),a.output)
        provenance[variant]=dict(mass_source=system["mass_provenance"],body_mass_properties=system["mass_properties"],
            actuator_count=len(system["active"]),physical_joint_count=20,simulated_coordinates=system["model"].nq,
            motor_angle_from_joint_angle_matrix=system["A"].tolist(),parameters=system["config"])
        for case in cases:
            time,commands,moves=scenario(case,a.dt,reduced=variant=="remote16")
            trace=run_simulation(system,time,commands)
            metrics=evaluate(trace,moves,system["active"])
            results[variant+"/"+case]=metrics
            print(json.dumps(dict(variant=variant,case=case,missing=metrics["missing_events"],
                                 max_ms=metrics["valid_event_max_ms"],error=metrics["raw_max_error_deg"])),flush=True)
            if case=="rapid_reversal":
                plotting[variant]=trace
                write_json(a.output/(variant+"_trajectory.json"),compact_trace(trace,variant,case))
    sensitivities={}
    settings={"horizontal_gravity":{"gravity_m_s2":[0,0,-9.81]},
        "higher_cable_friction":{"cable_mu":.12},"soft_cable":{"cable_stiffness_N_per_m":5000},
        "low_preload":{"cable_preload_N":.5},"mass_1_5x":{"mass_scale":1.5},
        "reduced_upstream_coupling":{"cable_upstream_coefficient":.3},
        "negative_upstream_coupling":{"cable_upstream_coefficient":-1},
        "slow_command_bus":{"received_to_servo_s":.008,"controller_period_s":.005}}
    if not a.quick:
        for label,parameters in settings.items():
            system=build_model("remote20",dict(dt_s=a.dt,**parameters))
            time,commands,moves=scenario("rapid_reversal",a.dt)
            trace=run_simulation(system,time,commands)
            sensitivities[label]=evaluate(trace,moves,system["active"])
            sensitivities[label]["parameter_overrides"]=parameters
    for f in files:
        key=str(f.relative_to(ROOT)).replace("\\","/")
        if hashlib.sha256(f.read_bytes()).hexdigest()!=source_hashes[key]:
            raise RuntimeError(f"Input changed during simulation: {key}; rerun to preserve provenance")
    report=dict(status="Uncalibrated whole-hand dynamics; physical100ms NOT verified",engine="MuJoCo "+mujoco.__version__,
        python=platform.python_version(),source_hashes=source_hashes,
        boundary="Received target angle stream -> joint motion.2ms robot-internal transport plus1ms controller sampling assumptions included.",
        geometry="V9 roots/lengths and offset preserved; primitive display geometry. Collision contacts disabled; CAD sweep review separate.",
        parameters={k:v["parameters"] for k,v in provenance.items()},provenance=provenance,cases=results,sensitivities=sensitivities,
        limitations=["Motor PD/current-torque/PWM-to-torque model uncalibrated; manufacturer supply current is not calibrated phase current.",
          "Thumb MCP35degree frame models the requested kinematic axis; the candidate fork/bearing assembly does not yet implement that hinge geometry. These dynamics do not validate that assembly.",
          "Cable constant routing Jacobian is schematic; routing/contact/path deformation not solved.",
          "No thermal, supply voltage sag, contact/grasp, structural deformation or strength validation.",
          "remote16 is a virtual DIP=.8PIP coupling study retaining20motor CAD mass; no physical16motor CAD candidate or equivalent20axis expression claimed.",
          "Baseline part intrinsic inertias use bounding-box estimates; source COM/masses are illustrative V9 solid-density estimates."])
    write_json(a.output/"report.json",report)
    with (a.output/"summary.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f);w.writerow(["case","event_count","missing","median_ms","p95_ms","valid_max_ms","unsettled_holds","max_error_deg","simulated_event_100ms_met"])
        for key,r in results.items():w.writerow([key,r["event_count"],r["missing_events"],r["lag_median_ms"],r["lag_p95_ms"],r["valid_event_max_ms"],r["unsettled_holds"],r["raw_max_error_deg"],r["simulated_100ms_event_budget_met"]])
    plot_trace(plotting,a.output/"tracking_comparison.png")
    print(str(a.output/"report.json"),flush=True)


if __name__=="__main__":main()

"""Model consistency tests, not hardware acceptance tests."""
import json
import hashlib
import argparse
from pathlib import Path
import unittest
import numpy as np
import mujoco
from model import HERE, ROOT, CAD, NAMES, ALIASES, build_model, cable_forces, motor_torque, slew_pwm, run_simulation
from evaluate import scenario, evaluate, first_crossing, project_reference
from run_thumb_diagnostics import damping_rule

DETAILS={}


def synthetic_trace(time,command_deg,actual_deg,requested_deg=None):
    command=np.asarray(command_deg)
    actual=np.asarray(actual_deg)
    if command.ndim==1:command=np.repeat(command[:,None],20,axis=1)
    if actual.ndim==1:actual=np.repeat(actual[:,None],20,axis=1)
    zeros=np.zeros_like(command)
    return dict(time=time,command=np.deg2rad(command),q=np.deg2rad(actual),
        requested=np.deg2rad(command if requested_deg is None else requested_deg),
        v=np.deg2rad(np.gradient(actual,time,axis=0)),motor_speed=zeros,saturation=zeros.astype(bool),
        peak_tension=np.zeros(len(time)),slack=np.zeros(len(time),bool),
        joint_ranges=np.deg2rad(np.repeat([[-180,180]],20,axis=0)))


class ModelConsistency(unittest.TestCase):
    @staticmethod
    def diagnostic_step():
        t=np.arange(0,.200125,.000125)
        command=np.zeros((len(t),20))
        command[(t>=.02)&(t<.10),17]=np.deg2rad(30)
        return t,command

    def test_instrumentation_preserves_nominal_forces_exactly(self):
        system=build_model();t,command=self.diagnostic_step()
        plain=run_simulation(system,t,command)
        logged=run_simulation(system,t,command,record_diagnostics=True)
        fields=["q","v","motor_pos","motor_speed","torque","saturation","peak_tension","slack"]
        for field in fields:self.assertTrue(np.array_equal(plain[field],logged[field]),field)
        period=round(system["config"]["controller_period_s"]/system["config"]["dt_s"])
        between=np.arange(1,len(t))%period!=0
        for field in ["motor_target_rad","motor_velocity_target_rad_s","pd_request_Nm","pwm_request_raw","pwm_request_clipped"]:
            self.assertTrue(np.array_equal(logged["diagnostics"][field][1:][between],logged["diagnostics"][field][:-1][between]),field)
        DETAILS["nominal_instrumentation_exact_equality"]=dict(passed=True,fields=fields,
            controller_requests_held_between_updates=True,test_motion="30degree opposition step and return; other axes commanded hold")

    def test_diagnostic_clipping_and_power_identities(self):
        system=build_model();cfg=system["config"];t,command=self.diagnostic_step()
        trace=run_simulation(system,t,command,record_diagnostics=True);log=trace["diagnostics"]
        omega=trace["motor_speed"];w0=cfg["no_load_rpm"]*2*np.pi/60
        unlimited=cfg["stall_torque_Nm"]*(log["pwm_applied"]-omega/w0)
        cap=cfg["stall_torque_Nm"]*cfg["current_limit_A"]/cfg["stall_current_A"]
        self.assertTrue(np.array_equal(log["current_clipped"],np.abs(unlimited)>cap))
        self.assertTrue(np.array_equal(log["voltage_clipped"],np.abs(log["pwm_request_raw"])>1))
        self.assertTrue(np.array_equal(log["slew_limited"],np.abs(log["pwm_request_clipped"]-log["pwm_applied"])>1e-12))
        self.assertTrue(np.array_equal(trace["saturation"],(np.abs(unlimited)>=cap)|(np.abs(log["pwm_request_clipped"])>=.999)))
        self.assertTrue(np.array_equal(log["envelope_torque_current_limited_Nm"],np.clip(unlimited,-cap,cap)))
        self.assertTrue(np.allclose(log["pd_request_Nm"],log["pd_position_component_Nm"]+log["pd_velocity_component_Nm"],atol=1e-14))
        self.assertTrue(np.array_equal(trace["torque"],log["electrical_torque_applied_Nm"]-cfg["motor_viscous_Nm_s_per_rad"]*omega))
        self.assertTrue(np.array_equal(log["net_shaft_power_W"],trace["torque"]*omega))
        self.assertTrue(np.array_equal(log["braking"],log["net_shaft_power_W"] < -1e-12))
        self.assertTrue(np.allclose(log["electrical_mechanical_power_W"]-log["net_shaft_power_W"],cfg["motor_viscous_Nm_s_per_rad"]*omega**2,atol=1e-14))
        counts={key:int(np.sum(log[key][:,17])) for key in ["current_clipped","voltage_clipped","slew_limited","braking"]}
        self.assertTrue(all(count>0 for count in counts.values()))
        DETAILS["diagnostic_classification_step_counts"]=counts

    def test_ideal_diagnostics_are_explicit_and_axis_limited(self):
        system=build_model();cfg=system["config"];t,command=self.diagnostic_step()
        with self.assertRaises(ValueError):run_simulation(system,t,command,diagnostic={"actuator_mode":"ideal_pd_torque"})
        ideal=run_simulation(system,t,command,True,dict(axis=17,actuator_mode="ideal_pd_torque"))
        log=ideal["diagnostics"];others=np.arange(20)!=17
        self.assertTrue(np.array_equal(log["electrical_torque_applied_Nm"][:,17],log["pd_request_Nm"][:,17]))
        self.assertTrue(np.array_equal(log["electrical_torque_applied_Nm"][:,others],log["envelope_torque_current_limited_Nm"][:,others]))
        for field in ["applied_current_clipped","applied_voltage_clipped","applied_slew_limited"]:
            self.assertFalse(np.any(log[field][:,17]))
        self.assertFalse(ideal["diagnostic_options"]["nominal_force_path"])
        self.assertTrue(ideal["diagnostic_options"]["ideal_envelope_signals_counterfactual"])
        bypass=run_simulation(system,t,command,True,dict(axis=17,actuator_mode="no_pwm_slew"))
        log=bypass["diagnostics"]
        self.assertTrue(np.array_equal(log["pwm_applied"][:,17],log["pwm_request_clipped"][:,17]))
        self.assertFalse(np.any(log["applied_slew_limited"][:,17]))
        limit=cfg["pwm_slope_register"]*cfg["pwm_slope_mV_per_ms"]/cfg["voltage_V"]*cfg["dt_s"]
        self.assertLessEqual(np.max(np.abs(np.diff(log["pwm_applied"][:,others],axis=0))),limit+1e-12)
        self.assertGreater(np.max(np.abs(np.diff(log["pwm_applied"][:,17]))),limit)

    def test_predeclared_damping_rule_does_not_mutate_nominal_gain(self):
        system=build_model();nominal=system["config"]["kd_Nm_s_per_rad"]
        rule=damping_rule(system)
        self.assertEqual(len(rule["samples"]),10)
        maximum=max(x["locked_other_coordinates_inertia_kg_m2"] for x in rule["samples"])
        self.assertAlmostEqual(rule["kd_override_Nm_s_per_rad"],2*np.sqrt(system["config"]["kp_Nm_per_rad"]*maximum))
        self.assertTrue(all(x["free_other_coordinates_inertia_kg_m2"]>0 for x in rule["samples"]))
        self.assertEqual(system["config"]["kd_Nm_s_per_rad"],nominal)
        self.assertTrue(rule["declared_before_trajectory_runs"])
        DETAILS["diagnostic_damping_rule"]=rule

    def test_cad_mass_units_and_coordinates(self):
        s=build_model(); m,d=s["model"],s["data"]
        contract=json.loads((CAD/"simulation_contract.json").read_text())
        mujoco.mj_forward(m,d)
        max_pos=max_rot=0
        for row in contract["joints"]:
            name=next((k for k,v in ALIASES.items() if v==row["name"]),row["name"])
            body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,name)
            T=np.array(row["frame_world"])
            max_pos=max(max_pos,float(np.max(np.abs(d.xpos[body]-T[:3,3]/1000))))
            max_rot=max(max_rot,float(np.max(np.abs(d.xmat[body].reshape(3,3)-T[:3,:3]))))
        self.assertLess(max_pos,1e-10);self.assertLess(max_rot,1e-10)
        for name,p in s["mass_properties"].items():
            self.assertGreater(p["mass_kg"],0)
            I=np.array(p["inertia_com_kg_m2"])
            eig=np.linalg.eigvalsh(I)
            self.assertTrue(np.all(eig>0));self.assertLessEqual(eig[-1],sum(eig[:-1])+1e-12)
            body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,name)
            self.assertAlmostEqual(m.body_mass[body],p["mass_kg"])
        DETAILS["coordinate_max_position_error_m"]=max_pos
        DETAILS["coordinate_max_rotation_error"]=max_rot

    def test_transmission_virtual_work_and_energy(self):
        s=build_model();A=s["A"];rng=np.random.default_rng(7)
        velocity=rng.normal(size=20);force=rng.normal(size=20)
        self.assertAlmostEqual(float(force@(A@velocity)),float((A.T@force)@velocity),places=12)
        J=A.T@np.diag(np.full(20,2e-5))@A
        self.assertTrue(np.linalg.eigvalsh(J).min()>0)
        cfg=s["config"].copy();cfg.update(cable_mu=0,cable_damping_N_s_per_m=0,cable_backlash_rad=0)
        e=np.array([.002,-.002]);edot=np.array([.1,-.1]);qv=np.array([1.,-1.])
        shaft,out,plus,minus=cable_forces(e,edot,qv,cfg)
        self.assertTrue(np.allclose(shaft,out))
        potential_rate=2*cfg["cable_stiffness_N_per_m"]*cfg["motor_spool_radius_m"]**2*(e@edot)
        self.assertAlmostEqual(float(shaft@edot),potential_rate)
        cfg["cable_mu"]=.1
        shaft,out,_,_=cable_forces(e,edot,qv,cfg)
        self.assertGreaterEqual(float((shaft-out)@qv),0)

    def test_gravity_potential_gradient(self):
        s=build_model();m,d=s["model"],s["data"]
        d.qpos[:20]=np.deg2rad([0,20,25,15]*4+[10,20,25,15]);mujoco.mj_forward(m,d)
        force=d.qfrc_bias.copy();errors=[]
        for i in range(20):
            q=d.qpos[i];eps=1e-6
            d.qpos[i]=q+eps;mujoco.mj_forward(m,d);mujoco.mj_energyPos(m,d);plus=d.energy[0]
            d.qpos[i]=q-eps;mujoco.mj_forward(m,d);mujoco.mj_energyPos(m,d);minus=d.energy[0]
            d.qpos[i]=q
            errors.append(abs((plus-minus)/(2*eps)-force[i]))
        self.assertLess(max(errors),1e-7)
        DETAILS["gravity_gradient_max_error_Nm"]=max(errors)

    def test_pwm_and_reverse_envelope(self):
        cfg=build_model()["config"];p=0.;count=0
        while p<1-1e-12:
            p=float(slew_pwm(p,1,cfg));count+=1
        measured=count*cfg["dt_s"]
        expected=cfg["voltage_V"]/(cfg["pwm_slope_register"]*cfg["pwm_slope_mV_per_ms"])
        self.assertLessEqual(abs(measured-expected),cfg["dt_s"])
        p=np.array([.2,.7]);v=np.array([1.,10.])
        tp,_=motor_torque(p,v,cfg);tn,_=motor_torque(-p,-v,cfg)
        self.assertTrue(np.allclose(tp,-tn));DETAILS["zero_to_full_PWM_s"]=measured

    def test_missing_event_is_failure_and_reset(self):
        s=build_model();t,c,moves=scenario("small5",s["config"]["dt_s"])
        tr=run_simulation(s,t,c);again=run_simulation(s,t,c)
        self.assertTrue(np.array_equal(tr["q"],again["q"]))
        tr["q"][:]=0
        metrics=evaluate(tr,moves,s["active"])
        self.assertGreater(metrics["missing_events"],0)
        self.assertFalse(metrics["simulated_100ms_event_budget_met"])
        self.assertIsNone(metrics["maximum_including_missing_ms"])
        self.assertIsNone(first_crossing(np.array([0,1]),np.array([2,3]),1,1))

    def test_time_step_convergence(self):
        values={}
        for dt in [.000125,.0000625]:
            s=build_model(overrides={"dt_s":dt})
            t,c,moves=scenario("rapid_reversal",dt)
            values[dt]=run_simulation(s,t,c)
        q1=values[.000125]["q"];q2=values[.0000625]["q"][::2]
        error=np.rad2deg(q1-q2)
        rms=float(np.sqrt(np.mean(error**2)));maximum=float(np.max(np.abs(error)))
        DETAILS["time_step_125_vs62_5us_rmse_deg"]=rms
        DETAILS["time_step_125_vs62_5us_max_deg"]=maximum
        self.assertLess(rms,1.0)
        self.assertLess(maximum,5.0)

    def test_small_motion_time_step_and_quality_classification_stability(self):
        # Declared before execution: numerical error <=20% of the0.1degree minimum band.
        threshold=.02
        values={};qualities={}
        for dt in [.000125,.0000625]:
            s=build_model(overrides={"dt_s":dt})
            t,c,moves=scenario("small5",dt)
            values[dt]=run_simulation(s,t,c)
            qualities[dt]=evaluate(values[dt],moves,s["active"])["hold_quality"]
        error=np.rad2deg(values[.000125]["q"]-values[.0000625]["q"][::2])
        maximum=float(np.max(np.abs(error)))
        first={(r["move"],r["joint"]):r["status"] for r in qualities[.000125]}
        second={(r["move"],r["joint"]):r["status"] for r in qualities[.0000625]}
        changed=[dict(move=key[0],joint=key[1],status_125us=first[key],status_62_5us=second[key])
                 for key in first if first[key]!=second[key]]
        DETAILS["small5_time_step_quality"]=dict(coarse_dt_s=.000125,fine_dt_s=.0000625,
            declared_max_difference_limit_deg=threshold,full_trajectory_max_difference_deg=maximum,
            hold_quality_rows=len(first),hold_quality_classifications_stable=not changed,
            changed_hold_classifications=changed,
            criterion="Both maximum angle difference<=0.02deg and all hold-quality statuses identical; fixed before run")
        self.assertLessEqual(maximum,threshold)
        self.assertEqual(first,second)

    def test_late_rebound_is_not_early_settling(self):
        t=np.arange(0,.801,.001);x=np.minimum(t/.1,1)*10
        q=np.repeat(x[:,None],20,axis=1)
        q[(t>=.3)&(t<.5)]=15
        zeros=np.zeros_like(q)
        trace=dict(time=t,command=np.repeat(x[:,None],20,axis=1)*np.pi/180,
            q=q*np.pi/180,v=zeros,motor_speed=zeros,saturation=zeros.astype(bool),
            peak_tension=np.zeros(len(t)),slack=np.zeros(len(t),bool),
            joint_ranges=np.repeat([[-1,1]],20,axis=0))
        move=dict(start=0,end=.1,hold_end=.8,q0=np.zeros(20),q1=np.full(20,10))
        r=evaluate(trace,[move],list(range(20)))
        self.assertTrue(all(x["settle_after_command_end_ms"]>=399 for x in r["settling"]))

    def test_known_delay_and_preexisting_recross_classification(self):
        t=np.arange(0,1.001,.001)
        c=np.interp(t,[0,.1,.3,.5,.7,1],[0,0,10,10,0,0])
        q=np.interp(t-.04,t,c)
        moves=[dict(start=.1,end=.3,hold_end=.5,q0=np.zeros(20),q1=np.full(20,10)),
               dict(start=.5,end=.7,hold_end=1.,q0=np.full(20,10),q1=np.zeros(20))]
        r=evaluate(synthetic_trace(t,c,q),moves,list(range(20)))
        self.assertEqual(r["missing_events"],0)
        self.assertTrue(all(abs(e["lag_ms"]-40)<1e-9 for e in r["events"]))
        self.assertEqual(set(e["direction"] for e in r["events"]),{-1,1})
        # A preexisting level can recross later, but cannot become a measured first response.
        t=np.arange(0,.601,.001);c=np.minimum(t/.4,1)*10
        q=np.interp(t,[0,.1,.2,.6],[6,4,6,6])
        move=dict(start=0,end=.4,hold_end=.6,q0=np.zeros(20),q1=np.full(20,10))
        r=evaluate(synthetic_trace(t,c,q),[move],list(range(20)))
        event=next(e for e in r["events"] if e["fraction"]==.5)
        self.assertEqual(event["classification"],"preexisting_level")
        self.assertAlmostEqual(event["later_directional_crossing_s"],.15)
        self.assertIsNone(event["lag_ms"]);self.assertFalse(event["timing_identifiable"])
        self.assertEqual(r["event_classification_counts"]["never_reached"],20)
        self.assertEqual(r["indeterminate_events"],r["missing_events"])
        self.assertFalse(r["simulated_100ms_event_budget_met"])

    def test_small_motion_requires_amplitude_scaled_hold_accuracy(self):
        t=np.arange(0,.601,.001);c=np.minimum(t/.1,1)*.6
        move=dict(start=0,end=.1,hold_end=.6,q0=np.zeros(20),q1=np.full(20,.6))
        r=evaluate(synthetic_trace(t,c,np.zeros_like(t)),[move],list(range(20)))
        self.assertEqual(r["unsettled_holds"],0)  # Old3degree criterion was too broad for this task.
        self.assertEqual(r["unsettled_quality_holds"],20)
        self.assertTrue(all(x["tolerance_deg"]==.1 for x in r["hold_quality"]))
        self.assertEqual(r["event_classification_counts"]["never_reached"],60)

    def test_independent_distal_projection_preserves_requested_task(self):
        t,requested,moves=scenario("independent_distal",.001)
        saved=requested.copy();saved_moves=[m["q1"].copy() for m in moves]
        feasible,projected=project_reference(requested,moves,True)
        self.assertTrue(np.array_equal(requested,saved))
        self.assertTrue(all(np.array_equal(m["q1"],old) for m,old in zip(moves,saved_moves)))
        self.assertAlmostEqual(float(np.max(np.abs(np.rad2deg(requested-feasible)))),60.)
        self.assertTrue(np.allclose(feasible[:,[3,7,11,15]],.8*feasible[:,[2,6,10,14]]))
        self.assertFalse(np.allclose(requested[:,3],.8*requested[:,2]))
        trace=synthetic_trace(t,np.rad2deg(feasible),np.rad2deg(feasible),np.rad2deg(requested))
        r=evaluate(trace,projected,list(range(20)))
        self.assertAlmostEqual(r["raw_max_error_deg"],0)
        self.assertAlmostEqual(r["projection_loss_max_deg"],60)
        self.assertAlmostEqual(r["task_raw_max_error_deg"],60)
        self.assertTrue(r["simulated_100ms_event_budget_met"])  # Event-only pass cannot imply task fidelity.

    def test_endpoint_overshoot_and_reversal_start_excursion_are_separate(self):
        t=np.arange(0,1.001,.001)
        c=np.interp(t,[0,.2,.5,.7,1],[0,45,45,0,0])
        q=np.interp(t,[0,.2,.5,.55,.7,1],[0,45,45,50,0,0])
        moves=[dict(start=0,end=.2,hold_end=.5,q0=np.zeros(20),q1=np.full(20,45)),
               dict(start=.5,end=.7,hold_end=1.,q0=np.full(20,45),q1=np.zeros(20))]
        r=evaluate(synthetic_trace(t,c,q),moves,list(range(20)))
        self.assertAlmostEqual(r["max_wrong_way_excursion_deg"],5)
        self.assertAlmostEqual(r["soft_joint_limit_max_overshoot_deg"],0)
        self.assertAlmostEqual(r["max_endpoint_overshoot_deg"],0)
        q=np.interp(t,[0,.2,.3,.5,.7,1],[0,45,50,45,0,0])
        r=evaluate(synthetic_trace(t,c,q),moves,list(range(20)))
        self.assertAlmostEqual(r["max_endpoint_overshoot_deg"],5)
        self.assertAlmostEqual(r["soft_joint_limit_max_overshoot_deg"],0)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,default=HERE/"results")
    args=parser.parse_args()
    paths=[HERE/name for name in ["model.py","evaluate.py","test_model.py","run_suite.py","run_thumb_diagnostics.py"]]
    paths += [ROOT/name for name in ["cad/remote_drive_v1/design.json","cad/remote_drive_v1/simulation_contract.json",
        "cad/modular_roots_v9/design.json","cad/modular_roots_v9/mass_screen.json","cad/modular_roots_v9/scene.json",
        "cad/modular_roots_v9/cad_manifest.json","cad/modular_roots_v9/kinematics_numpy.py"]]
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(ModelConsistency)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    changed=[p.relative_to(ROOT).as_posix() for p in paths if hashlib.sha256(p.read_bytes()).hexdigest()!=hashes[p.relative_to(ROOT).as_posix()]]
    passed=result.wasSuccessful() and not changed
    out=args.output;out.mkdir(parents=True,exist_ok=True)
    (out/"model_consistency.json").write_text(json.dumps(dict(passed=passed,tests=result.testsRun,
        source_hashes={str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                       [HERE/"model.py",HERE/"evaluate.py",HERE/"test_model.py",CAD/"simulation_contract.json"]},
        all_input_hashes=hashes,input_hash_guard=dict(passed=not changed,changed_inputs=changed),
        details=DETAILS,failures=[str(x[1]) for x in result.failures],errors=[str(x[1]) for x in result.errors]),indent=2))
    raise SystemExit(0 if passed else 1)

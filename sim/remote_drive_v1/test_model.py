"""Model consistency tests, not hardware acceptance tests."""
import json
import hashlib
import argparse
from pathlib import Path
import unittest
import numpy as np
import mujoco
from model import HERE, CAD, NAMES, ALIASES, build_model, cable_forces, motor_torque, slew_pwm, run_simulation
from evaluate import scenario, evaluate, first_crossing

DETAILS={}


class ModelConsistency(unittest.TestCase):
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


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,default=HERE/"results")
    args=parser.parse_args()
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(ModelConsistency)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    out=args.output;out.mkdir(parents=True,exist_ok=True)
    (out/"model_consistency.json").write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
        source_hashes={str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                       [HERE/"model.py",HERE/"evaluate.py",HERE/"test_model.py",CAD/"simulation_contract.json"]},
        details=DETAILS,failures=[str(x[1]) for x in result.failures],errors=[str(x[1]) for x in result.errors]),indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)

"""Bounded command/metric protocol tests; no force integration or CAD generation."""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import unittest

import numpy as np

import model_adapter as adapter
import run_screen as screen


class ScreenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design=json.loads((adapter.ROOT/"cad/webcam_articulation_v1/design.json").read_text(encoding="utf-8"))
        cls.legacy=adapter.load_legacy(adapter.ROOT/"sim/form_drive_v1/reference_06b9ab4")

    def test_parameters_unmodified_and_transmission_explicit(self):
        data=json.loads((screen.HERE/"parameters.json").read_text(encoding="utf-8"))
        src=adapter.ROOT/data["source_provenance"]["path"]
        self.assertEqual(adapter.digest(src),data["source_provenance"]["sha256"])
        self.assertEqual(data["parameters"],json.loads(src.read_text(encoding="utf-8"))["parameters"])
        self.assertEqual(data["parameters"]["received_to_servo_s"],.002)
        tr=json.loads((screen.HERE/"transmission.json").read_text(encoding="utf-8"))
        np.testing.assert_array_equal(tr["motor_angle_jacobian"],np.eye(20))
        self.assertEqual(tr["cable_actuator_indices"],list(range(20)))
        self.assertEqual(tr["motor_spool_radius_m"],data["parameters"]["motor_spool_radius_m"])
        self.assertEqual(tr["status"],"assumed_not_validated")

    def test_fist_preserves_every_waypoint_and_reverses_timing(self):
        for dt in screen.DT:
            t,q,m,g=screen.scenario(self.legacy,self.design,"cad_fist",dt)
            screen.check_command_ranges(self.design,q)
            self.assertEqual(len(m),6)
            self.assertAlmostEqual(t[-1],1.95)
            self.assertEqual([(x["waypoint_from"],x["waypoint_to"]) for x in m],[(0,1),(1,2),(2,3),(3,2),(2,1),(1,0)])
            for x in m:
                for key in ("start","end"):
                    ix=round(x[key]/dt)
                    np.testing.assert_allclose(np.rad2deg(q[ix]),x["q0" if key=="start" else "q1"],rtol=0,atol=2e-10)
                ix=round((x["start"]+x["end"])/(2*dt))
                np.testing.assert_allclose(np.rad2deg(q[ix]),(x["q0"]+x["q1"])/2,rtol=0,atol=1e-9)
            self.assertAlmostEqual(m[2]["hold_end"]-m[2]["end"],.3)
            self.assertAlmostEqual(m[5]["hold_end"]-m[5]["end"],.3)
            self.assertEqual(g["timing"],screen.FIST_TIMING)

    def test_bad_waypoint_or_range_cannot_silently_clip(self):
        bad=copy.deepcopy(self.design);bad["fist_path"][1]["fraction"]=.8
        with self.assertRaisesRegex(ValueError,"ordered CAD"):
            screen.scenario(self.legacy,bad,"cad_fist",screen.DT[0])
        bad=copy.deepcopy(self.design);bad["fist_path"][1]["angles_deg"][6]=140
        _,q,_,_=screen.scenario(self.legacy,bad,"cad_fist",screen.DT[0])
        with self.assertRaisesRegex(ValueError,"exceeds new CAD ranges"):
            screen.check_command_ranges(bad,q)

    def test_small5_identical_and_fist_ideal_metric_coverage(self):
        dt=screen.DT[0]
        t,q,m,_=screen.scenario(self.legacy,self.design,"small5",dt)
        t0,q0,m0=adapter.scenario(self.legacy,"small5",dt)
        np.testing.assert_array_equal(t,t0);np.testing.assert_array_equal(q,q0)
        self.assertEqual(screen.jsonable(m),screen.jsonable(m0))
        t,q,m,_=screen.scenario(self.legacy,self.design,"cad_fist",dt)
        v=np.gradient(q,dt,axis=0)
        limits=np.deg2rad([j["range_deg"] for j in adapter.kinematic_blueprint(self.design,[])["joints"]])
        trace=dict(time=t,command=q,q=q,v=v,saturation=np.zeros_like(q,dtype=bool),motor_speed=v,
            joint_ranges=limits,peak_tension=np.zeros(len(t)),slack=np.zeros(len(t),dtype=bool))
        metrics=self.legacy["evaluate"].evaluate(trace,m,list(range(20)))
        self.assertGreater(metrics["event_count"],0)
        self.assertEqual(metrics["missing_events"],0)
        self.assertEqual(metrics["anticipatory_events"],0)
        self.assertEqual(metrics["raw_max_error_deg"],0)
        self.assertEqual(len(metrics["hold_quality"]),40)
        # No actual response is a failure, even though some held axes match.
        trace["q"]=np.zeros_like(q);trace["v"]=np.zeros_like(q)
        failed=self.legacy["evaluate"].evaluate(trace,m,list(range(20)))
        self.assertFalse(failed["simulated_100ms_event_budget_met"])
        self.assertGreater(failed["missing_events"],0)
        self.assertGreater(failed["unsettled_quality_holds"],0)

    def test_convergence_rejects_missing_pairs_bias_and_changed_classes(self):
        self.assertFalse(screen.compare_resolution(None,None)["passed"])
        event=dict(move=0,joint="index_mcp",fraction=.5,classification="reached",lag_ms=20.)
        hold=dict(move=0,joint="index_mcp",status="settled")
        r=dict(finite=True,scenario="small5",metrics=dict(events=[event],hold_quality=[hold]))
        t=np.arange(3)*.000125;q=np.zeros((3,20))
        a=(r,t,q);b=(copy.deepcopy(r),t,q.copy())
        self.assertTrue(screen.compare_resolution(a,b)["passed"])
        b[2][:,0]=np.deg2rad(.021)
        self.assertFalse(screen.compare_resolution(a,b)["passed"])
        b[2][:]=0;b[0]["metrics"]["events"][0].update(classification="preexisting_level",lag_ms=None)
        c=screen.compare_resolution(a,b)
        self.assertFalse(c["passed"]);self.assertFalse(c["lag_availability_identical"])
        b=(copy.deepcopy(r),t,q.copy());b[0]["metrics"]["events"].append(event.copy())
        self.assertFalse(screen.compare_resolution(a,b)["event_identity_identical"])
        b=(copy.deepcopy(r),t,q.copy());b[0]["metrics"]["hold_quality"][0]["status"]="unsettled"
        self.assertFalse(screen.compare_resolution(a,b)["hold_quality_classifications_identical"])


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--output",type=Path,required=True);a=p.parse_args()
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ScreenTests))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    paths=[Path(__file__),screen.HERE/"run_screen.py",screen.HERE/"model_adapter.py",screen.HERE/"parameters.json",screen.HERE/"transmission.json"]
    screen.write_json(a.output,dict(tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        passed=result.wasSuccessful(),scope="No physics integration; command/metric protocol only",
        source_sha256={p.relative_to(adapter.ROOT).as_posix():adapter.digest(p) for p in paths},
        details=[str(test)+": "+message for test,message in result.failures+result.errors]))
    raise SystemExit(0 if result.wasSuccessful() else 1)

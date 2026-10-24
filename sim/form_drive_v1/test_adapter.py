"""Bounded adapter tests. Synthetic inertias test code, never hand performance.

Run through the workspace hidden launcher with --reference-root <pinned tree>.
No CAD generation or full trajectory suite is performed.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import model_adapter as adapter

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = None
ACTUAL_CONTRACT = None


def fixture():
    design = json.loads((ROOT / "cad/form_candidate_v1/design.json").read_text(encoding="utf-8"))
    scene = json.loads((ROOT / "cad/form_candidate_v1/generated/scene.json").read_text(encoding="utf-8"))
    blue = adapter.kinematic_blueprint(design, [r["name"] for r in scene["hand"]])
    for b in blue["bodies"]:
        b.update(mass_kg=.02, com_local_mm=[1, 12, -2],
                 inertia_com_local_kg_m2=[[3e-6, .1e-6, 0], [.1e-6, 2e-6, .05e-6], [0, .05e-6, 2.5e-6]])
    for j in blue["joints"]:
        j["range_deg"] = [-120, 120]
    return dict(schema_version=adapter.SCHEMA_VERSION, design=design, bodies=blue["bodies"],
                joints=blue["joints"], source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                    for name in sorted(adapter._REQUIRED_INPUTS)},
                mass_scope="SYNTHETIC UNIT-TEST INERTIAS, NOT A CAD MASS OR PERFORMANCE RESULT",
                limitations=["Synthetic input exists only to test adapter plumbing"],
                transmission=dict(status="assumed_not_validated", assumptions=["Synthetic identity routing"],
                    motor_angle_jacobian=np.eye(20).tolist(), cable_actuator_indices=list(range(20))))


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = fixture()
        cls.legacy = adapter.load_legacy(REFERENCE)

    def setUp(self):
        self.c = copy.deepcopy(self.original)

    def test_blueprint_origin_and_thumb_yaw(self):
        bodies = {r["name"]:r for r in self.c["bodies"]}
        self.assertTrue(np.allclose(np.array(bodies["middle_spread"]["frame_world_mm"])[:3,3], [19.5,127,.5]))
        self.assertTrue(np.allclose(np.array(bodies["middle_mcp"]["frame_world_mm"])[:3,3], [19.5,127,8]))
        a = np.deg2rad(-40)
        expected = [46-np.sin(a)*50, 34+np.cos(a)*50, 8]
        T = np.array(bodies["thumb_mcp"]["frame_world_mm"])
        self.assertTrue(np.allclose(T[:3,3], expected, rtol=0, atol=1e-10))
        self.assertAlmostEqual(np.rad2deg(np.arctan2(T[1,0], T[0,0])), -15)

    def test_parent_pin_grouping_and_no_missing_parts(self):
        adapter.validate_contract(self.c, repo_root=ROOT)
        for finger in (*adapter.DIGITS, "thumb"):
            names = adapter._names_for(finger)
            for j in range(3):
                self.assertEqual(adapter.component_body(finger+f"_joint_pin_{j}"), names[j])
        self.assertEqual(sum(len(b["component_names"]) for b in self.c["bodies"]), 120)

    def test_mass_and_inertia_fail_closed(self):
        del self.c["bodies"][2]["mass_kg"]
        with self.assertRaises(KeyError):
            adapter.validate_contract(self.c)
        self.c = copy.deepcopy(self.original)
        self.c["bodies"][2]["inertia_com_local_kg_m2"] = np.diag([1e-6, 1e-6, 9e-6]).tolist()
        with self.assertRaisesRegex(ValueError, "triangle"):
            adapter.validate_contract(self.c)

    def test_old_root_frame_and_axis_rejected(self):
        self.c["joints"][1]["frame_parent_mm"][2][3] = 32
        with self.assertRaisesRegex(ValueError, "axis convention"):
            adapter.validate_contract(self.c)
        self.c = copy.deepcopy(self.original)
        self.c["joints"][0]["axis_local"] = [0,0,-1]
        with self.assertRaisesRegex(ValueError, "axis convention"):
            adapter.validate_contract(self.c)

    def test_duplicate_and_missing_parts_rejected(self):
        self.c["bodies"][2]["component_names"].append(self.c["bodies"][1]["component_names"][0])
        with self.assertRaisesRegex(ValueError, "twice"):
            adapter.validate_contract(self.c)
        self.c = copy.deepcopy(self.original)
        self.c["bodies"][2]["component_names"].pop()
        with self.assertRaisesRegex(ValueError, "exactly once"):
            adapter.validate_contract(self.c, repo_root=ROOT)

    def test_declared_extra_fixed_mass_components(self):
        name = "concept_motor_0"
        self.c["additional_components"] = [dict(name=name, body="palm", scope="Fixed-base synthetic test component")]
        self.c["bodies"][0]["component_names"].append(name)
        adapter.validate_contract(self.c, repo_root=ROOT)
        self.c["additional_components"][0]["body"] = "middle_mcp"
        with self.assertRaisesRegex(ValueError, "grouping"):
            adapter.validate_contract(self.c)

    def test_routing_independence_and_direct_contract(self):
        # All 20 root/distal axes may be remote. Cable rows retain their declared map.
        A = np.eye(20)
        A[19,16:19] = [.2,.3,.4]
        self.c["transmission"]["motor_angle_jacobian"] = A.tolist()
        adapter.validate_contract(self.c)
        self.c["transmission"]["cable_actuator_indices"].remove(19)
        with self.assertRaisesRegex(ValueError, "direct actuator"):
            adapter.validate_contract(self.c)
        self.c["transmission"]["cable_actuator_indices"] = list(range(20))
        A[19,:] = 0
        self.c["transmission"]["motor_angle_jacobian"] = A.tolist()
        with self.assertRaisesRegex(ValueError, "invertible"):
            adapter.validate_contract(self.c)

    def test_cable_map_reciprocity_is_not_old_thumb_mapping(self):
        A = np.eye(20)
        A[18,16:18] = [.3,.3]
        A[19,16:19] = [.3,.3,.3]
        self.c["transmission"]["motor_angle_jacobian"] = A.tolist()
        adapter.validate_contract(self.c)
        qd = np.linspace(-1, 2, 20)
        torque = np.linspace(.01, .04, 20)
        self.assertAlmostEqual(float(torque @ (A @ qd)), float((A.T @ torque) @ qd), places=14)
        A[18,1] = .1
        self.c["transmission"]["motor_angle_jacobian"] = A.tolist()
        with self.assertRaisesRegex(ValueError, "ancestors"):
            adapter.validate_contract(self.c)

    def test_source_and_explicit_assumption_guards(self):
        self.c["source_sha256"]["cad/form_candidate_v1/design.json"] = "0"*64
        with self.assertRaisesRegex(ValueError, "Stale"):
            adapter.validate_contract(self.c, repo_root=ROOT)
        with self.assertRaisesRegex(ValueError, "allow_conceptual"):
            adapter.build_system("not-read.json", REFERENCE, parameters=self.legacy["model"].DEFAULTS)
        with self.assertRaisesRegex(ValueError, "every explicit"):
            adapter._parameters({}, self.legacy)

    def test_all_seven_tasks_keep_physical_spread_direction(self):
        for name in ["small5", "small10", "large45", "fast90_150ms", "upstream_hold", "rapid_reversal", "independent_distal"]:
            t, q, moves = adapter.scenario(self.legacy, name, .000125)
            ot, oq, om = self.legacy["evaluate"].scenario(name, .000125)
            self.assertTrue(np.array_equal(t, ot))
            self.assertTrue(np.array_equal(q[:,adapter.SPREAD_INDICES], -oq[:,adapter.SPREAD_INDICES]))
            other = [i for i in range(20) if i not in adapter.SPREAD_INDICES]
            self.assertTrue(np.array_equal(q[:,other], oq[:,other]))
            for m, old in zip(moves, om):
                self.assertTrue(np.array_equal(m["q1"][adapter.SPREAD_INDICES], -old["q1"][adapter.SPREAD_INDICES]))

    def test_compiled_si_inertia_and_non_neutral_frames(self):
        with tempfile.TemporaryDirectory(prefix="form_adapter_test_") as td:
            path = Path(td)/"synthetic_contract.json"
            path.write_text(json.dumps(self.c), encoding="utf-8")
            system = adapter.build_system(path, REFERENCE, parameters=self.legacy["model"].DEFAULTS,
                                          allow_conceptual=True, repo_root=ROOT)
        m, d, old = system["model"], system["data"], self.legacy["model"]
        self.assertEqual(m.nq, 40)
        self.assertTrue(np.array_equal(m.dof_armature[:20], np.zeros(20)))
        for sample in [np.zeros(20), np.linspace(-.2,.3,20)]:
            old.mujoco.mj_resetData(m,d)
            d.qpos[:20] = sample
            old.mujoco.mj_forward(m,d)
            world = {"palm":np.eye(4)}
            for i, j in enumerate(self.c["joints"]):
                T = np.array(j["frame_parent_mm"]); T[:3,3] /= 1000
                axis = np.array(j["axis_local"],dtype=float)
                x,y,z = axis
                K = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                turn = np.eye(4)
                turn[:3,:3] = np.eye(3)+np.sin(sample[i])*K+(1-np.cos(sample[i]))*(K@K)
                world[j["name"]] = world[j["parent"]] @ T @ turn
                b = m.body(j["name"]).id
                self.assertTrue(np.allclose(d.xpos[b],world[j["name"]][:3,3],rtol=0,atol=1e-10))
                self.assertTrue(np.allclose(d.xmat[b].reshape(3,3),world[j["name"]][:3,:3],rtol=0,atol=1e-10))
                self.assertTrue(np.allclose(m.body_ipos[b], [.001,.012,-.002],rtol=0,atol=1e-12))
                R = np.zeros(9); old.mujoco.mju_quat2Mat(R,m.body_iquat[b]); R=R.reshape(3,3)
                expected=np.array(self.c["bodies"][i+1]["inertia_com_local_kg_m2"])
                self.assertTrue(np.allclose(R@np.diag(m.body_inertia[b])@R.T,expected,rtol=1e-8,atol=1e-13))

    def test_force_reuse_and_command_guard(self):
        with tempfile.TemporaryDirectory(prefix="form_adapter_force_") as td:
            path = Path(td)/"synthetic_contract.json"
            path.write_text(json.dumps(self.c),encoding="utf-8")
            system=adapter.build_system(path,REFERENCE,parameters=self.legacy["model"].DEFAULTS,
                                        allow_conceptual=True,repo_root=ROOT)
        t=np.arange(0,.001001,.000125);q=np.zeros((len(t),20))
        a=adapter.run_simulation(system,t,q)
        b=self.legacy["model"].run_simulation(system,t,q)
        for name in ["q","v","torque","saturation","peak_tension"]:
            self.assertTrue(np.array_equal(a[name],b[name]))
        q[:,0]=np.deg2rad(121)
        with self.assertRaisesRegex(ValueError,"range"):
            adapter.run_simulation(system,t,q)

    def test_actual_contract_when_supplied(self):
        if ACTUAL_CONTRACT is None:
            self.skipTest("Actual BRep contract not supplied; no real-mass result inferred")
        c=json.loads(Path(ACTUAL_CONTRACT).read_text(encoding="utf-8"))
        adapter.validate_contract(c,repo_root=ROOT)
        system=adapter.build_system(ACTUAL_CONTRACT,REFERENCE,parameters=self.legacy["model"].DEFAULTS,
                                    allow_conceptual=True,repo_root=ROOT)
        self.assertEqual(system["model"].nq,20+len(c["transmission"]["cable_actuator_indices"]))


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-root",required=True)
    parser.add_argument("--contract")
    parser.add_argument("--output",required=True)
    args=parser.parse_args()
    REFERENCE=Path(args.reference_root)
    ACTUAL_CONTRACT=args.contract
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(AdapterTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report=dict(scope="Adapter consistency only; no physical or trajectory performance assertion",
        tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        skipped=len(result.skipped),passed=result.wasSuccessful(),reference_sha=adapter.REFERENCE_SHA,
        legacy_source_sha256=adapter.REFERENCE_FILES,
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(adapter.__file__)]},
        actual_contract_sha256=hashlib.sha256(Path(args.contract).read_bytes()).hexdigest() if args.contract else None,
        details=[str(test)+": "+message for test,message in result.failures+result.errors])
    Path(args.output).write_text(json.dumps(report,indent=2),encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)

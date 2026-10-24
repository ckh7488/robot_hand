"""Small coordinate/contract tests; no renderer, camera, CAD rebuild or long run.

The webcam CAD is an independent kinematic reference, not a performance model.
Synthetic mass fixtures test arithmetic only. --contract adds real new-CAD mass
and compiled-model checks after the geometry source is frozen.
"""
from __future__ import annotations

import argparse
import copy
import importlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest

import numpy as np

import model_adapter as adapter

ROOT=Path(__file__).resolve().parents[2]
REFERENCE=ROOT/"sim/form_drive_v1/reference_06b9ab4"
CONTRACT=None
EVIDENCE={}
WEBCAM_REFERENCE=Path(__file__).resolve().parent/"reference_webcam_31ba43f"
WEBCAM_SHA="31ba43fd4ab6393945a612b46c7b279de11a5267"
WEBCAM_FILES={
    "hand_follow/cad_hand.py":"38a512c236a46e3480c59f22719bc4caeb92dd4ffa2cef06caa0b71ee3bc07b9",
    "hand_follow/retarget.py":"f52a6872497a5764444283ebf8257ce890f84c90282081ca4c868582bc57fbfc",
    "hand_follow/__init__.py":"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"}


def load_webcam_reference():
    provenance=json.loads((WEBCAM_REFERENCE/"provenance.json").read_text(encoding="utf-8"))
    if provenance["reference_commit"]!=WEBCAM_SHA or {
            r["snapshot_path"]:r["sha256"] for r in provenance["files"]}!=WEBCAM_FILES:
        raise ValueError("Webcam reference provenance differs from the pinned commit")
    for rel,expected in WEBCAM_FILES.items():
        if adapter.digest(WEBCAM_REFERENCE/rel)!=expected:
            raise ValueError("Webcam reference bytes changed: "+rel)
    # A unique package avoids any imported active hand_follow application.
    name="_articulation_pinned_webcam_31ba43f"
    package=WEBCAM_REFERENCE/"hand_follow"
    spec=importlib.util.spec_from_file_location(name,package/"__init__.py",submodule_search_locations=[str(package)])
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    cad=importlib.import_module(name+".cad_hand")
    if Path(cad.__file__).resolve()!=package/"cad_hand.py":
        raise ValueError("Wrong webcam kinematic module imported")
    return cad.CadHand


def synthetic_parts(design):
    """One explicitly named synthetic component per body, not CAD mass estimates."""
    bp=adapter.kinematic_blueprint(design,[])
    parts=[];mass=[]
    for i,b in enumerate(bp["bodies"]):
        name="synthetic_component_"+str(i)
        T=np.array(b["frame_world_mm"])
        parts.append(dict(name=name,body=b["name"],volume_mm3=10000.))
        mass.append(dict(name=name,mass_kg=.01,volume_mm3=10000.,density_kg_per_mm3=1e-6,
            com_world_mm=T[:3,3].tolist(),inertia_com_world_kg_m2=np.diag([1e-6,1.2e-6,1.5e-6]).tolist()))
    return parts,mass


def synthetic_contract(design):
    parts,mass=synthetic_parts(design)
    bp=adapter.aggregate_mass(design,parts,mass)
    return dict(schema_version=adapter.SCHEMA_VERSION,
        units={"length":"mm","mass":"kg","inertia":"kg*m^2","joint_angle":"deg"},
        design=design,part_metadata=parts,bodies=bp["bodies"],joints=bp["joints"],
        mass_scope="SYNTHETIC UNIT TEST ONLY; no hand mass or latency result",
        limitations=["Synthetic components used only to test validation"],
        transmission=dict(status="assumed_not_validated",assumptions=["Synthetic identity map"],
            motor_angle_jacobian=np.eye(20).tolist(),cable_actuator_indices=list(range(20))))


def poses(design):
    result=[("neutral",np.zeros(20))]
    result += [("fist_path_"+str(i),np.array(p["angles_deg"],dtype=float)) for i,p in enumerate(design["fist_path"])]
    limits=np.array([j["range_deg"] for j in adapter.kinematic_blueprint(design,[])["joints"]])
    for i,name in enumerate(adapter.NAMES):
        q=np.zeros(20);q[i]=min(35.,limits[i,1]*.5)
        result.append((name+"_only",q))
    rng=np.random.default_rng(20261025)
    for i in range(3):
        result.append(("mixed_"+str(i),limits[:,0]+rng.uniform(.2,.7,20)*(limits[:,1]-limits[:,0])))
    return result


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot=load_webcam_reference()(ROOT,cad="webcam_articulation_v1")
        cls.design=copy.deepcopy(cls.robot.design)
        cls.legacy=adapter.load_legacy(REFERENCE)

    @classmethod
    def tearDownClass(cls):
        cls.robot.close()

    def test_explicit_component_body_not_name_inference(self):
        parts=[dict(name="entirely_new_root_keeper",body="middle_spread")]
        bp=adapter.kinematic_blueprint(self.design,parts)
        b=next(b for b in bp["bodies"] if b["name"]=="middle_spread")
        self.assertEqual(b["component_names"],["entirely_new_root_keeper"])
        with self.assertRaisesRegex(ValueError,"explicit component body"):
            adapter.kinematic_blueprint(self.design,[dict(name="middle_spread_ring")])
        with self.assertRaisesRegex(ValueError,"explicit component body"):
            adapter.kinematic_blueprint(self.design,[dict(name="part",body="forearm")])

    def test_thumb_offset_and_rotation_order(self):
        bp=adapter.kinematic_blueprint(self.design,[])
        joints={j["name"]:j for j in bp["joints"]}
        self.assertEqual(joints["thumb_opposition"]["frame_parent_mm"][2][3],21)
        self.assertEqual(joints["middle_mcp"]["frame_parent_mm"][2][3],0)
        self.assertEqual(joints["thumb_base_sweep"]["axis_local"],[0,0,-1])
        R=np.array(joints["thumb_mcp"]["frame_parent_mm"])[:3,:3]
        np.testing.assert_allclose(R,adapter.rotation("z",25)@adapter.rotation("y",-65),rtol=0,atol=1e-12)
        self.assertGreater(np.max(np.abs(R-adapter.rotation("y",-65)@adapter.rotation("z",25))),.1)

    def test_all_axes_against_actual_webcam_mujoco(self):
        robot=self.robot;maximum_position=maximum_rotation=maximum_axis=0.
        cases=poses(self.design)
        for label,degrees in cases:
            robot.set(degrees)
            np.testing.assert_allclose(robot.angles,degrees,rtol=0,atol=1e-10,err_msg=label+" unexpectedly clipped")
            frames=adapter.forward_frames(self.design,np.deg2rad(degrees))
            bp=adapter.kinematic_blueprint(self.design,[])
            for j in bp["joints"]:
                name=j["name"];body=robot.model.body(name).id;jid=robot.model.joint(name).id
                T=frames[name];R=T[:3,:3]
                p=np.max(np.abs(T[:3,3]/1000-robot.data.xpos[body]))
                r=np.max(np.abs(R-robot.data.xmat[body].reshape(3,3)))
                a=np.max(np.abs(R@j["axis_local"]-robot.data.xaxis[jid]))
                maximum_position=max(maximum_position,float(p));maximum_rotation=max(maximum_rotation,float(r));maximum_axis=max(maximum_axis,float(a))
                self.assertLess(p,1e-10,label+" "+name)
                self.assertLess(r,1e-10,label+" "+name)
                self.assertLess(a,1e-10,label+" "+name)
                if "link_tip_local_mm" in j:
                    tip=(T@np.r_[j["link_tip_local_mm"],1])[:3]/1000
                    np.testing.assert_allclose(tip,robot.data.site_xpos[robot.model.site(name+"_tip").id],rtol=0,atol=1e-10)
        EVIDENCE["webcam_frame_comparison"]=dict(pose_count=len(cases),joints_per_pose=20,
            max_position_difference_m=maximum_position,max_rotation_component_difference=maximum_rotation,max_axis_component_difference=maximum_axis,
            scope="Kinematics only; mixed poses have no collision acceptance")

    def test_actual_fist_has_new_ranges_and_tips(self):
        degrees=np.array(self.design["fist_deg"])
        bp=adapter.kinematic_blueprint(self.design,[])
        ranges=np.array([j["range_deg"] for j in bp["joints"]])
        self.assertTrue(np.all(degrees>=ranges[:,0]) and np.all(degrees<=ranges[:,1]))
        self.assertTrue(np.all(degrees[[2,6,10,14]]>120))
        self.assertTrue(np.array_equal(ranges[[2,6,10,14],1],[135]*4))
        frames=adapter.forward_frames(self.design,np.deg2rad(degrees))
        for fi,name in enumerate(["index_dip","middle_dip","ring_dip","little_dip"]):
            L=self.design["fingers"][fi]["links"][2]
            tip=frames[name]@np.array([0,L,0,1])
            self.assertAlmostEqual(tip[2],16,places=7)

    def test_old_form_design_cannot_silently_supply_thumb(self):
        old=json.loads((ROOT/"cad/form_candidate_v1/design.json").read_text())
        with self.assertRaises(KeyError):
            adapter.kinematic_blueprint(old,[])

    def test_parallel_axis_units_and_mass_conservation(self):
        parts,masses=synthetic_parts(self.design)
        masses[0].update(com_world_mm=[-10,0,0],inertia_com_world_kg_m2=np.eye(3).tolist())
        parts.append(dict(name="second_palm_mass",body="palm",volume_mm3=20000.))
        masses.append(dict(name="second_palm_mass",mass_kg=.02,volume_mm3=20000.,density_kg_per_mm3=1e-6,
            com_world_mm=[10,0,0],inertia_com_world_kg_m2=np.eye(3).tolist()))
        result=adapter.aggregate_mass(self.design,parts,masses)
        palm=result["bodies"][0]
        self.assertAlmostEqual(palm["mass_kg"],.03)
        np.testing.assert_allclose(palm["com_local_mm"],[10/3,0,0],rtol=0,atol=1e-12)
        extra=(.01*.02/.03)*(.02**2)
        np.testing.assert_allclose(palm["inertia_com_local_kg_m2"],np.diag([2,2+extra,2+extra]),rtol=0,atol=1e-14)
        self.assertAlmostEqual(sum(b["mass_kg"] for b in result["bodies"]),sum(p["mass_kg"] for p in masses),places=14)

    def test_world_to_oblique_local_full_tensor(self):
        parts,masses=synthetic_parts(self.design)
        result=adapter.aggregate_mass(self.design,parts,masses)
        row=result["bodies"][19]  # thumb_mcp after palm +18 preceding bodies
        self.assertEqual(row["name"],"thumb_mcp")
        R=np.array(row["frame_world_mm"])[:3,:3]
        world=np.array(masses[19]["inertia_com_world_kg_m2"])
        local=np.array(row["inertia_com_local_kg_m2"])
        np.testing.assert_allclose(R@local@R.T,world,rtol=0,atol=1e-18)
        self.assertGreater(np.max(np.abs(local-np.diag(np.diag(local)))),1e-8)

    def test_missing_duplicate_mass_and_density_units_rejected(self):
        parts,masses=synthetic_parts(self.design)
        with self.assertRaisesRegex(ValueError,"one-to-one"):
            adapter.aggregate_mass(self.design,parts,masses[:-1])
        with self.assertRaisesRegex(ValueError,"one-to-one"):
            adapter.aggregate_mass(self.design,parts,masses+[masses[-1]])
        masses[2]["mass_kg"]*=1000
        with self.assertRaisesRegex(ValueError,"density units"):
            adapter.aggregate_mass(self.design,parts,masses)

    def test_empty_rigid_body_cannot_gain_placeholder_mass(self):
        parts,masses=synthetic_parts(self.design)
        with self.assertRaisesRegex(ValueError,"No real CAD mass"):
            adapter.aggregate_mass(self.design,parts[:-1],masses[:-1])

    def test_contract_rejects_legacy_frame_and_bad_units(self):
        c=synthetic_contract(self.design);adapter.validate_contract(c)
        c["joints"][17]["frame_parent_mm"][2][3]=7.5
        with self.assertRaisesRegex(ValueError,"Joint axis/frame/range"):
            adapter.validate_contract(c)
        c=synthetic_contract(self.design);c["units"]["inertia"]="kg*mm^2"
        with self.assertRaisesRegex(ValueError,"unit contract"):
            adapter.validate_contract(c)

    def test_transmission_needs_explicit_reciprocal_map(self):
        c=synthetic_contract(self.design)
        A=np.eye(20);A[19,16:19]=[.2,.3,.4]
        c["transmission"]["motor_angle_jacobian"]=A.tolist();adapter.validate_contract(c)
        # Pure work-map identity; not complete motor/cable energy validation.
        qd=np.linspace(-1,2,20);t=np.linspace(-.04,.06,20)
        self.assertAlmostEqual(float(t@(A@qd)),float((A.T@t)@qd),places=14)
        c["transmission"]["cable_actuator_indices"].remove(19)
        with self.assertRaisesRegex(ValueError,"direct actuator"):
            adapter.validate_contract(c)

    def test_legacy_tasks_are_not_flipped_or_substituted_for_fist(self):
        for name in ["small5","small10","large45","fast90_150ms","upstream_hold","rapid_reversal","independent_distal"]:
            t,q,m=adapter.scenario(self.legacy,name,.000125)
            old_t,old_q,old_m=self.legacy["evaluate"].scenario(name,.000125)
            self.assertTrue(np.array_equal(t,old_t) and np.array_equal(q,old_q))
            for a,b in zip(m,old_m):
                self.assertTrue(np.array_equal(a["q0"],b["q0"]))
        self.assertGreater(self.design["fist_deg"][6],132)

    def test_actual_contract_and_compiled_new_model(self):
        if CONTRACT is None:
            self.skipTest("New geometry not frozen; no mass/performance placeholder")
        c=json.loads(Path(CONTRACT).read_text(encoding="utf-8"))
        adapter.validate_contract(c,repo_root=ROOT)
        system=adapter.build_system(CONTRACT,REFERENCE,parameters=self.legacy["model"].DEFAULTS,
                                    allow_conceptual=True,repo_root=ROOT)
        m,d=system["model"],system["data"];mj=self.legacy["model"].mujoco
        self.assertEqual(m.nq,20+len(c["transmission"]["cable_actuator_indices"]))
        max_p=max_r=max_inertia=0.
        for label,degrees in poses(c["design"]):
            mj.mj_resetData(m,d);d.qpos[:20]=np.deg2rad(degrees);mj.mj_forward(m,d)
            frames=adapter.forward_frames(c["design"],np.deg2rad(degrees))
            for i,name in enumerate(adapter.NAMES):
                b=m.body(name).id;T=frames[name]
                max_p=max(max_p,float(np.max(np.abs(d.xpos[b]-T[:3,3]/1000))))
                max_r=max(max_r,float(np.max(np.abs(d.xmat[b].reshape(3,3)-T[:3,:3]))))
                actual=c["bodies"][i+1]
                self.assertAlmostEqual(float(m.body_mass[b]),actual["mass_kg"],places=13)
                np.testing.assert_allclose(m.body_ipos[b],np.array(actual["com_local_mm"])/1000,rtol=0,atol=1e-10)
                R=np.zeros(9);mj.mju_quat2Mat(R,m.body_iquat[b]);R=R.reshape(3,3)
                reconstructed=R@np.diag(m.body_inertia[b])@R.T
                max_inertia=max(max_inertia,float(np.max(np.abs(reconstructed-actual["inertia_com_local_kg_m2"]))))
                np.testing.assert_allclose(reconstructed,actual["inertia_com_local_kg_m2"],rtol=1e-7,atol=1e-12)
        self.assertLess(max_p,1e-10);self.assertLess(max_r,1e-10)
        EVIDENCE["new_compiled_model"]=dict(contract_sha256=adapter.digest(CONTRACT),pose_count=len(poses(c["design"])),
            max_position_difference_m=max_p,max_rotation_component_difference=max_r,
            max_full_inertia_component_difference_kg_m2=max_inertia,simulation_stepped=False)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--reference-root",type=Path,default=REFERENCE)
    p.add_argument("--contract",type=Path);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();REFERENCE=a.reference_root;CONTRACT=a.contract
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(AdapterTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    paths=[Path(__file__),Path(adapter.__file__),Path(__file__).with_name("build_contract.py"),
        ROOT/adapter.SHARED_REL,WEBCAM_REFERENCE/"provenance.json",*[WEBCAM_REFERENCE/p for p in WEBCAM_FILES],
        ROOT/"cad/webcam_articulation_v1/design.json",ROOT/"cad/webcam_articulation_v1/generated/scene.json"]
    report=dict(scope="Coordinate/mass-contract consistency only; no assembly or latency approval",
        tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),passed=result.wasSuccessful(),
        evidence=EVIDENCE,source_sha256={f.relative_to(ROOT).as_posix():adapter.digest(f) for f in paths},
        actual_contract_sha256=adapter.digest(CONTRACT) if CONTRACT else None,
        details=[str(test)+": "+message for test,message in result.failures+result.errors])
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)

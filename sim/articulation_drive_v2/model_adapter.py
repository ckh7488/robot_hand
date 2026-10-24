"""New articulated-CAD bridge; no camera, preview mass or old form geometry.

The reference axes are those of webcam_articulation_v1 / CadHand: root -Z,
flexion +X, thumb flexion offset21 mm, then Rz(25) @ Ry(-65). Coordinates and
ranges are read from the new CAD design. Every component needs an explicit body
assignment; names never silently determine rigid attachment. A body assignment
is a modeling assumption, not evidence that disconnected CAD is attached.

Only the hash-pinned form adapter's generic validation/legacy-loading utilities
are reused. Its blueprint, mass loader, geometry builder and angle-sign wrapper
are NEVER used. Finite motor/cable forces are the explicit pinned CP006 code.
"""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_VERSION = "articulation_drive_v2.contract.v1"
SHARED_REL = "sim/form_drive_v1/model_adapter.py"
SHARED_SHA256 = "6499f8cac525c77e1ee01a76bb4deced9d4a7f64de4aa619709a18d9d8c777b4"
NAMES = [f"{f}_{j}" for f in ("index", "middle", "ring", "little")
         for j in ("spread", "mcp", "pip", "dip")] + [
             "thumb_base_sweep", "thumb_opposition", "thumb_mcp", "thumb_ip"]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _shared():
    path = ROOT / SHARED_REL
    if digest(path) != SHARED_SHA256:
        raise ValueError("Generic shared adapter changed; review dependency before reuse")
    spec = importlib.util.spec_from_file_location("_articulation_generic_support", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_GENERIC = _shared()
_array = _GENERIC._array
_check_frame = _GENERIC._check_frame


def rotation(axis, degrees):
    a = math.radians(float(degrees))
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if axis == "z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    raise ValueError("Expected x/y/z axis")


def frame(position=(0, 0, 0), R=None):
    T = np.eye(4)
    T[:3, 3] = _array(position, (3,), "frame position")
    if R is not None:
        T[:3, :3] = _array(R, (3, 3), "frame rotation")
    return _check_frame(T, "rigid frame")


def _part_rows(part_metadata):
    if not isinstance(part_metadata, list):
        raise ValueError("Explicit part metadata must be a list")
    names = [p.get("name") for p in part_metadata]
    if any(not isinstance(n, str) or not n for n in names) or len(set(names)) != len(names):
        raise ValueError("Every component needs a unique nonempty name")
    for p in part_metadata:
        if p.get("body") not in ["palm"] + NAMES:
            raise ValueError("Missing/unknown explicit component body: " + p["name"])
    return part_metadata


def kinematic_blueprint(design, part_metadata):
    """Canonical20 joints and palm+20 bodies, CAD mm, explicit part assignment.

    Empty part metadata can inspect axes before geometry exists. Contract/model
    construction separately requires actual parts and positive body inertias.
    Pin/washer/bushing attachment is supplied by CAD metadata, never inferred.
    """
    if design.get("units") != "mm":
        raise ValueError("Articulated design must declare mm")
    if [f["name"] for f in design["fingers"]] != ["index", "middle", "ring", "little"]:
        raise ValueError("Expected index/middle/ring/little order")
    part_metadata = _part_rows(part_metadata)
    groups = {n: [] for n in ["palm"] + NAMES}
    for p in part_metadata:
        groups[p["body"]].append(p["name"])
    world = {"palm": np.eye(4)}
    joints = []
    configs = [*design["fingers"], dict(design["thumb"], name="thumb")]
    for fi, f in enumerate(configs):
        lengths = _array(f["links"], (3,), "link lengths")
        if np.any(lengths <= 0):
            raise ValueError("Link lengths must be positive")
        thumb = fi == 4
        # The oblique thumb keys must exist; an old straight-thumb design fails.
        if thumb:
            offset = f["flexion_offset"]
            child_R = rotation("z", f["second_yaw_deg"]) @ rotation("y", f["second_roll_deg"])
        else:
            offset = f.get("flexion_offset", [0, 0, 0])
            child_R = rotation("z", f.get("second_yaw_deg", 0)) @ rotation("y", f.get("second_roll_deg", 0))
        relative = [frame(f["root"], rotation("z", f.get("base_yaw_deg", 0))),
                    frame(offset), frame([0, lengths[0], 0], child_R), frame([0, lengths[1], 0])]
        parent = "palm"
        for ji in range(4):
            name = NAMES[4*fi+ji]
            key = name if thumb else name.split("_", 1)[1]
            limits = _array(design["joint_ranges_deg"][key], (2,), name + " range_deg")
            if limits[0] >= limits[1]:
                raise ValueError("Unordered CAD joint range: " + name)
            world[name] = world[parent] @ relative[ji]
            row = dict(name=name, parent=parent, frame_parent_mm=relative[ji].tolist(),
                       axis_local=[0, 0, -1] if ji == 0 else [1, 0, 0], range_deg=limits.tolist())
            if ji:
                row["link_tip_local_mm"] = [0, float(lengths[ji-1]), 0]
            joints.append(row)
            parent = name
    return dict(joints=joints, bodies=[dict(name=n, frame_world_mm=world[n].tolist(),
                component_names=groups[n]) for n in ["palm"] + NAMES], assumptions=[
        "Root -Z at design.root and flexion +X at design.flexion_offset; no old form root offset/sign transformation.",
        "The thumb child rotation is Rz(second_yaw_deg) @ Ry(second_roll_deg), in that order.",
        "Rigid attachment follows explicit CAD per-part body metadata; this does not prove physical attachment or retention.",
        "Design ranges and fist poses are kinematic references, not verified collision-free travel or manufacturing approval.",
        "Palm is the fixed test base; arm_reference is outside hand moving dynamics."])


def forward_frames(design, q_rad):
    """Analytical body frames in millimetres, including each joint's rotation."""
    q = _array(q_rad, (20,), "joint radians")
    result = {"palm": np.eye(4)}
    for i, j in enumerate(kinematic_blueprint(design, [])["joints"]):
        axis = np.array(j["axis_local"], dtype=float)
        x, y, z = axis
        K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        R = np.eye(3) + np.sin(q[i])*K + (1-np.cos(q[i]))*(K@K)
        result[j["name"]] = result[j["parent"]] @ np.array(j["frame_parent_mm"]) @ frame(R=R)
    return result


def check_inertia(mass, com, tensor, label):
    mass = float(mass)
    if not math.isfinite(mass) or mass <= 0:
        raise ValueError(label + ": positive actual component/body mass required")
    c = _array(com, (3,), label + " COM mm")
    I = _array(tensor, (3, 3), label + " inertia kg m^2")
    if not np.allclose(I, I.T, rtol=0, atol=1e-13):
        raise ValueError(label + ": inertia must be symmetric about COM")
    values = np.linalg.eigvalsh(I)
    if values[0] <= 0 or values[-1] > values[:2].sum()+1e-12:
        raise ValueError(label + ": nonphysical COM inertia")
    return mass, c, I


def principal_inertia(tensor):
    """Represent the complete COM tensor without MuJoCo's coarse eigen solver.

    This is only an orthogonal basis change: I = R diag(values) R.T. The input
    CAD tensor, mass, COM and physical joint/body frame are not altered.
    """
    I=_array(tensor,(3,3),"COM tensor")
    values,R=np.linalg.eigh(I)
    if values[0]<=0:
        raise ValueError("Positive principal inertia required")
    if np.linalg.det(R)<0:
        R[:,-1]*=-1
    if not np.allclose(R@np.diag(values)@R.T,I,rtol=1e-12,atol=1e-18):
        raise ValueError("Principal inertia representation lost tensor precision")
    return values,R


def aggregate_mass(design, part_metadata, mass_parts):
    """Combine exact part tensors using one mm->m conversion in parallel axes."""
    _part_rows(part_metadata)
    actual = {p["name"]: p for p in mass_parts}
    if len(actual) != len(mass_parts) or set(actual) != {p["name"] for p in part_metadata}:
        raise ValueError("Mass components must match explicit CAD part metadata one-to-one")
    for p in part_metadata:
        mp = actual[p["name"]]
        check_inertia(mp["mass_kg"], mp["com_world_mm"], mp["inertia_com_world_kg_m2"], p["name"])
        if "body" in mp and mp["body"] != p["body"]:
            raise ValueError("Conflicting CAD and mass body assignment: " + p["name"])
        if "volume_mm3" in p and (not math.isfinite(float(mp["volume_mm3"]))
                or abs(float(p["volume_mm3"])-float(mp["volume_mm3"])) > max(1e-5, abs(float(p["volume_mm3"]))*1e-9)):
            raise ValueError("Mass/scene solid volume mismatch: " + p["name"])
        if "density_kg_per_mm3" in mp and not math.isclose(mp["mass_kg"], mp["volume_mm3"]*mp["density_kg_per_mm3"], rel_tol=1e-9, abs_tol=1e-12):
            raise ValueError("Mass/volume/density units disagree: " + p["name"])
    blueprint = kinematic_blueprint(design, part_metadata)
    for b in blueprint["bodies"]:
        rows = [actual[n] for n in b["component_names"]]
        if not rows:
            raise ValueError("No real CAD mass assigned to body: " + b["name"])
        mass = sum(p["mass_kg"] for p in rows)
        com = sum((p["mass_kg"]*np.array(p["com_world_mm"]) for p in rows), np.zeros(3))/mass
        I = np.zeros((3, 3))
        for p in rows:
            delta = (np.array(p["com_world_mm"])-com)/1000
            I += np.array(p["inertia_com_world_kg_m2"]) + p["mass_kg"]*(delta@delta*np.eye(3)-np.outer(delta, delta))
        T = np.array(b["frame_world_mm"])
        R = T[:3, :3]
        b.update(mass_kg=float(mass), com_local_mm=(R.T@(com-T[:3,3])).tolist(),
                 inertia_com_local_kg_m2=(R.T@I@R).tolist())
    return blueprint


def _resolve_source(root, relative):
    if not isinstance(relative, str) or "\\" in relative or ":" in relative or ".." in Path(relative).parts:
        raise ValueError("Expected portable repository-relative source path")
    base = Path(root).resolve()
    path = (base/relative).resolve()
    if Path(relative).is_absolute() or not path.is_relative_to(base):
        raise ValueError("Source path escapes repository")
    return path


def verify_sources(contract, repo_root):
    files = contract["source_files"]
    if set(files) != {"design", "scene", "step", "mass"}:
        raise ValueError("Explicit design/scene/step/mass source files required")
    hashes = contract["source_sha256"]
    if not set(files.values()).issubset(hashes):
        raise ValueError("All CAD/mass inputs must have source hashes")
    for rel, expected in hashes.items():
        path = _resolve_source(repo_root, rel)
        if digest(path) != expected:
            raise ValueError("Source hash mismatch: " + rel)
    design = json.loads(_resolve_source(repo_root, files["design"]).read_text(encoding="utf-8"))
    scene = json.loads(_resolve_source(repo_root, files["scene"]).read_text(encoding="utf-8"))
    mass = json.loads(_resolve_source(repo_root, files["mass"]).read_text(encoding="utf-8"))
    if contract["design"] != design:
        raise ValueError("Embedded design differs from hashed articulated CAD")
    if contract["part_metadata"] != [{k:p[k] for k in ("name", "body", "volume_mm3") if k in p} for p in scene["hand"]]:
        raise ValueError("Contract body metadata differs from the actual hand scene")
    expected = aggregate_mass(design, contract["part_metadata"], mass["parts"])
    for actual, calculated in zip(contract["bodies"], expected["bodies"]):
        for key, atol in [("mass_kg",1e-12), ("com_local_mm",1e-8), ("inertia_com_local_kg_m2",1e-13)]:
            if not np.allclose(actual[key], calculated[key], rtol=1e-10, atol=atol):
                raise ValueError("Contract mass aggregation mismatch: " + actual["name"] + " " + key)


def validate_contract(contract, *, repo_root=None):
    c = copy.deepcopy(contract)
    if c.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported articulated-drive schema")
    if c.get("units") != {"length":"mm", "mass":"kg", "inertia":"kg*m^2", "joint_angle":"deg"}:
        raise ValueError("Explicit CAD/SI unit contract is required")
    if not c.get("mass_scope") or not c.get("limitations"):
        raise ValueError("Explicit incomplete-assembly scope/limitations required")
    bp = kinematic_blueprint(c["design"], c["part_metadata"])
    if [b["name"] for b in c["bodies"]] != ["palm"]+NAMES or len(c["joints"]) != 20:
        raise ValueError("Expected canonical palm+20 body and20 joint ordering")
    for b, expected in zip(c["bodies"], bp["bodies"]):
        check_inertia(b["mass_kg"], b["com_local_mm"], b["inertia_com_local_kg_m2"], b["name"])
        if b["component_names"] != expected["component_names"] or not b["component_names"]:
            raise ValueError("Incomplete or wrong explicit part/body grouping")
        T = _check_frame(b["frame_world_mm"], "body neutral frame")
        if not np.allclose(T, expected["frame_world_mm"], rtol=0, atol=1e-9):
            raise ValueError("Body frame differs from new articulated CAD")
    for j, expected in zip(c["joints"], bp["joints"]):
        if j != expected:
            raise ValueError("Joint axis/frame/range differs from new articulated CAD: " + expected["name"])
    tr = c["transmission"]
    if tr.get("status") != "assumed_not_validated" or not tr.get("assumptions"):
        raise ValueError("Explicit provisional transmission assumptions required")
    A = _array(tr["motor_angle_jacobian"], (20,20), "motor rad/joint rad Jacobian")
    cable = tr["cable_actuator_indices"]
    if not isinstance(cable,list) or any(type(i) is not int or not 0<=i<20 for i in cable) or cable!=sorted(set(cable)):
        raise ValueError("Cable indices must be sorted unique0..19 integers")
    if not np.isfinite(np.linalg.cond(A)) or np.linalg.cond(A)>1e8:
        raise ValueError("20 independent axes require invertible transmission")
    for i in range(20):
        if i not in cable and not np.array_equal(A[i],np.eye(20)[i]):
            raise ValueError("A direct actuator requires identity mapping")
        if i in cable and any(A[i,k]!=0 for k in range(20) if k not in range((i//4)*4,i+1)):
            raise ValueError("Initial cable model permits only same-chain ancestors")
    if repo_root is not None:
        verify_sources(c, repo_root)
    return c


def load_legacy(reference_root):
    return _GENERIC.load_legacy(reference_root)


def build_system(contract_path, reference_root, *, parameters, allow_conceptual=False, repo_root=None):
    """Compile real new tensors with explicit finite-actuator assumptions; no stepping."""
    if not allow_conceptual:
        raise ValueError("Unfinished assembly/transmission requires allow_conceptual=True")
    root_dir = Path(repo_root or ROOT).resolve()
    path = Path(contract_path)
    raw = path.read_bytes()
    c = validate_contract(json.loads(raw.decode("utf-8")), repo_root=root_dir)
    _shared()  # verify the generic dependency again at construction time
    legacy = load_legacy(reference_root)
    cfg = _GENERIC._parameters(parameters, legacy)
    old = legacy["model"]
    A = np.array(c["transmission"]["motor_angle_jacobian"])
    cable = c["transmission"]["cable_actuator_indices"]
    radius = c["transmission"].get("motor_spool_radius_m")
    if radius is None or not math.isclose(float(radius), cfg["motor_spool_radius_m"], rel_tol=0, abs_tol=1e-12):
        raise ValueError("Explicit transmission spool radius must match the finite cable-force parameter")
    xml = ET.Element("mujoco", model="articulation_drive_v2 provisional finite-actuator hand")
    ET.SubElement(xml,"compiler",angle="radian",autolimits="true",inertiafromgeom="false")
    ET.SubElement(xml,"option",timestep=str(cfg["dt_s"]),gravity=old.fmt(cfg["gravity_m_s2"]),integrator="implicitfast",iterations="40")
    default=ET.SubElement(xml,"default")
    ET.SubElement(default,"geom",contype="0",conaffinity="0",density="0")
    ET.SubElement(default,"joint",damping=str(cfg["joint_damping"]),frictionloss=str(cfg["joint_coulomb_Nm"]),limited="true")
    world=ET.SubElement(xml,"worldbody");elements={"palm":world};props={}
    for i,(j,b) in enumerate(zip(c["joints"],c["bodies"][1:])):
        T=np.array(j["frame_parent_mm"]);I=np.array(b["inertia_com_local_kg_m2"])
        moments,inertia_rotation=principal_inertia(I)
        body=ET.SubElement(elements[j["parent"]],"body",name=j["name"],pos=old.fmt(T[:3,3]/1000),quat=old.fmt(old.quat(T[:3,:3])))
        ET.SubElement(body,"inertial",pos=old.fmt(np.array(b["com_local_mm"])/1000),mass=str(b["mass_kg"]),
            diaginertia=old.fmt(moments),quat=old.fmt(old.quat(inertia_rotation)))
        ET.SubElement(body,"joint",name=j["name"],type="hinge",axis=old.fmt(j["axis_local"]),
            range=old.fmt(np.deg2rad(j["range_deg"])),armature=str(0 if i in cable else cfg["rotor_inertia_kg_m2"]))
        if "link_tip_local_mm" in j:
            ET.SubElement(body,"site",name=j["name"]+"_tip",pos=old.fmt(np.array(j["link_tip_local_mm"])/1000),size=".001")
        elements[j["name"]]=body
        props[j["name"]]=dict(mass_kg=b["mass_kg"],com_local_m=(np.array(b["com_local_mm"])/1000).tolist(),
            inertia_com_kg_m2=b["inertia_com_local_kg_m2"],source_parts=b["component_names"])
    rotor_index={}
    for i in cable:
        body=ET.SubElement(world,"body",name="rotor_"+NAMES[i])
        ET.SubElement(body,"inertial",pos="0 0 0",mass=".001",diaginertia=old.fmt([cfg["rotor_inertia_kg_m2"]]*3))
        ET.SubElement(body,"joint",name="rotor_"+NAMES[i],type="hinge",axis="1 0 0",limited="false",damping="0",frictionloss="0")
        rotor_index[i]=20+len(rotor_index)
    xml=ET.tostring(xml,encoding="unicode")
    model=old.mujoco.MjModel.from_xml_string(xml);data=old.mujoco.MjData(model)
    for i,name in enumerate(NAMES):
        if model.joint(name).qposadr[0]!=i or model.joint(name).dofadr[0]!=i:
            raise ValueError("Compiled joint order differs from canonical force interface")
    if path.read_bytes()!=raw:
        raise ValueError("Contract changed during compilation")
    verify_sources(c,root_dir)
    return dict(model=model,data=data,config=cfg,A=A,active=list(range(20)),cable=cable,rotor_index=rotor_index,
        variant="articulation20_conceptual",mass_properties=props,mass_provenance=c["mass_scope"],xml=xml,
        contract=c,legacy=legacy,contract_sha256=hashlib.sha256(raw).hexdigest(),provenance=dict(
            adapter_sha256=digest(__file__),generic_adapter_sha256=SHARED_SHA256,legacy_source_sha256=legacy["source_sha256"],
            form_source_sha256=c["source_sha256"],reference_angle_signs=[1]*20,collision_checked=False,
            hardware_performance_validated=False,motor_assembly_and_transmission_mass_complete=False,
            legacy_routing_parameters_not_applied=["joint_pulley_radius_m","cable_upstream_coefficient","cable_spread_coefficient"]))


def scenario(legacy, name, dt):
    """Legacy command angles already use the new reference's -Z spread sign.

    The seven legacy tasks are comparisons, not full coverage of the new fist
    with PIP angles up to135 degrees. A caller must separately declare timing
    for the CAD fist_path; this adapter does not silently replace old tasks.
    """
    return legacy["evaluate"].scenario(name,dt,reduced=False)


def run_simulation(system,time,commands,**kwargs):
    return _GENERIC.run_simulation(system,time,commands,**kwargs)

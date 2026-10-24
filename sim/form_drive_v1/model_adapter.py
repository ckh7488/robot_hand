"""Strict bridge from the new form CAD to the frozen CP006 force/evaluation code.

This file builds new kinematics and inertias; it NEVER calls the legacy geometry
builder or its V9/remote mass loaders. A contract must contain measured-from-BRep
mass properties, explicit joint ranges and an explicitly assumed transmission.
The currently unfinished transmission requires allow_conceptual=True. Numerical
output is not evidence of assembled routing, collision clearance or hardware
latency. Millimetres in CAD frames/COM are converted once; inertia is kg m^2.

Public entry points:
  kinematic_blueprint(design, part_names) -> joints, bodies, assumptions
  validate_contract(contract, *, repo_root=None) -> normalized checked input
  load_legacy(reference_root) -> pinned model/evaluate modules and hashes
  build_system(contract_path, reference_root, *, parameters,
               allow_conceptual=False, repo_root=None) -> legacy-compatible system
  scenario(legacy, name, dt) -> same physical direction as the old seven tasks
  run_simulation(system, time, commands, **kwargs) -> pinned force implementation

No model, force, mass, controller, motor or routing defaults are silently supplied.
The caller may explicitly copy legacy.DEFAULTS as a named comparison assumption.
The supplied Jacobian already includes motor/joint radius ratios and all coupling.
Legacy joint_pulley_radius_m, cable_upstream_coefficient and
cable_spread_coefficient are retained only for parameter-schema compatibility;
they do not modify that explicit Jacobian. motor_spool_radius_m remains active in
the cable extension-to-force law and must describe the declared modeled spool.
"""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import sys
import threading
import xml.etree.ElementTree as ET

import numpy as np

SCHEMA_VERSION = "form_drive_v1.contract.v1"
REFERENCE_SHA = "06b9ab4a867cede7d8ffce15a1ee70d90e3a193f"
REFERENCE_FILES = {
    "sim/remote_drive_v1/model.py": "2e76245ad8a43de53c9b204862508f37a80c62fab14ff91354b3e6ec27d6e128",
    "sim/remote_drive_v1/evaluate.py": "bbccd9165a26e4bc850d9f60655a0a2de18b19ef87164728f5b0439ab8b6b4a8",
}
DIGITS = ("index", "middle", "ring", "little")
NAMES = [f"{f}_{j}" for f in DIGITS for j in ("spread", "mcp", "pip", "dip")] + [
    "thumb_base_sweep", "thumb_opposition", "thumb_mcp", "thumb_ip"]
SPREAD_INDICES = [0, 4, 8, 12, 16]
_IMPORT_LOCK = threading.Lock()
_REQUIRED_INPUTS = {
    "cad/form_candidate_v1/design.json", "cad/form_candidate_v1/generated/scene.json",
    "cad/form_candidate_v1/generated/hand_form_candidate.step",
}


def _array(value, shape, label):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}: expected finite numeric array") from exc
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{label}: expected finite shape {shape}")
    return result


def _frame(position=(0, 0, 0), yaw_deg=0):
    angle = math.radians(float(yaw_deg))
    c, s = math.cos(angle), math.sin(angle)
    result = np.eye(4)
    result[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    result[:3, 3] = _array(position, (3,), "frame position")
    return result


def _check_frame(value, label):
    result = _array(value, (4, 4), label)
    R = result[:3, :3]
    if not (np.allclose(result[3], [0, 0, 0, 1], rtol=0, atol=1e-12)
            and np.allclose(R.T @ R, np.eye(3), rtol=0, atol=1e-10)
            and abs(np.linalg.det(R) - 1) < 1e-10):
        raise ValueError(label + ": expected a rigid, right-handed transform")
    return result


def _names_for(finger):
    return NAMES[16:] if finger == "thumb" else [finger + "_" + tag for tag in ("spread", "mcp", "pip", "dip")]


def component_body(name):
    """Rigid grouping assumption; joint pins are retained by the parent fork.

    Separate fascia/caps/bushings follow their link, despite unfinished physical
    retention. No part silently becomes fixed-palm merely because it is unknown.
    """
    if name.startswith(("Palm_", "Cover_", "Wrist_")):
        return "palm"
    for finger in (*DIGITS, "thumb"):
        names = _names_for(finger)
        if name == finger + "_spread_housing":
            return "palm"
        if name == finger + "_spread_ring" or name.startswith(finger + "_root_fork_"):
            return names[0]
        for j in range(3):
            if name.startswith(finger + f"_segment{j+1}_"):
                return names[j+1]
            if name == finger + f"_joint_pin_{j}":
                return names[j]
    raise ValueError("Unclassified CAD component: " + str(name))


def kinematic_blueprint(design, part_names):
    """Return 20 canonical joints and 21 body frames in CAD millimetres.

    The spread reference is the center plane of the existing root disk at z-7.5
    mm. Its axis line is +Z. Flexion is +X at the CAD root, 7.5 mm higher. No
    old 32 mm root-axis separation or old -Z coordinate sign is inherited.
    """
    if design.get("units") != "mm":
        raise ValueError("Form design units must be mm")
    fingers = design["fingers"]
    if [f["name"] for f in fingers] != list(DIGITS):
        raise ValueError("Expected four fingers in index/middle/ring/little order")
    part_names = list(part_names)
    if not part_names or len(set(part_names)) != len(part_names):
        raise ValueError("CAD component names must be nonempty and unique")
    groups = {name: [] for name in ["palm"] + NAMES}
    for name in part_names:
        groups[component_body(name)].append(name)
    world = {"palm": np.eye(4)}
    joints = []
    for f in fingers + [{**design["thumb"], "name": "thumb"}]:
        is_thumb = f["name"] == "thumb"
        names = _names_for(f["name"])
        root = _array(f["root"], (3,), "CAD root")
        lengths = _array(f["links"], (3,), "CAD links")
        if np.any(lengths <= 0):
            raise ValueError("CAD link lengths must be positive")
        frames = [_frame(root + [0, 0, -7.5], f["base_yaw_deg"] if is_thumb else 0),
                  _frame([0, 0, 7.5]),
                  _frame([0, lengths[0], 0], f["second_yaw_deg"] if is_thumb else 0),
                  _frame([0, lengths[1], 0])]
        parent = "palm"
        for j, name in enumerate(names):
            world[name] = world[parent] @ frames[j]
            joints.append(dict(name=name, parent=parent, frame_parent_mm=frames[j].tolist(),
                               axis_local=[0, 0, 1] if j == 0 else [1, 0, 0]))
            parent = name
    return dict(joints=joints, bodies=[dict(name=name, frame_world_mm=world[name].tolist(),
        component_names=groups[name]) for name in ["palm"] + NAMES], assumptions=[
        "Spread +Z reference at the root disk center plane (root z - 7.5 mm); flex +X at root.",
        "Pin0 follows spread, pin1 follows proximal link, pin2 follows middle link; retention is unfinished.",
        "Separate link fascia, bushings and tip caps are assumed rigidly retained to their listed body.",
        "The fixed palm is the test base. The reused arm reference is excluded from moving dynamics.",
    ])


def _records(rows, expected, label):
    if not isinstance(rows, list) or len(rows) != len(expected):
        raise ValueError(f"{label}: expected exactly {len(expected)} records")
    if [r.get("name") for r in rows] != expected:
        raise ValueError(label + ": unexpected order, duplicate or missing canonical name")
    return {r["name"]: r for r in rows}


def _check_sources(contract, repo_root):
    hashes = contract.get("source_sha256", {})
    if not isinstance(hashes, dict) or not _REQUIRED_INPUTS.issubset(hashes):
        raise ValueError("Contract must hash the form design, scene and STEP inputs")
    for name, digest in hashes.items():
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts or ":" in name or "\\" in name:
            raise ValueError("Unsafe source path: " + name)
        if not isinstance(digest, str) or not re.fullmatch("[a-f0-9]{64}", digest):
            raise ValueError("Invalid SHA256 for " + name)
        if repo_root is not None:
            root = Path(repo_root).resolve()
            path = (root / relative).resolve()
            if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise ValueError("Stale/missing source hash: " + name)
    if repo_root is not None:
        root = Path(repo_root)
        design = json.loads((root / "cad/form_candidate_v1/design.json").read_text(encoding="utf-8"))
        scene = json.loads((root / "cad/form_candidate_v1/generated/scene.json").read_text(encoding="utf-8"))
        if design != contract["design"]:
            raise ValueError("Embedded design differs from the hashed form design")
        scene_names = [r["name"] for r in scene["hand"]]
        extras = {r["name"] for r in contract.get("additional_components", [])}
        included = [n for row in contract["bodies"] for n in row["component_names"] if n not in extras]
        if sorted(included) != sorted(scene_names):
            raise ValueError("Body grouping must include each actual hand-scene component exactly once")


def validate_contract(contract, *, repo_root=None):
    """Reject stale frames, missing parts, invalid SI inertias and silent routing defaults."""
    c = copy.deepcopy(contract)
    if c.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported form-drive contract schema")
    if not c.get("mass_scope") or not isinstance(c.get("limitations"), list) or not c["limitations"]:
        raise ValueError("Explicit mass scope and limitations are required")
    bodies = _records(c.get("bodies"), ["palm"] + NAMES, "bodies")
    joints = _records(c.get("joints"), NAMES, "joints")
    extra_rows = c.get("additional_components", [])
    extras = {r["name"]: r for r in extra_rows}
    if len(extras) != len(extra_rows) or any(r.get("body") not in bodies or not r.get("scope") for r in extra_rows):
        raise ValueError("Additional components need unique names, an explicit body and scope")
    parts = [name for row in c["bodies"] for name in row["component_names"]]
    if len(parts) != len(set(parts)) or not parts:
        raise ValueError("Component mass may not be omitted or counted twice")
    if not set(extras).issubset(parts):
        raise ValueError("Declared additional component is absent from body mass")
    blueprint = kinematic_blueprint(c["design"], [n for n in parts if n not in extras])
    expected_bodies = {b["name"]: b for b in blueprint["bodies"]}
    for b in c["bodies"]:
        label = b["name"]
        mass = float(b["mass_kg"])
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError(label + ": positive BRep body mass required")
        _array(b["com_local_mm"], (3,), label + " COM mm")
        I = _array(b["inertia_com_local_kg_m2"], (3, 3), label + " inertia kg m^2")
        if not np.allclose(I, I.T, rtol=0, atol=1e-13):
            raise ValueError(label + ": COM inertia must be symmetric")
        eig = np.linalg.eigvalsh(I)
        if np.min(eig) <= 0 or eig[-1] > eig[:2].sum() + 1e-12:
            raise ValueError(label + ": COM inertia is not physically positive/triangle-consistent")
        actual = _check_frame(b["frame_world_mm"], label + " world frame")
        if not np.allclose(actual, expected_bodies[label]["frame_world_mm"], rtol=0, atol=1e-8):
            raise ValueError(label + ": stale/wrong neutral CAD body frame")
        expected_parts = expected_bodies[label]["component_names"] + [r["name"] for r in extra_rows if r["body"] == label]
        if sorted(b["component_names"]) != sorted(expected_parts):
            raise ValueError(label + ": incorrect rigid-body component grouping")
    for expected in blueprint["joints"]:
        j = joints[expected["name"]]
        if j["parent"] != expected["parent"]:
            raise ValueError(j["name"] + ": incorrect parent")
        actual = _check_frame(j["frame_parent_mm"], j["name"] + " parent frame")
        axis = _array(j["axis_local"], (3,), j["name"] + " axis")
        if not (np.allclose(actual, expected["frame_parent_mm"], rtol=0, atol=1e-8)
                and np.allclose(axis, expected["axis_local"], rtol=0, atol=1e-12)):
            raise ValueError(j["name"] + ": joint differs from the explicit form-CAD axis convention")
        limits = _array(j["range_deg"], (2,), j["name"] + " range")
        if limits[0] >= limits[1]:
            raise ValueError(j["name"] + ": ordered explicit range required")
    tr = c["transmission"]
    if tr.get("status") != "assumed_not_validated" or not tr.get("assumptions"):
        raise ValueError("Current form transmission must explicitly remain assumed_not_validated")
    A = _array(tr["motor_angle_jacobian"], (20, 20), "motor-angle Jacobian (motor rad/joint rad)")
    cable = tr["cable_actuator_indices"]
    if not isinstance(cable, list) or any(type(i) is not int or not 0 <= i < 20 for i in cable) or cable != sorted(set(cable)):
        raise ValueError("Cable actuator indices must be unique, sorted integers in 0..19")
    if not np.isfinite(np.linalg.cond(A)) or np.linalg.cond(A) > 1e8:
        raise ValueError("20 independent actuators require a numerically invertible Jacobian")
    for i in range(20):
        if i not in cable:
            if not np.array_equal(A[i], np.eye(20)[i]):
                raise ValueError("A direct actuator must have an identity mapping; otherwise model it as cable")
        else:
            allowed = set(range((i // 4) * 4, i + 1))
            if any(A[i, k] != 0 for k in range(20) if k not in allowed):
                raise ValueError("This initial routing model permits only the actuator's own chain ancestors")
    _check_sources(c, repo_root)
    return c


def load_legacy(reference_root):
    """Load two exact-SHA sources, without altering their text or importing dirty main."""
    root = Path(reference_root).resolve()
    for relative, digest in REFERENCE_FILES.items():
        path = root / relative
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError("Legacy source differs from pinned " + REFERENCE_SHA + ": " + relative)
    suffix = hashlib.sha256(str(root).encode()).hexdigest()[:12]
    def load(name, relative):
        spec = importlib.util.spec_from_file_location(name, root / relative)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    with _IMPORT_LOCK:
        model = load("_form_legacy_model_" + suffix, "sim/remote_drive_v1/model.py")
        # evaluate imports only NAMES from `model`; restore any caller binding.
        sentinel = object()
        old = sys.modules.get("model", sentinel)
        sys.modules["model"] = model
        try:
            evaluator = load("_form_legacy_evaluate_" + suffix, "sim/remote_drive_v1/evaluate.py")
        finally:
            if old is sentinel:
                del sys.modules["model"]
            else:
                sys.modules["model"] = old
    if model.NAMES != NAMES:
        raise ValueError("Pinned legacy canonical joint order unexpectedly differs")
    return dict(model=model, evaluate=evaluator, reference_sha=REFERENCE_SHA,
                source_sha256=dict(REFERENCE_FILES), reference_root=str(root))


def _parameters(parameters, legacy):
    if parameters is None or set(parameters) != set(legacy["model"].DEFAULTS):
        raise ValueError("Pass every explicit legacy parameter key; defaults/unknown keys are not accepted")
    cfg = copy.deepcopy(parameters)
    for name, value in cfg.items():
        a = np.asarray(value, dtype=float)
        if not np.all(np.isfinite(a)):
            raise ValueError("Non-finite parameter " + name)
    for name in ["dt_s", "controller_period_s", "voltage_V", "stall_torque_Nm", "stall_current_A",
                 "no_load_rpm", "rotor_inertia_kg_m2", "friction_velocity_scale_rad_s",
                 "motor_spool_radius_m", "joint_pulley_radius_m"]:
        if cfg[name] <= 0:
            raise ValueError("Positive parameter required: " + name)
    for name in ["received_to_servo_s", "current_limit_A", "pwm_slope_register", "pwm_slope_mV_per_ms",
                 "kp_Nm_per_rad", "kd_Nm_s_per_rad", "joint_damping", "joint_coulomb_Nm",
                 "cable_stiffness_N_per_m", "cable_damping_N_s_per_m", "cable_preload_N",
                 "cable_mu", "cable_backlash_rad", "motor_viscous_Nm_s_per_rad"]:
        if cfg[name] < 0:
            raise ValueError("Nonnegative parameter required: " + name)
    if cfg["mass_scale"] != 1:
        raise ValueError("New BRep mass/inertia may not be silently rescaled")
    _array(cfg["gravity_m_s2"], (3,), "gravity")
    for name in ["controller_period_s", "received_to_servo_s"]:
        steps = cfg[name] / cfg["dt_s"]
        if abs(steps - round(steps)) > 1e-9 or (name == "controller_period_s" and steps < 1):
            raise ValueError("Explicit timing must be an integer number of timesteps: " + name)
    return cfg


def build_system(contract_path, reference_root, *, parameters, allow_conceptual=False, repo_root=None):
    """Build a new 20-DOF hand with finite legacy forces and explicit new routing.

    No visualization mesh is used to infer inertia or contacts. Collision handling
    remains disabled and geometry feasibility requires separate CAD evidence.
    """
    if not allow_conceptual:
        raise ValueError("Current motor/routing/retention assumptions require allow_conceptual=True")
    contract_path = Path(contract_path).resolve()
    raw = contract_path.read_bytes()
    if repo_root is None:
        repo_root = Path(__file__).resolve().parents[2]
    c = validate_contract(json.loads(raw.decode("utf-8")), repo_root=repo_root)
    legacy = load_legacy(reference_root)
    cfg = _parameters(parameters, legacy)
    old = legacy["model"]
    cable = c["transmission"]["cable_actuator_indices"]
    A = np.array(c["transmission"]["motor_angle_jacobian"], dtype=float)
    root = ET.Element("mujoco", model="form_drive_v1 conceptual rigid hand")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", inertiafromgeom="false")
    ET.SubElement(root, "option", timestep=str(cfg["dt_s"]), gravity=old.fmt(cfg["gravity_m_s2"]),
                  integrator="implicitfast", iterations="40")
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "geom", contype="0", conaffinity="0", density="0")
    ET.SubElement(default, "joint", damping=str(cfg["joint_damping"]),
                  frictionloss=str(cfg["joint_coulomb_Nm"]), limited="true")
    world = ET.SubElement(root, "worldbody")
    elements = {"palm": world}
    props = {}
    body_rows = {b["name"]: b for b in c["bodies"]}
    for i, j in enumerate(c["joints"]):
        name = j["name"]
        T = np.array(j["frame_parent_mm"])
        body = ET.SubElement(elements[j["parent"]], "body", name=name,
            pos=old.fmt(T[:3, 3]/1000), quat=old.fmt(old.quat(T[:3, :3])))
        b = body_rows[name]
        I = np.array(b["inertia_com_local_kg_m2"])
        ET.SubElement(body, "inertial", pos=old.fmt(np.array(b["com_local_mm"])/1000),
                      mass=str(b["mass_kg"]), fullinertia=old.fmt([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]]))
        ET.SubElement(body, "joint", name=name, type="hinge", axis=old.fmt(j["axis_local"]),
                      range=old.fmt(np.deg2rad(j["range_deg"])),
                      armature=str(0 if i in cable else cfg["rotor_inertia_kg_m2"]))
        elements[name] = body
        props[name] = dict(mass_kg=b["mass_kg"], com_local_m=(np.array(b["com_local_mm"])/1000).tolist(),
                          inertia_com_kg_m2=b["inertia_com_local_kg_m2"], source_parts=b["component_names"],
                          inertia_method="New form BRep COM inertia; declared density and incomplete assembly scope")
    rotor_index = {}
    for i in cable:
        body = ET.SubElement(world, "body", name="rotor_" + NAMES[i])
        # Same fixed-location scalar equivalent rotor as the pinned force model;
        # its dummy 1 g is not counted as the physical motor's chassis mass.
        ET.SubElement(body, "inertial", pos="0 0 0", mass=".001",
                      diaginertia=old.fmt([cfg["rotor_inertia_kg_m2"]]*3))
        ET.SubElement(body, "joint", name="rotor_" + NAMES[i], type="hinge", axis="1 0 0",
                      limited="false", damping="0", frictionloss="0")
        rotor_index[i] = 20 + len(rotor_index)
    xml = ET.tostring(root, encoding="unicode")
    model = old.mujoco.MjModel.from_xml_string(xml)
    data = old.mujoco.MjData(model)
    for i, name in enumerate(NAMES):
        if model.joint(name).qposadr[0] != i or model.joint(name).dofadr[0] != i:
            raise ValueError("Compiled joint order differs from the force/evaluator contract")
    if contract_path.read_bytes() != raw:
        raise ValueError("Contract changed during model construction")
    _check_sources(c, repo_root)
    return dict(model=model, data=data, config=cfg, A=A, active=list(range(20)),
                cable=cable, rotor_index=rotor_index, variant="form20_conceptual", mass_properties=props,
                mass_provenance=c["mass_scope"], xml=xml, contract=c, legacy=legacy,
                contract_sha256=hashlib.sha256(raw).hexdigest(),
                provenance=dict(reference_sha=REFERENCE_SHA, source_sha256=dict(REFERENCE_FILES),
                    form_source_sha256=c["source_sha256"], adapter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    collision_checked=False, hardware_performance_validated=False,
                    reference_angle_signs=[-1 if i in SPREAD_INDICES else 1 for i in range(20)],
                    legacy_routing_parameters_not_applied=["joint_pulley_radius_m", "cable_upstream_coefficient", "cable_spread_coefficient"],
                    motor_assembly_and_transmission_mass_complete=False))


def scenario(legacy, name, dt):
    """Preserve physical direction while mapping old -Z spread to new +Z axes."""
    time, command, moves = legacy["evaluate"].scenario(name, dt, reduced=False)
    signs = np.ones(20)
    signs[SPREAD_INDICES] = -1
    command = command * signs
    moves = copy.deepcopy(moves)
    for move in moves:
        move["q0"] = np.array(move["q0"]) * signs
        move["q1"] = np.array(move["q1"]) * signs
    return time, command, moves


def run_simulation(system, time, commands, **kwargs):
    """Validate command shape/range before running the pinned finite-force code."""
    time = np.asarray(time, dtype=float)
    command = _array(commands, (len(time), 20), "commands")
    if len(time) < 2 or not np.all(np.isfinite(time)) or not np.allclose(np.diff(time), system["config"]["dt_s"], rtol=0, atol=1e-12):
        raise ValueError("Strict finite, regular command timestamps required")
    limits = np.deg2rad([j["range_deg"] for j in system["contract"]["joints"]])
    if np.any(command < limits[:,0] - 1e-12) or np.any(command > limits[:,1] + 1e-12):
        raise ValueError("Command exceeds the explicitly declared conceptual joint range")
    return system["legacy"]["model"].run_simulation(system, time, command, **kwargs)

"""Saved-STEP gesture and sampled-transition audit; no geometry repair or release.

Named native parts, volume/bounds correspondence and the generic FK/STEP tools
are reused from the hash-frozen v2 verifier. Input CAD is supplied explicitly.
All same-body and cross-body pairs are in scope. Proximity is reported separately
from material intersection; touching pins/seats are never silently exempted.
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import cadquery as cq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
V2_REL = "cad/articulation_drive_v2/verify.py"
V2_SHA = "79a0b66b8c5ede499e870e954b32ac0d2e679c4f96a9e3e0bf4d0fdca44680f3"
OVERLAP_MM3 = 0.02
BOOLEAN_TOL_MM = 1e-6


def progress(stage, **values):
    print(json.dumps(dict(progress=stage, **values), allow_nan=False), flush=True)


class ProgressParts(list):
    """Observe the frozen loader's completed rows without modifying its code."""
    def __init__(self, total):
        super().__init__(); self.total=total; self.started=time.perf_counter()

    def append(self, row):
        super().append(row)
        if len(self)==1 or len(self)%10==0 or len(self)==self.total:
            progress('part_loaded', completed=len(self), total=self.total, name=row['name'],
                     elapsed_s=time.perf_counter()-self.started)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(value, separators=(",", ":"), allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def inside(root, relative):
    relative = str(relative).replace("\\", "/")
    if Path(relative).is_absolute() or ":" in relative or ".." in Path(relative).parts:
        raise ValueError("Expected safe relative input path: " + relative)
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError("Missing or escaping input: " + relative)
    return path


def load_v2(repository):
    path = repository / V2_REL
    if digest(path) != V2_SHA:
        raise ValueError("Frozen verifier changed; review a new dependency explicitly")
    spec = importlib.util.spec_from_file_location("_gesture_saved_step_verifier_v2", path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def valid_angles(value, limits, label):
    q = np.asarray(value, float)
    if q.shape != (20,) or not np.all(np.isfinite(q)):
        raise ValueError(label + ": expected 20 finite degrees")
    if np.any(q < limits[:, 0] - 1e-8) or np.any(q > limits[:, 1] + 1e-8):
        raise ValueError(label + ": outside declared joint range; no clipping allowed")
    return q


def expand_poses(document, design, names, mode, step_deg):
    """Subdivide each declared angle-space segment; timing is not inferred."""
    if document["joint_names"] != names:
        raise ValueError("Pose canonical joint ordering differs from CAD")
    limits = np.array([design["joint_ranges_deg"][name if name.startswith("thumb_") else name.split("_", 1)[1]]
                       for name in names], float)
    gestures = document["gestures"]
    if not gestures or len({g["id"] for g in gestures}) != len(gestures):
        raise ValueError("Empty or duplicate gesture identities")
    angle_map = {g["id"]: valid_angles(g["angles_deg"], limits, g["id"]) for g in gestures}
    observations = [dict(id="gesture:" + g["id"], kind="endpoint", gesture=g["id"],
                         label_ko=g.get("label_ko"), angles_deg=angle_map[g["id"]].tolist()) for g in gestures]
    transition_rows = []
    if mode == "full":
        transitions = document["transitions"]
        if len({t["id"] for t in transitions}) != len(transitions):
            raise ValueError("Duplicate transition identities")
        for transition in transitions:
            waypoints = transition["waypoints"]
            f = np.array([w["fraction"] for w in waypoints], float)
            q = np.array([valid_angles(w["angles_deg"], limits, transition["id"]) for w in waypoints])
            if len(waypoints) < 2 or not np.all(np.isfinite(f)) or f[0] != 0 or f[-1] != 1 or np.any(np.diff(f) <= 0):
                raise ValueError("Transition needs ordered fractions 0..1: " + transition["id"])
            if (not np.allclose(q[0], angle_map[transition["source"]], atol=1e-10, rtol=0)
                    or not np.allclose(q[-1], angle_map[transition["target"]], atol=1e-10, rtol=0)):
                raise ValueError("Transition endpoint differs from named gesture")
            samples = []; maximum = 0.
            for segment in range(len(waypoints) - 1):
                steps = max(1, int(math.ceil(float(np.max(np.abs(q[segment + 1] - q[segment]))) / step_deg)))
                for k in range(steps + 1):
                    if segment and not k: continue
                    alpha = k / steps
                    # Preserve exact declared endpoint floats for cache identity.
                    angles = q[segment].copy() if not k else q[segment + 1].copy() if k == steps else q[segment] + alpha * (q[segment + 1] - q[segment])
                    row = dict(id=f"transition:{transition['id']}:{len(samples):04d}", kind="transition",
                        transition=transition["id"], source=transition["source"], target=transition["target"],
                        segment=segment, segment_fraction=alpha, path_fraction=float(f[segment] + alpha * (f[segment + 1] - f[segment])),
                        phase=waypoints[segment + 1].get("phase"), angles_deg=angles.tolist())
                    if samples:
                        maximum = max(maximum, float(np.max(np.abs(angles - samples[-1]["angles_deg"]))))
                    samples.append(row)
            if maximum > step_deg + 1e-9: raise ValueError("Sampling step exceeded declaration")
            transition_rows.append(dict(id=transition["id"], source=transition["source"], target=transition["target"],
                source_status=transition.get("status"), waypoint_count=len(waypoints), sample_count=len(samples),
                maximum_actual_joint_step_deg=maximum, observations=[r["id"] for r in samples]))
            observations.extend(samples)
    return observations, transition_rows


def kinematic_definition(design):
    configs = [*design["fingers"], dict(design["thumb"], name="thumb")]
    return dict(units=design["units"], joint_ranges_deg=design["joint_ranges_deg"],
                palm_target_envelope_mm=design["palm_target_envelope_mm"], chains=[dict(
        name=c["name"], root=c["root"], links=c["links"], base_yaw_deg=c.get("base_yaw_deg", 0),
        second_yaw_deg=c.get("second_yaw_deg", 0), second_roll_deg=c.get("second_roll_deg", 0),
        flexion_offset=c.get("flexion_offset", [0, 0, 0])) for c in configs])


def quaternion_matrix(values):
    q = np.asarray(values, float); q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


def preview_frames(xml_root, names, angles):
    """FK from frozen preview XML without importing the active webcam app."""
    compiler = xml_root.find("compiler")
    if compiler is None or compiler.get("angle") != "radian":
        raise ValueError("Expected explicit radian preview XML")
    q = dict(zip(names, np.deg2rad(angles))); result = {"palm": (np.zeros(3), np.eye(3))}
    def walk(node, parent):
        for body in node.findall("body"):
            if any(key in body.attrib for key in ("euler", "axisangle", "xyaxes", "zaxis")):
                raise ValueError("Unsupported preview orientation; explicit quaternion required")
            transform = np.eye(4)
            transform[:3, 3] = np.fromstring(body.get("pos", "0 0 0"), sep=" ") * 1000
            transform[:3, :3] = quaternion_matrix(np.fromstring(body.get("quat", "1 0 0 0"), sep=" "))
            world = parent @ transform
            joints = body.findall("joint")
            if len(joints) > 1: raise ValueError("Expected single hinge per preview body")
            for joint in joints:
                if joint.get("type", "hinge") != "hinge" or joint.get("name") not in q:
                    raise ValueError("Unexpected preview joint")
                axis = np.fromstring(joint.get("axis", "0 0 1"), sep=" "); axis /= np.linalg.norm(axis)
                origin = np.fromstring(joint.get("pos", "0 0 0"), sep=" ") * 1000
                angle = q[joint.get("name")] - float(joint.get("ref", "0"))
                x, y, z = axis; skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                r = np.eye(3) + math.sin(angle)*skew + (1 - math.cos(angle))*(skew @ skew)
                motion = np.eye(4); motion[:3, :3] = r; motion[:3, 3] = origin - r @ origin
                world = world @ motion
            name = body.get("name")
            if name in names: result[name] = (world[:3, 3].copy(), world[:3, :3].copy())
            walk(body, world)
    worldbody = xml_root.find("worldbody")
    if worldbody is None: raise ValueError("Missing preview worldbody")
    walk(worldbody, np.eye(4))
    if set(result) != {"palm", *names}: raise ValueError("Preview did not produce every canonical body")
    return result


def compare_reference_fk(v2, design, accepted_root, observations):
    xml = ET.parse(accepted_root / "preview.xml").getroot()
    names = v2.joint_names(); poses = [np.zeros(20)]
    poses += [np.array(row["angles_deg"]) for row in observations if row["kind"] == "endpoint"]
    motion = read_json(accepted_root / "motion.json")["frames"]
    poses += [np.array(row["angles_deg"]) for row in motion]
    max_position = 0.; max_rotation = 0.; worst = None
    for pi, q in enumerate(poses):
        expected = preview_frames(xml, names, q); actual = v2.frames(design, q)
        for name in expected:
            dp = float(np.max(np.abs(expected[name][0] - actual[name][0])))
            dr = float(np.max(np.abs(expected[name][1] - actual[name][1])))
            if dp > max_position: max_position, worst = dp, dict(pose=pi, body=name, angles_deg=q.tolist())
            max_rotation = max(max_rotation, dr)
    return dict(passed=max_position <= 1e-6 and max_rotation <= 1e-10, pose_count=len(poses),
        source="Independent frozen preview.xml hierarchy/hinge FK versus generic CAD design FK",
        max_position_component_error_mm=max_position, max_rotation_component_error=max_rotation,
        position_limit_mm=1e-6, rotation_component_limit=1e-10, worst_position=worst,
        scope="Kinematic frame comparison only; accepted STL appearance is not a BRep/collision certificate")


def body_chains(names):
    result = {"palm": ()}
    for i, name in enumerate(names): result[name] = tuple(range(i // 4 * 4, i + 1))
    return result


def relative_indices(a, b, chains):
    first, second = chains[a], chains[b]; common = 0
    while common < min(len(first), len(second)) and first[common] == second[common]: common += 1
    return first[common:] + second[common:]


def transform_point(point, old_frame, new_frame):
    old_origin, old_rotation = old_frame; new_origin, new_rotation = new_frame
    return (new_origin + new_rotation @ old_rotation.T @ (np.array(point) - old_origin)).tolist()


class PairScreen:
    """Exact pair results cached only on analytical relative-configuration identity.

    The key is the pair of immutable solids plus all joint angles below the lowest
    common ancestor of their explicit bodies. Shared ancestor rotations cancel.
    No rounded angles, tessellation exclusion or approximate pose cache is used.
    """
    def __init__(self, v2, parts, rows, design, near_mm, skip_proximity=False):
        self.v2, self.parts, self.design, self.near_mm = v2, parts, design, near_mm
        self.skip_proximity=skip_proximity
        self.names = list(parts); self.bodies = {r["name"]: r["body"] for r in rows}
        self.neutral = v2.frames(design, np.zeros(20)); self.chains = body_chains(v2.joint_names())
        self.pair_indices = {(a, b): relative_indices(self.bodies[a], self.bodies[b], self.chains)
                             for i, a in enumerate(self.names) for b in self.names[i + 1:]}
        self.cache = {}; self.cache_hits = 0; self.exact_boolean_calls = 0; self.distance_calls = 0
        self.exact_boolean_seconds=0.; self.exact_distance_seconds=0.

    def evaluate(self, angles):
        started = time.perf_counter(); q = np.asarray(angles, float)
        current = self.v2.frames(self.design, q)
        shapes = {name: self.v2.relocate(shape, current[self.bodies[name]], self.neutral[self.bodies[name]])
                  for name, shape in self.parts.items()}
        boxes = np.array([self.v2.bounds(shapes[name]) for name in self.names])
        # Euclidean AABB separation is a conservative lower bound on solid gap.
        separations = np.maximum(np.maximum(boxes[:, None, 0] - boxes[None, :, 1],
                                             boxes[None, :, 0] - boxes[:, None, 1]), 0.)
        lower_distance = np.linalg.norm(separations, axis=2)
        threshold=0. if self.skip_proximity else self.near_mm
        candidate_indices = np.argwhere(np.triu(lower_distance <= threshold + BOOLEAN_TOL_MM, 1))
        progress('pose_broadphase', candidates=len(candidate_indices), parts=len(shapes),
                 seconds=time.perf_counter()-started, proximity_enabled=not self.skip_proximity)
        rows = []; hits_before = self.cache_hits; booleans_before = self.exact_boolean_calls; distances_before = self.distance_calls
        boolean_seconds_before=self.exact_boolean_seconds; distance_seconds_before=self.exact_distance_seconds
        for pair_number,(ai, bi) in enumerate(candidate_indices,1):
            a, b = self.names[ai], self.names[bi]; selected = self.pair_indices[(a, b)]
            key = (a, b, tuple(float(q[j]).hex() for j in selected))
            if key in self.cache:
                original, original_frame = self.cache[key]; row = copy.deepcopy(original); self.cache_hits += 1
                for label in ("intersection_center_mm",):
                    if label in row: row[label] = transform_point(row[label], original_frame, current[self.bodies[a]])
                row["reused_exact_relative_configuration"] = True
            else:
                row = dict(a=a, b=b, body_a=self.bodies[a], body_b=self.bodies[b],
                    same_simulator_body=self.bodies[a] == self.bodies[b], relative_joint_indices=list(selected),
                    material_intersection_mm3=0., intersection_tested=False, near_distance_mm=None,
                    collision=False, errors=[], reused_exact_relative_configuration=False)
                overlap = np.minimum(boxes[ai, 1], boxes[bi, 1]) - np.maximum(boxes[ai, 0], boxes[bi, 0])
                # A thin positive AABB overlap can still enclose reportable
                # material volume. Only a nonpositive box overlap excludes it.
                if np.all(overlap > 0.):
                    row["intersection_tested"] = True; self.exact_boolean_calls += 1
                    operation_started=time.perf_counter()
                    try:
                        common = shapes[a].intersect(shapes[b], tol=BOOLEAN_TOL_MM); volume = float(common.Volume())
                        if not math.isfinite(volume) or volume < -1e-9: raise ValueError("Invalid common volume")
                        row.update(material_intersection_mm3=volume, collision=volume > OVERLAP_MM3)
                        if volume > OVERLAP_MM3: row["intersection_center_mm"] = list(common.Center().toTuple())
                    except Exception as exc:
                        row["errors"].append(dict(operation="intersection", error=type(exc).__name__ + ": " + str(exc)))
                    elapsed=time.perf_counter()-operation_started;self.exact_boolean_seconds+=elapsed
                    if elapsed>=5:progress('slow_exact_operation',operation='intersection',a=a,b=b,seconds=elapsed)
                if self.skip_proximity:
                    row['distance_status']='not_computed_explicit_skip_proximity'
                elif row["collision"]:
                    row["near_distance_mm"] = 0.
                    row['distance_status']='zero_from_positive_material_intersection'
                else:
                    self.distance_calls += 1
                    operation_started=time.perf_counter()
                    try:
                        distance = float(shapes[a].distance(shapes[b]))
                        if not math.isfinite(distance) or distance < 0: raise ValueError("Invalid solid distance")
                        row["near_distance_mm"] = distance
                        row['distance_status']='computed_exact_native_shape_distance'
                    except Exception as exc:
                        row['distance_status']='operation_failed'
                        row["errors"].append(dict(operation="distance", error=type(exc).__name__ + ": " + str(exc)))
                    elapsed=time.perf_counter()-operation_started;self.exact_distance_seconds+=elapsed
                    if elapsed>=5:progress('slow_exact_operation',operation='distance',a=a,b=b,seconds=elapsed)
                self.cache[key] = (copy.deepcopy(row), current[self.bodies[a]])
            rows.append(row)
            if pair_number%100==0 or pair_number==len(candidate_indices):
                progress('pose_exact_pairs',completed=pair_number,total=len(candidate_indices),last_pair=[a,b],
                         boolean_seconds=self.exact_boolean_seconds-boolean_seconds_before,
                         distance_seconds=self.exact_distance_seconds-distance_seconds_before,
                         cache_hits=self.cache_hits-hits_before,elapsed_s=time.perf_counter()-started)
        collisions = [r for r in rows if r["collision"]]
        errors = [r for r in rows if r["errors"]]
        near = [r for r in rows if r["near_distance_mm"] is not None and r["near_distance_mm"] <= self.near_mm + 1e-8]
        same_counts = Counter(self.bodies.values())
        return dict(angles_deg=q.tolist(), total_pairs_in_scope=len(self.names)*(len(self.names)-1)//2,
            same_body_pairs_in_scope=sum(n*(n-1)//2 for n in same_counts.values()),
            aabb_candidates=len(rows), exact_boolean_calls=self.exact_boolean_calls-booleans_before,
            exact_distance_calls=self.distance_calls-distances_before, relative_configuration_cache_hits=self.cache_hits-hits_before,
            exact_boolean_seconds=self.exact_boolean_seconds-boolean_seconds_before,
            exact_distance_seconds=self.exact_distance_seconds-distance_seconds_before,
            clearance_reported=not self.skip_proximity,
            collision_count=len(collisions), cross_body_collision_count=sum(not r["same_simulator_body"] for r in collisions),
            collision_details=collisions, near_pairs=near, operation_failures=errors,
            exact_candidate_results=rows, minimum_computed_distance_mm=min((r["near_distance_mm"] for r in rows if r["near_distance_mm"] is not None), default=None),
            no_reportable_material_collision_and_no_operation_failure=not collisions and not errors,
            proximity_is_not_a_clearance_pass=True, elapsed_s=time.perf_counter()-started)


def collect_inputs(repository, candidate, poses_path, accepted, scene, build, pose_document):
    paths = {Path(__file__).resolve(), repository / V2_REL, poses_path,
             candidate / "design.json", candidate / "generated/scene.json", candidate / "generated/build_report.json",
             candidate / "generated/hand.step", accepted / "manifest.json"}
    pose_source = poses_path.with_suffix(".py")
    if pose_source.is_file(): paths.add(pose_source)
    expected = {}
    for document in (scene, build):
        if document.get("inputs_unchanged") is not True: raise ValueError("CAD start/end guard absent")
        for relative, sha in document["source_sha256"].items():
            path = inside(repository, relative)
            if path in expected and expected[path] != sha: raise ValueError("Conflicting source hash")
            paths.add(path); expected[path] = sha
    for row in scene["hand"]:
        for key, sha_key in (("step", "sha256_step"), ("mesh", "sha256_stl")):
            path = inside(candidate, row[key]); paths.add(path); expected[path] = row[sha_key]
    for relative, sha in read_json(accepted / "manifest.json").items():
        path = inside(accepted, relative); paths.add(path); expected[path] = sha
    for relative, sha in pose_document["reference"]["source_sha256"].items():
        path = inside(repository, relative); paths.add(path)
        if path in expected and expected[path] != sha: raise ValueError("Pose/reference hash conflict")
        expected[path] = sha
    for relative, sha in pose_document.get('study_scope', {}).get('source_sha256', {}).items():
        path = inside(repository, relative); paths.add(path)
        if path in expected and expected[path] != sha: raise ValueError('Study source hash conflict')
        expected[path] = sha
    start = {str(path): digest(path) for path in sorted(paths)}
    stale = [str(path) for path, sha in expected.items() if start[str(path)] != sha]
    if stale: raise ValueError("Stale source or accepted-form manifest: " + repr(stale))
    return start


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad-root", type=Path, required=True)
    parser.add_argument("--poses", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("endpoints", "full"), default="full")
    parser.add_argument("--max-joint-step-deg", type=float, default=5.)
    parser.add_argument("--near-mm", type=float, default=1.)
    parser.add_argument("--skip-proximity", action="store_true", help="Skip native minimum distances explicitly; exact material collision checks remain active")
    parser.add_argument("--accepted-root", type=Path, default=ROOT / "cad/accepted_form_v1")
    args = parser.parse_args()
    if not math.isfinite(args.max_joint_step_deg) or not 0 < args.max_joint_step_deg <= 5:
        parser.error("Joint step must be positive and no greater than 5 degrees")
    if not math.isfinite(args.near_mm) or args.near_mm < 0: parser.error("Nonnegative finite near distance required")
    candidate = args.cad_root.resolve(); repository = candidate.parents[1]
    poses_path = args.poses.resolve(); output = args.output.resolve(); accepted = args.accepted_root.resolve()
    if output.exists(): raise FileExistsError("Use a new output path; prior evidence is preserved")
    if output.suffix.lower() != ".json": raise ValueError("Output must be a new JSON report")
    report = dict(schema_version="gesture_hand_v3.audit.v1", completed=False, manufacturing_approved=False,
        hardware_performance_validated=False, mode=args.mode, cad_root=str(candidate), poses_source=str(poses_path),
        clearance_reported=not args.skip_proximity,skip_proximity=args.skip_proximity,
        thresholds=dict(material_overlap_mm3=OVERLAP_MM3, boolean_tolerance_mm=BOOLEAN_TOL_MM,
                        maximum_joint_step_deg=args.max_joint_step_deg, proximity_reporting_mm=None if args.skip_proximity else args.near_mm),
        parts=[], mesh_topology=[], cases={}, observations=[], transitions=[], errors=[],
        scope=["All separate hand native STEP parts, including parts assigned to the same body.",
               "No pair exemptions, mesh approximations or contact-based suppression of material overlap.",
               "Declared gesture endpoints and angle-spaced transition samples only; no continuous-clearance guarantee.",
               ("Native minimum distances are explicitly omitted; null is not a zero gap or a clearance pass." if args.skip_proximity else
                "Pin/seat touching and other near distances are preserved separately and do not establish fabrication fit."),
               "No arm/motor packaging, force simulation, strength, tolerances or hardware latency approval."],
        cache_method="Immutable pair plus unrounded joint values below lowest common body ancestor; common ancestors cancel exactly",
        runtime=dict(python=sys.version, numpy=np.__version__, cadquery=cq.__version__, platform=platform.platform()))
    before = {}
    try:
        v2 = load_v2(repository)
        design = read_json(candidate / "design.json"); scene = read_json(candidate / "generated/scene.json")
        build = read_json(candidate / "generated/build_report.json"); pose_document = read_json(poses_path)
        before = collect_inputs(repository, candidate, poses_path, accepted, scene, build, pose_document)
        if str(output) in before: raise ValueError("Output is an input path")
        report["input_sha256"] = before
        accepted_design = read_json(accepted / "design.json")
        if (kinematic_definition(design) != kinematic_definition(accepted_design)
                or kinematic_definition(pose_document["kinematic_design"]) != kinematic_definition(accepted_design)):
            raise ValueError("Candidate/pose dimensions or axes differ from accepted form; no silent substitution")
        report["accepted_kinematic_definition_matched"] = True
        observations, transitions = expand_poses(pose_document, design, v2.joint_names(), args.mode, args.max_joint_step_deg)
        report["observations"] = observations; report["transitions"] = transitions
        report["accepted_reference_fk"] = compare_reference_fk(v2, design, accepted, observations)
        if not report["accepted_reference_fk"]["passed"]: raise ValueError("Candidate FK differs from accepted preview reference")
        write_json(output, report)
        report['parts']=ProgressParts(len(scene['hand']))
        progress('native_parts_load_started',total=len(scene['hand']))
        parts = v2.load_parts(scene, candidate, report)
        progress('assembly_roundtrip_started',parts=len(parts))
        report["assembly_roundtrip"] = v2.assembly_roundtrip(candidate / "generated/hand.step", scene["hand"], parts)
        progress('assembly_roundtrip_completed',passed=report['assembly_roundtrip']['passed'])
        write_json(output,report)
        if not report["assembly_roundtrip"]["passed"]: raise ValueError("Native assembly/individual STEP mismatch")
        if not all(p["valid"] and p["solid_count"] == 1 for p in report["parts"]):
            raise ValueError("Invalid/disconnected saved native part")
        engine = PairScreen(v2, parts, scene["hand"], design, args.near_mm,args.skip_proximity)
        known = {}; last_save = time.monotonic()
        for observation in observations:
            key = tuple(float(v).hex() for v in observation["angles_deg"])
            if key not in known:
                case_id = f"pose_{len(known):04d}"; known[key] = case_id
                progress('pose_started',case=case_id,observation=observation['id'])
                result = engine.evaluate(observation["angles_deg"]); result["first_observation"] = observation["id"]
                report["cases"][case_id] = result
                print(json.dumps(dict(case=case_id, observation=observation["id"], collisions=result["collision_count"],
                    errors=len(result["operation_failures"]), exact_booleans=result["exact_boolean_calls"],
                    cache_hits=result["relative_configuration_cache_hits"], seconds=result["elapsed_s"])), flush=True)
            observation["case_id"] = known[key]
            if time.monotonic() - last_save >= 10:
                write_json(output, report); last_save = time.monotonic()
        report["gesture_results"] = {o["gesture"]: dict(case_id=o["case_id"],
            collision_count=report["cases"][o["case_id"]]["collision_count"],
            operation_failure_count=len(report["cases"][o["case_id"]]["operation_failures"]))
            for o in observations if o["kind"] == "endpoint"}
        report["summary"] = dict(gesture_count=len(report["gesture_results"]), transition_count=len(transitions),
            observation_count=len(observations), unique_pose_count=len(known), exact_pair_cache_entries=len(engine.cache),
            exact_boolean_calls=engine.exact_boolean_calls, exact_distance_calls=engine.distance_calls,
            exact_boolean_seconds=engine.exact_boolean_seconds,exact_distance_seconds=engine.exact_distance_seconds,
            clearance_reported=not args.skip_proximity,
            relative_pair_cache_hits=engine.cache_hits,
            cases_with_material_collision=sum(bool(r["collision_details"]) for r in report["cases"].values()),
            cases_with_operation_failure=sum(bool(r["operation_failures"]) for r in report["cases"].values()),
            no_reportable_collision_in_sampled_scope=all(r["no_reportable_material_collision_and_no_operation_failure"] for r in report["cases"].values()),
            full_transition_scope_executed=args.mode == "full")
        report["completed"] = True
    except Exception as exc:
        report["errors"].append(dict(type=type(exc).__name__, message=str(exc), traceback=traceback.format_exc()))
    finally:
        after = {}; changed = []
        for name, sha in before.items():
            try: after[name] = digest(name)
            except OSError as exc: after[name] = dict(error=str(exc))
            if after[name] != sha: changed.append(name)
        report["input_hash_guard"] = dict(passed=bool(before) and not changed, start=before, end=after, changed=changed)
        if changed: report["completed"] = False
        write_json(output, report)
    print(json.dumps(dict(output=str(output), completed=report["completed"], errors=report["errors"])), flush=True)
    return 0 if report["completed"] and report["input_hash_guard"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

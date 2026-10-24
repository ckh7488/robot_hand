"""Read-only audit of the saved articulation-drive candidate geometry.

The input STEP files are never repaired. Every unordered part pair is in scope,
including parts assigned to the same rigid body. Thirteen sampled fist poses do
not establish continuous clearance, assembly feasibility, strength or performance.
Only the report directory is written. Run this after the CAD input is frozen.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time
import traceback

import numpy as np
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.gp import gp_Trsf

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OVERLAP_MM3 = 0.02
BOOLEAN_TOL_MM = 1e-6
BBOX_TOL_MM = 0.002
AXIS_TOL_MM = 1e-4
VOLUME_ABS_TOL_MM3 = 0.05
VOLUME_REL_TOL = 1e-5
FINGERS = ("index", "middle", "ring", "little", "thumb")


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def safe_relative(base, relative):
    """Scene and provenance paths may not escape their declared repository root."""
    relative = str(relative).replace("\\", "/")
    if not relative or Path(relative).is_absolute() or ":" in relative:
        raise ValueError("Not a relative input path: " + relative)
    candidate = (Path(base) / relative).resolve()
    if not candidate.is_relative_to(Path(base).resolve()):
        raise ValueError("Input escapes declared root: " + relative)
    if not candidate.is_file():
        raise FileNotFoundError(candidate)
    return candidate


def rotate(axis, degrees):
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    if axis == "x":
        return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
    if axis == "y":
        return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    if axis == "z":
        return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    raise ValueError(axis)


def joint_names():
    return [finger + "_" + suffix for finger in FINGERS for suffix in
            (("base_sweep", "opposition", "mcp", "ip") if finger == "thumb"
             else ("spread", "mcp", "pip", "dip"))]


def frames(design, angles_deg):
    """Independent FK; neutral STEP is already in world mm.

    Spread is negative Z. Thumb translation precedes opposition, and distal
    yaw then roll precede its MCP X rotation. These conventions are explicit:
    reusing the earlier form candidate's axes would be incorrect here.
    """
    angles_deg = np.asarray(angles_deg, float)
    if angles_deg.shape != (20,) or not np.all(np.isfinite(angles_deg)):
        raise ValueError("Expected 20 finite joint angles in degrees")
    result = {"palm": (np.zeros(3), np.eye(3))}
    configs = [*design["fingers"], dict(design["thumb"], name="thumb")]
    if [c["name"] for c in configs] != list(FINGERS):
        raise ValueError("Unexpected joint ordering")
    names = joint_names()
    for fi, config in enumerate(configs):
        a = angles_deg[fi * 4:fi * 4 + 4]
        origin = np.array(config["root"], float)
        rotation = rotate("z", config.get("base_yaw_deg", 0) - a[0])
        result[names[fi * 4]] = (origin.copy(), rotation.copy())
        origin += rotation @ np.array(config.get("flexion_offset", [0, 0, 0]), float)
        rotation = rotation @ rotate("x", a[1])
        result[names[fi * 4 + 1]] = (origin.copy(), rotation.copy())
        for ji in (2, 3):
            origin += rotation @ np.array([0, config["links"][ji - 2], 0])
            if ji == 2:
                rotation = (rotation @ rotate("z", config.get("second_yaw_deg", 0))
                            @ rotate("y", config.get("second_roll_deg", 0)))
            rotation = rotation @ rotate("x", a[ji])
            result[names[fi * 4 + ji]] = (origin.copy(), rotation.copy())
    return result


def relocate(shape, frame, neutral):
    origin, rotation = frame
    old_origin, old_rotation = neutral
    r = rotation @ old_rotation.T
    t = origin - r @ old_origin
    transform = gp_Trsf()
    transform.SetValues(float(r[0, 0]), float(r[0, 1]), float(r[0, 2]), float(t[0]),
                        float(r[1, 0]), float(r[1, 1]), float(r[1, 2]), float(t[1]),
                        float(r[2, 0]), float(r[2, 1]), float(r[2, 2]), float(t[2]))
    return shape.moved(cq.Location(transform))


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, box, False, False)
    values = box.Get()
    return np.array([values[:3], values[3:]], dtype=float)


def topology(path):
    raw = Path(path).read_bytes()
    if len(raw) < 84:
        raise ValueError("Not a binary STL")
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + 50 * count:
        raise ValueError("STL count/length mismatch")
    dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    triangles = np.frombuffer(raw[84:], dtype=dtype)["v"].astype(float)
    if not np.all(np.isfinite(triangles)):
        raise ValueError("Non-finite STL vertex")
    vertices, index = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    index = index.reshape(-1, 3)
    edges = np.concatenate((index[:, [0, 1]], index[:, [1, 2]], index[:, [2, 0]]))
    _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
    area2 = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                  triangles[:, 2] - triangles[:, 0]), axis=1)
    return dict(triangles=count, vertices=len(vertices),
                boundary_edges=int(np.sum(counts == 1)),
                nonmanifold_edges=int(np.sum(counts > 2)),
                degenerate_triangles=int(np.sum(area2 < 1e-10)),
                welding="exact float32 coordinates; no mesh repair")


def sample_poses(design):
    path = design["fist_path"]
    fractions = np.array([p["fraction"] for p in path], float)
    angles = np.array([p["angles_deg"] for p in path], float)
    if (angles.shape != (len(path), 20) or not np.all(np.isfinite(angles))
            or np.any(np.diff(fractions) <= 0) or fractions[0] != 0 or fractions[-1] != 1):
        raise ValueError("Malformed fist path")
    if np.any(angles[0] != 0) or not np.allclose(angles[-1], design["fist_deg"], atol=1e-10, rtol=0):
        raise ValueError("Path must start in neutral and end at the declared fist")
    samples = sorted(set(float(t) for a, b in zip(fractions, fractions[1:])
                         for t in np.linspace(a, b, 5)))
    result = []
    for t in samples:
        q = np.array([np.interp(t, fractions, angles[:, j]) for j in range(20)])
        label = "open" if t == 0 else "fist" if t == 1 else f"fist_path_{t:.6f}"
        result.append(dict(name=label, fraction=t, angles_deg=q.tolist(), kind="fist_path"))
    counter = np.zeros(20); counter[0] = -22
    result.append(dict(name="index_spread_minus22_others_open", fraction=None,
                       angles_deg=counter.tolist(), kind="retained_source_counterexample"))
    return result


def load_parts(scene, candidate, report):
    rows = scene["hand"]
    if not rows or len({r["name"] for r in rows}) != len(rows):
        raise ValueError("Empty or duplicate component names")
    parts = {}
    known_bodies = {"palm", *joint_names()}
    for row in rows:
        name = row["name"]
        if row["body"] not in known_bodies or row["mesh_units"] != "mm":
            raise ValueError("Unrecognized body or units: " + name)
        step = safe_relative(candidate, row["step"])
        mesh = safe_relative(candidate, row["mesh"])
        if digest(step) != row["sha256_step"] or digest(mesh) != row["sha256_stl"]:
            raise ValueError("Scene file hash mismatch: " + name)
        shape = cq.importers.importStep(str(step)).val()
        volume = float(shape.Volume())
        dv = abs(volume - row["volume_mm3"])
        db = float(np.max(np.abs(bounds(shape) - row["bbox_mm"])))
        topology_row = dict(name=name, **topology(mesh))
        item = dict(name=name, body=row["body"], role=row["role"], source=row.get("source"), valid=bool(shape.isValid()),
                    solid_count=len(shape.Solids()), volume_mm3=volume,
                    scene_volume_error_mm3=dv, scene_bbox_error_mm=db,
                    mapping_passed=bool(dv <= max(VOLUME_ABS_TOL_MM3, row["volume_mm3"] * VOLUME_REL_TOL)
                                        and db <= BBOX_TOL_MM))
        report["parts"].append(item)
        report["mesh_topology"].append(topology_row)
        if not item["mapping_passed"]:
            raise ValueError("Individual STEP provenance mapping mismatch: " + name)
        parts[name] = shape
    return parts


def assembly_roundtrip(step_path, rows, parts):
    solids = cq.importers.importStep(str(step_path)).val().Solids()
    indices = [r["step_index"] for r in rows]
    if sorted(indices) != list(range(len(rows))) or len(solids) != len(rows):
        raise ValueError("Assembly index/solid count mismatch")
    checks = []
    for row in rows:
        source = parts[row["name"]]
        solid = solids[row["step_index"]]
        dv = abs(source.Volume() - solid.Volume())
        db = float(np.max(np.abs(bounds(source) - bounds(solid))))
        checks.append(dict(name=row["name"], step_index=row["step_index"],
                           volume_error_mm3=dv, bbox_error_mm=db,
                           passed=bool(dv <= max(VOLUME_ABS_TOL_MM3, source.Volume() * VOLUME_REL_TOL)
                                       and db <= BBOX_TOL_MM)))
    return dict(solid_count=len(solids), checks=checks,
                passed=all(r["passed"] for r in checks),
                identity_scope="Independent named STEP is canonical; assembly order, volume and bounds checked")


def pair_intersection(a, b, shapes, boxes, same_body):
    delta = np.minimum(boxes[a][1], boxes[b][1]) - np.maximum(boxes[a][0], boxes[b][0])
    if np.any(delta <= BOOLEAN_TOL_MM):
        return None
    row = dict(a=a, b=b, same_simulator_body=same_body)
    try:
        common = shapes[a].intersect(shapes[b], tol=BOOLEAN_TOL_MM)
        volume = float(common.Volume())
        if not math.isfinite(volume) or volume < -1e-9:
            raise ValueError("Invalid common volume")
        row.update(volume_mm3=volume, above_reporting_threshold=volume > OVERLAP_MM3)
        if volume > OVERLAP_MM3:
            row["center_mm"] = list(common.Center().toTuple())
    except Exception as exc:
        row.update(error=type(exc).__name__ + ": " + str(exc), above_reporting_threshold=None)
    return row


def screen_pose(parts, bodies, design, pose, same_body_cache=None):
    started = time.perf_counter()
    neutral = frames(design, np.zeros(20))
    current = frames(design, pose["angles_deg"])
    shapes = {name: relocate(shape, current[bodies[name]], neutral[bodies[name]])
              for name, shape in parts.items()}
    boxes = {name: bounds(shape) for name, shape in shapes.items()}
    checks = []; cache = {} if same_body_cache is None else same_body_cache
    names = list(parts)
    same_count = sum(n * (n - 1) // 2 for n in Counter(bodies.values()).values())
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            same = bodies[a] == bodies[b]
            if same and same_body_cache is not None:
                original = cache[(a, b)]
                if original is None:
                    continue
                row = dict(original, reused_rigid_body_invariant=True)
                if "center_mm" in row:
                    body = bodies[a]; on, rn = neutral[body]; oc, rc = current[body]
                    row["center_mm"] = (oc + rc @ rn.T @ (np.array(row["center_mm"]) - on)).tolist()
            else:
                row = pair_intersection(a, b, shapes, boxes, same)
                if same:
                    cache[(a, b)] = row
                if row is None:
                    continue
                row["reused_rigid_body_invariant"] = False
            checks.append(row)
    hits = [r for r in checks if r.get("above_reporting_threshold")]
    failures = [r for r in checks if "error" in r]
    return dict(**pose, total_unordered_pairs=len(names) * (len(names) - 1) // 2,
                same_body_pairs_in_scope=same_count,
                different_body_pairs_in_scope=len(names) * (len(names) - 1) // 2 - same_count,
                broad_phase_candidates=len(checks),
                exact_operations_this_pose=sum(not r["reused_rigid_body_invariant"] for r in checks),
                same_body_results_reused=sum(r["reused_rigid_body_invariant"] for r in checks),
                exact_pair_results=checks, overlaps=hits, boolean_failures=failures,
                overlap_count=len(hits), different_body_overlap_count=sum(not r["same_simulator_body"] for r in hits),
                no_reportable_overlap_and_no_boolean_failure=not hits and not failures,
                seconds=time.perf_counter() - started), cache


def cylinder_surfaces(shape, origin, direction):
    """Finite surface bounds projected onto the declared axis (world mm)."""
    origin = np.asarray(origin, float); direction = np.asarray(direction, float)
    direction /= np.linalg.norm(direction)
    results = []
    for index, face in enumerate(shape.Faces()):
        surface = BRepAdaptor_Surface(face.wrapped)
        if surface.GetType() != GeomAbs_Cylinder:
            continue
        cylinder = surface.Cylinder(); axis = cylinder.Axis()
        point = np.array([axis.Location().X(), axis.Location().Y(), axis.Location().Z()])
        actual = np.array([axis.Direction().X(), axis.Direction().Y(), axis.Direction().Z()])
        angle_error = float(np.linalg.norm(np.cross(actual, direction)))
        offset = float(np.linalg.norm(np.cross(point - origin, direction)))
        if angle_error > 1e-6 or offset > AXIS_TOL_MM:
            continue
        # The analytic cylinder parameter direction is axial. Vertices alone can
        # under-cover a trimmed face; UV endpoint samples provide the finite span.
        projected = [float(np.dot(np.array(v.toTuple()) - origin, direction)) for v in face.Vertices()]
        for u in (surface.FirstUParameter(), surface.LastUParameter()):
            for v in (surface.FirstVParameter(), surface.LastVParameter()):
                p = surface.Value(u, v)
                projected.append(float(np.dot(np.array([p.X(), p.Y(), p.Z()]) - origin, direction)))
        if not projected or not np.all(np.isfinite(projected)):
            continue
        results.append(dict(face_index=index, radius_mm=float(cylinder.Radius()),
                            axis_offset_mm=offset, axis_direction_cross_error=angle_error,
                            axial_span_mm=[min(projected), max(projected)]))
    return results


def merged_span_covers(intervals, requested, tolerance=AXIS_TOL_MM):
    low, high = map(float, requested)
    cursor = low
    for start, end in sorted(intervals):
        if end < cursor - tolerance:
            continue
        if start > cursor + tolerance:
            return False
        cursor = max(cursor, end)
        if cursor >= high - tolerance:
            return True
    return False


def planar_faces(shape, origin, normal):
    origin = np.array(origin, float); normal = np.array(normal, float)
    normal /= np.linalg.norm(normal)
    faces = []
    for face in shape.Faces():
        surface = BRepAdaptor_Surface(face.wrapped)
        if surface.GetType() != GeomAbs_Plane:
            continue
        plane = surface.Plane(); point = plane.Location(); axis = plane.Axis().Direction()
        p = np.array([point.X(), point.Y(), point.Z()])
        d = np.array([axis.X(), axis.Y(), axis.Z()])
        if abs(np.dot(p - origin, normal)) <= AXIS_TOL_MM and np.linalg.norm(np.cross(d, normal)) <= 1e-6:
            faces.append(face)
    return faces


def seating_check(check, parts):
    row = dict(id=check["id"], definition=check)
    try:
        first, second = check["parts"]
        origin = check["plane_origin_mm"]; normal = np.array(check["normal"], float)
        normal /= np.linalg.norm(normal)
        a = planar_faces(parts[first], origin, normal)
        b = planar_faces(parts[second], origin, normal)
        if not a or not b:
            row.update(contact_area_mm2=0., passed=False, reason="Missing planar seating face")
            return row
        common = cq.Compound.makeCompound(a).intersect(cq.Compound.makeCompound(b), tol=BOOLEAN_TOL_MM)
        if "patch_rectangle_mm" in check:
            # Current contract patches are horizontal; reject a different plane
            # rather than silently choosing a new local yaw convention.
            if np.linalg.norm(np.cross(normal, [0, 0, 1])) > 1e-6:
                raise ValueError("Patch yaw convention requires a horizontal seat")
            yaw = math.radians(check.get("patch_yaw_deg", 0))
            plane = cq.Plane(origin=tuple(origin), xDir=(math.cos(yaw), math.sin(yaw), 0), normal=tuple(normal))
            wire = cq.Workplane(plane).rect(*check["patch_rectangle_mm"]).wire().val()
            patch = cq.Face.makeFromWires(wire)
            common = common.intersect(patch, tol=BOOLEAN_TOL_MM)
        area = float(common.Area())
        nominal = float(check["nominal_contact_area_mm2"])
        tolerance = max(0.02, nominal * 1e-3)
        row.update(contact_area_mm2=area, nominal_contact_area_mm2=nominal,
                   area_difference_mm2=area - nominal, area_comparison_tolerance_mm2=tolerance,
                   distance_mm=float(parts[first].distance(parts[second])),
                   passed=bool(area > 0 and abs(area - nominal) <= tolerance))
    except Exception as exc:
        row.update(passed=False, error=type(exc).__name__ + ": " + str(exc))
    return row


def mounting_check(check, parts):
    """Nominal finite through-path; axial overlap is not actual thread engagement."""
    row = dict(id=check["id"], definition=check, cylinder_checks=[], clearance_checks=[], seating_checks=[])
    try:
        origin = np.array(check["axis_point_mm"], float)
        direction = np.array(check["axis_direction"], float)
        direction /= np.linalg.norm(direction)
        if not np.allclose(direction, [0, 0, 1], atol=1e-8, rtol=0):
            raise ValueError("Mount z-span contract requires positive world Z axis")
        required = ("shaft_radius_mm", "clearance_radius_mm", "nut_bore_radius_mm", "housing_bore_span_z_mm", "washer")
        missing = [key for key in required if key not in check]
        if missing:
            raise ValueError("Incomplete finite mount contract: " + repr(missing))
        definitions = [
            (check["bolt"], check["shaft_radius_mm"], check["shaft_span_z_mm"], "bolt_shaft"),
            (check["nut"], check["nut_bore_radius_mm"], check["nut_span_z_mm"], "nut_nominal_bore"),
            (check["chassis"], check["clearance_radius_mm"], check["floor_span_z_mm"], "floor_bore"),
            (check["housing"], check["clearance_radius_mm"], check["housing_bore_span_z_mm"], "housing_lower_bore")]
        for name, radius, world_span, label in definitions:
            surfaces = cylinder_surfaces(parts[name], origin, direction)
            surfaces = [s for s in surfaces if abs(s["radius_mm"] - radius) <= AXIS_TOL_MM]
            span = (np.array(world_span) - origin[2]).tolist()
            row["cylinder_checks"].append(dict(part=name, label=label, radius_mm=radius,
                requested_span_z_mm=world_span, matching_finite_surfaces=surfaces,
                passed=merged_span_covers([s["axial_span_mm"] for s in surfaces], span)))
        lo, hi = map(float, check["shaft_span_z_mm"])
        tool = cq.Solid.makeCylinder(float(check["shaft_radius_mm"]) - 1e-5, hi - lo,
                                     cq.Vector(origin[0], origin[1], lo), cq.Vector(0, 0, 1))
        for name in (check["chassis"], check["housing"], check["nut"], check["washer"]):
            volume = float(tool.intersect(parts[name], tol=BOOLEAN_TOL_MM).Volume())
            row["clearance_checks"].append(dict(part=name, material_in_shaft_envelope_mm3=volume,
                                                passed=volume <= 1e-4))
        nut_lo, nut_hi = map(float, check["nut_span_z_mm"])
        overlap = max(0., min(hi, nut_hi) - max(lo, nut_lo))
        tip = hi - nut_hi
        row.update(measured_nominal_shaft_nut_axial_overlap_mm=overlap,
                   measured_nominal_tip_beyond_nut_mm=tip, helical_thread_engagement_verified=False,
                   nominal_axial_lengths_match=bool(abs(overlap - check["nut_engagement_mm"]) <= AXIS_TOL_MM
                                                   and abs(tip - check["tip_beyond_nut_mm"]) <= AXIS_TOL_MM))
        nut_faces = planar_faces(parts[check["nut"]], [origin[0], origin[1], nut_lo], [0, 0, 1])
        housing_faces = planar_faces(parts[check["housing"]], [origin[0], origin[1], nut_lo], [0, 0, 1])
        nut_seat_area = (float(cq.Compound.makeCompound(nut_faces).intersect(cq.Compound.makeCompound(housing_faces),
                               tol=BOOLEAN_TOL_MM).Area()) if nut_faces and housing_faces else 0.)
        row.update(nut_lower_seat_contact_area_mm2=nut_seat_area,
                   nut_housing_minimum_distance_mm=float(parts[check["nut"]].distance(parts[check["housing"]])),
                   nut_seated_at_nominal_pose=nut_seat_area > .02,
                   preloaded_retained_connection_approved=False)
        # Measure the two actual planar washer seats. No nominal area assertion
        # is needed here; a positive area and aligned shaft are only geometric evidence.
        washer_box = bounds(parts[check["washer"]])
        for other, z, label in ((check["bolt"], washer_box[0, 2], "bolt_head_to_washer"),
                                (check["chassis"], washer_box[1, 2], "washer_to_floor")):
            a = planar_faces(parts[check["washer"]], [origin[0], origin[1], z], [0, 0, 1])
            b = planar_faces(parts[other], [origin[0], origin[1], z], [0, 0, 1])
            area = (float(cq.Compound.makeCompound(a).intersect(cq.Compound.makeCompound(b),
                           tol=BOOLEAN_TOL_MM).Area()) if a and b else 0.)
            row["seating_checks"].append(dict(label=label, parts=[check["washer"], other],
                                               contact_area_mm2=area, passed=area > .02))
        row["passed"] = bool(row["nominal_axial_lengths_match"] and all(
            c["passed"] for group in (row["cylinder_checks"], row["clearance_checks"], row["seating_checks"]) for c in group))
    except Exception as exc:
        row.update(passed=False, error=type(exc).__name__ + ": " + str(exc))
    return row


def assembly_translation_check(probe, parts):
    """Replay actual saved-part translations, without claiming a swept volume.

    The generator's continuous outer-hex envelope is a separate calculation.
    Its definition is retained for traceability; neither its implementation nor
    its reported zero volume is imported as evidence of these sample results.
    """
    row = dict(id=probe["id"], definition=probe, samples=[], passed=False,
               method="Independent saved STEP at declared discrete translations",
               continuous_path_independently_verified=False,
               generator_continuous_result_adopted=False,
               material_reporting_threshold_mm3=1e-4)
    try:
        positions = np.asarray(probe["translations_world_mm"], float)
        if (positions.ndim != 2 or positions.shape[1] != 3 or len(positions) < 2
                or not np.all(np.isfinite(positions))):
            raise ValueError("Expected at least two finite world-mm translations")
        if type(probe["sample_count"]) is not int or len(positions) != probe["sample_count"]:
            raise ValueError("Declared sample count differs from explicit positions")
        spacing = float(probe["sample_spacing_mm"])
        if (not math.isfinite(spacing) or spacing <= 0
                or not np.allclose(np.linalg.norm(np.diff(positions, axis=0), axis=1), spacing, atol=1e-6, rtol=0)):
            raise ValueError("Translation spacing differs from explicit positions")
        if not np.allclose(positions[-1], 0, atol=1e-8, rtol=0):
            raise ValueError("Insertion samples must end at the saved assembled position")
        names = probe["against_parts"]
        if not names or len(set(names)) != len(names) or probe["moving_part"] in names:
            raise ValueError("Expected distinct stationary counterparts")
        shape = parts[probe["moving_part"]]
        for index, translation in enumerate(positions):
            moved = shape.translate(tuple(translation))
            sample = dict(index=index, translation_world_mm=translation.tolist(), checks=[])
            for name in names:
                check = dict(against_part=name, passed=False)
                try:
                    common = moved.intersect(parts[name], tol=BOOLEAN_TOL_MM)
                    volume = float(common.Volume())
                    if not math.isfinite(volume) or volume < -1e-9:
                        raise ValueError("Invalid common volume")
                    check.update(material_intersection_mm3=volume, passed=volume <= 1e-4)
                except Exception as exc:
                    check["error"] = type(exc).__name__ + ": " + str(exc)
                sample["checks"].append(check)
            sample["passed"] = all(c["passed"] for c in sample["checks"])
            row["samples"].append(sample)
        volumes = [c["material_intersection_mm3"] for s in row["samples"] for c in s["checks"]
                   if "material_intersection_mm3" in c]
        row.update(sample_count=len(positions), actual_sample_spacing_mm=spacing,
                   max_material_intersection_mm3=max(volumes, default=None),
                   boolean_failure_count=sum("error" in c for s in row["samples"] for c in s["checks"]),
                   passed=all(s["passed"] for s in row["samples"]))
    except Exception as exc:
        row.update(passed=False, error=type(exc).__name__ + ": " + str(exc))
    return row


def audit_interfaces(metadata, parts):
    """Expected interface metadata is intentionally declarative, not a pass flag.

    Missing metadata remains unverified. The builder's geometric assertions are
    never adopted without querying the saved STEP. Contract adaptation lives here.
    """
    report = dict(metadata_present=bool(metadata), clearance_probes=[], cylinder_interfaces=[],
                  solid_connections=[], all_declared_checks_passed=False,
                  scope="Finite nominal geometry only; no fit, retention, load or fatigue approval")
    for probe in metadata.get("clearance_probes", []):
        row = dict(id=probe["id"], definition=probe, checks=[])
        try:
            direction = np.array(probe["direction"], float)
            direction /= np.linalg.norm(direction)
            tool = cq.Solid.makeCylinder(float(probe["radius_mm"]), float(probe["length_mm"]),
                                         cq.Vector(*probe["origin_mm"]), cq.Vector(*direction))
            for name in probe["against_parts"]:
                common = tool.intersect(parts[name], tol=BOOLEAN_TOL_MM)
                volume = float(common.Volume())
                row["checks"].append(dict(part=name, material_in_clearance_mm3=volume,
                                          passed=volume <= float(probe.get("max_material_mm3", 1e-4))))
            row["passed"] = bool(row["checks"]) and all(x["passed"] for x in row["checks"])
        except Exception as exc:
            row.update(passed=False, error=type(exc).__name__ + ": " + str(exc))
        report["clearance_probes"].append(row)
    for check in metadata.get("cylinder_interfaces", []):
        row = dict(id=check["id"], definition=check)
        try:
            surfaces = cylinder_surfaces(parts[check["part"]], check["axis_origin_mm"], check["axis_direction"])
            matching = [s for s in surfaces if abs(s["radius_mm"] - check["radius_mm"]) <= AXIS_TOL_MM]
            row.update(matching_finite_surfaces=matching,
                       passed=merged_span_covers([s["axial_span_mm"] for s in matching], check["span_mm"]))
        except Exception as exc:
            row.update(passed=False, error=type(exc).__name__ + ": " + str(exc))
        report["cylinder_interfaces"].append(row)
    for check in metadata.get("solid_connections", []):
        row = dict(id=check["id"], definition=check)
        try:
            names = check["part_names"]
            if len(names) == 1:
                shape = parts[names[0]]
                row.update(valid=bool(shape.isValid()), solid_count=len(shape.Solids()),
                           passed=bool(shape.isValid() and len(shape.Solids()) == 1))
            else:
                contacts = []
                for a, b in zip(names, names[1:]):
                    common = parts[a].intersect(parts[b], tol=BOOLEAN_TOL_MM)
                    contacts.append(dict(a=a, b=b, distance_mm=float(parts[a].distance(parts[b])),
                                         overlap_mm3=float(common.Volume())))
                row.update(contacts=contacts, passed=False,
                           unresolved="Contact alone cannot prove a retained mechanical connection")
        except Exception as exc:
            row.update(passed=False, error=type(exc).__name__ + ": " + str(exc))
        report["solid_connections"].append(row)
    report["seating_interfaces"] = [seating_check(check, parts) for check in metadata.get("seating_interfaces", [])]
    report["mount_interfaces"] = [mounting_check(check, parts) for check in metadata.get("mount_interfaces", [])]
    report["assembly_translation_probes"] = [assembly_translation_check(probe, parts)
        for probe in metadata.get("assembly_translation_probes", [])]
    groups = [report[k] for k in ("clearance_probes", "cylinder_interfaces", "solid_connections")]
    groups.extend(report[key] for key in ("seating_interfaces", "mount_interfaces", "assembly_translation_probes") if key in metadata)
    report["all_declared_checks_passed"] = all(groups) and all(r["passed"] for group in groups for r in group)
    report["all_mount_nuts_seated_at_nominal_pose"] = bool(report["mount_interfaces"]) and all(
        r.get("nut_seated_at_nominal_pose", False) for r in report["mount_interfaces"])
    report["full_root_load_path_approved"] = False
    return report


def source_comparison(baseline, cases):
    original = baseline["assembly_cases"]
    comparisons = []
    for name, before in original.items():
        if name in cases:
            after = cases[name]
            comparisons.append(dict(case=name, source_reported_all_pairs=before["all_pairs"],
                                    source_reported_different_bodies=before["different_bodies"],
                                    candidate_all_pairs=after["overlap_count"],
                                    candidate_different_bodies=after["different_body_overlap_count"]))
    return dict(source_geometry_commit=baseline["audited_geometry_commit"],
                source_verdict=baseline["verdict"], source_part_count=baseline["parts"],
                counts_from_committed_audit_not_source_reexecution=True,
                comparisons=comparisons,
                limitations="Changed component partition can change overlap counts without fixing a defect")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, default=HERE)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--poses", choices=("core", "path"), default="path",
                        help="core: open, fist and retained spread counterexample; path: 13 path samples plus counterexample")
    options = parser.parse_args()
    candidate = options.candidate.resolve()
    repository = candidate.parents[1]
    output = (options.output or candidate / "generated/assembly_review.json").resolve()
    report_path = output if output.suffix.lower() == ".json" else output / "report.json"
    scene_path = candidate / "generated/scene.json"
    design_path = candidate / "design.json"
    build_path = candidate / "generated/build_report.json"
    assembly_path = candidate / "generated/hand.step"
    baseline_path = repository / "cad/webcam_articulation_v1/generated/manufacturing_audit/readiness.json"
    report = dict(schema_version=1, completed=False, manufacture_approved=False,
                  performance_validated=False, scope="Saved hand STEP only; all parts; no pair exemptions",
                  thresholds=dict(overlap_mm3=OVERLAP_MM3, boolean_tolerance_mm=BOOLEAN_TOL_MM,
                                  bbox_comparison_mm=BBOX_TOL_MM, axis_tolerance_mm=AXIS_TOL_MM,
                                  volume_absolute_mm3=VOLUME_ABS_TOL_MM3, volume_relative=VOLUME_REL_TOL),
                  environment=dict(python=sys.version, platform=platform.platform(), numpy=np.__version__,
                                   cadquery=cq.__version__),
                  parts=[], mesh_topology=[], cases={}, errors=[],
                  limitations=["Declared sampled poses only, not continuous clearance.",
                               "All positive intersections above the declared numerical threshold are reported, including cosmetic mating parts.",
                               "Same-body intersections are rigid-transform invariant and reused from neutral; no pair is exempted.",
                               "No motor packaging, cable routing, strength, tolerances, pin retention or real latency approval.",
                               "Known source wall, attachment and incomplete-fastener issues remain unless separately corrected and verified."])
    inputs = set(); before = {}; write_allowed = True
    fixed_inputs = (scene_path, design_path, build_path, assembly_path, Path(__file__).resolve(), baseline_path)
    if report_path in fixed_inputs or report_path.suffix.lower() != ".json":
        raise ValueError("Report must be a separate JSON file")
    try:
        parsed_bytes = {p: p.read_bytes() for p in (scene_path, design_path, build_path)}
        scene, design, build = [json.loads(parsed_bytes[p].decode("utf-8-sig"))
                                for p in (scene_path, design_path, build_path)]
        inputs.update(fixed_inputs)
        for row in scene["hand"]:
            inputs.update((safe_relative(candidate, row["step"]), safe_relative(candidate, row["mesh"])))
        expected = {}
        for block in (scene, build):
            for relative, sha in block.get("source_sha256", {}).items():
                path = safe_relative(repository, relative)
                if path in expected and expected[path] != sha:
                    raise ValueError("Conflicting source manifest: " + str(path))
                expected[path] = sha; inputs.add(path)
            if block.get("inputs_unchanged") is not True:
                raise ValueError("Builder did not record a passing input guard")
        before = {str(p): digest(p) for p in sorted(inputs)}
        if any(before[str(p)] != hashlib.sha256(raw).hexdigest() for p, raw in parsed_bytes.items()):
            raise ValueError("Metadata changed during initial read")
        bad = [str(p) for p, sha in expected.items() if before[str(p)] != sha]
        report["source_manifest_check"] = dict(passed=not bad, files=len(expected), mismatches=bad)
        report["input_sha256"] = before
        if bad:
            raise ValueError("Saved output no longer matches its input sources")
        if report_path in inputs:
            write_allowed = False
            raise ValueError("Output must not replace an input")
        if write_allowed:
            write_json(report_path, report)
        parts = load_parts(scene, candidate, report)
        report["assembly_roundtrip"] = assembly_roundtrip(assembly_path, scene["hand"], parts)
        report["root_interfaces"] = audit_interfaces(build.get("root_module", {}), parts)
        poses = sample_poses(design)
        if options.poses == "core":
            poses = [p for p in poses if p["name"] in ("open", "fist", "index_spread_minus22_others_open")]
        report["pose_sampling"] = dict(joint_names=joint_names(), fist_path=design["fist_path"],
                                      mode=options.poses,
                                      rule="Five inclusive equally spaced fractions per path segment; deduplicate shared endpoints; linear angles",
                                      fist_samples=sum(p["kind"] == "fist_path" for p in poses),
                                      extra_diagnostic_samples=1, poses=poses)
        bodies = {row["name"]: row["body"] for row in scene["hand"]}
        repair_names = set(build.get("added_parts", []))
        cache = None
        for pose in poses:
            result, cache = screen_pose(parts, bodies, design, pose, cache)
            for row in result["exact_pair_results"]:
                row["involves_root_replacement"] = row["a"] in repair_names or row["b"] in repair_names
                row["involves_named_appearance_part"] = any(
                    token in name for token in ("dark_faceplate", "rounded_tip_cap", "fascia") for name in (row["a"], row["b"]))
            result["overlap_classification"] = dict(
                involving_root_replacement=sum(r["involves_root_replacement"] for r in result["overlaps"]),
                involving_named_appearance_part=sum(r["involves_named_appearance_part"] for r in result["overlaps"]),
                classes_can_overlap=True, pair_exemptions=[])
            report["cases"][pose["name"]] = result
            write_json(report_path, report)
            print(json.dumps(dict(pose=pose["name"], overlaps=result["overlap_count"],
                                  different_bodies=result["different_body_overlap_count"],
                                  boolean_failures=len(result["boolean_failures"]), seconds=result["seconds"])), flush=True)
        report["source_comparison"] = source_comparison(read_json(baseline_path), report["cases"])
        report["geometry_screen"] = dict(
            all_parts_valid_single_solid=all(p["valid"] and p["solid_count"] == 1 for p in report["parts"]),
            all_sampled_poses_no_reportable_overlap=all(c["no_reportable_overlap_and_no_boolean_failure"] for c in report["cases"].values()),
            boolean_failure_count=sum(len(c["boolean_failures"]) for c in report["cases"].values()),
            root_declared_geometry_checks_passed=report["root_interfaces"]["all_declared_checks_passed"])
        report["completed"] = True
    except Exception as exc:
        report["errors"].append(dict(type=type(exc).__name__, message=str(exc), traceback=traceback.format_exc()))
    finally:
        after = {}; changed = []
        for path, sha in before.items():
            try:
                after[path] = digest(path)
            except OSError as exc:
                after[path] = {"error": str(exc)}
            if after[path] != sha:
                changed.append(path)
        report["input_hash_guard"] = dict(passed=bool(before) and not changed,
                                          start=before, end=after, changed=changed)
        if changed:
            report["completed"] = False
        if write_allowed:
            write_json(report_path, report)
    print(json.dumps(dict(completed=report["completed"], report=str(report_path),
                          errors=report["errors"])), flush=True)
    return 0 if report["completed"] and report["input_hash_guard"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

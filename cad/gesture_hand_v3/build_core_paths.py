"""Copy three canonical gestures and forward paths into a bounded study catalog.

No CAD loading, collision test or dynamics is performed. Run only after the
updated poses.json is generated and frozen. A new output file is required.
The generator starts no child processes and opens no GUI.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys

for _key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_key, "1")
sys.dont_write_bytecode = True

import poses

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
GESTURE_IDS = ("open", "fist", "thumb_up")
PATH_IDS = ("open__fist", "open__thumb_up", "fist__thumb_up")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def repo_path(path):
    path = Path(path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError("Catalog sources must be inside the repository: " + str(path))
    return path


def relative(path):
    return repo_path(path).relative_to(ROOT).as_posix()


def select_catalog(catalog):
    """Deep-copy exact values; do not resample, clip or overwrite their status."""
    if catalog["joint_names"] != poses.NAMES or catalog.get("units") != "degrees":
        raise ValueError("Expected canonical twenty-axis degree catalog")
    gestures = {row["id"]: row for row in catalog["gestures"]}
    paths = {row["id"]: row for row in catalog["transitions"]}
    if len(gestures) != len(catalog["gestures"]) or len(paths) != len(catalog["transitions"]):
        raise ValueError("Duplicate gesture or path identity")
    current = gestures["thumb_up"]["angles_deg"]
    if current[16:] != [-28., 0., 0., 0.] or current[:4] != [8., 90., 109.97777233225946, 90.]:
        raise ValueError("Generate the updated straight-thumb/index candidate first")
    if current[4:16] != catalog["reference"]["authored_fist_deg"][4:16]:
        raise ValueError("Thumb-up unexpectedly changes middle/ring/little targets")
    for ident in GESTURE_IDS:
        poses.validate_angles(gestures[ident]["angles_deg"], catalog["kinematic_design"])
    for ident in PATH_IDS:
        path = paths[ident]
        source, target = ident.split("__")
        if (path["source"], path["target"]) != (source, target):
            raise ValueError("Path identity differs from its endpoints: " + ident)
        points = path["waypoints"]
        if len(points) < 2:
            raise ValueError("A path needs at least two waypoints")
        fractions = [float(p["fraction"]) for p in points]
        if (not all(math.isfinite(f) for f in fractions) or fractions[0] != 0. or fractions[-1] != 1.
                or any(a >= b for a, b in zip(fractions, fractions[1:]))):
            raise ValueError("Fractions must strictly increase from zero to one: " + ident)
        for point in points:
            poses.validate_angles(point["angles_deg"], catalog["kinematic_design"])
        if points[0]["angles_deg"] != gestures[source]["angles_deg"] or points[-1]["angles_deg"] != gestures[target]["angles_deg"]:
            raise ValueError("Named gesture and path endpoint differ: " + ident)
    fist = paths["open__fist"]["waypoints"]
    reference = catalog["reference"]["authored_fist_path"]
    if len(fist) != 33 or len(reference) != 33:
        raise ValueError("The complete authored/candidate 33-point fist paths must remain present")
    for candidate, original in zip(fist, reference):
        if candidate["fraction"] != original["fraction"] or candidate["angles_deg"][:16] != original["angles_deg"][:16]:
            raise ValueError("The fist path changed original fractions or four-finger angles")
    up_path = paths["fist__thumb_up"]
    if ([p["fraction"] for p in up_path["waypoints"]] != [0., .45, .65, 1.]
            or up_path["method"] != "thumb_clear_then_index_adjust_then_thumb_raise"):
        raise ValueError("Expected the explicit thumb-clear / index-adjust / thumb-raise path")
    first, lifted, adjusted, last = up_path["waypoints"]
    if (lifted["angles_deg"][:16] != first["angles_deg"][:16]
            or lifted["angles_deg"][16:] != [25., 55., 0., 0.]
            or adjusted["angles_deg"][:16] != last["angles_deg"][:16]
            or adjusted["angles_deg"][16:] != lifted["angles_deg"][16:]):
        raise ValueError("Intermediate index adjustment differs from the declared path")
    selected = copy.deepcopy(catalog)
    selected["gestures"] = [copy.deepcopy(gestures[name]) for name in GESTURE_IDS]
    selected["transitions"] = [copy.deepcopy(paths[name]) for name in PATH_IDS]
    return selected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", type=Path, default=HERE / "poses.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.path.lexists(args.output):
        parser.error("Use a new output file; existing evidence is never overwritten")
    source = repo_path(args.poses)
    generator = repo_path(__file__)
    definitions = repo_path(poses.__file__)
    hashes = {relative(p): sha(p) for p in (source, generator, definitions)}
    catalog = poses.load_catalog(source)
    for name, expected in catalog["reference"]["source_sha256"].items():
        path = repo_path(ROOT / name)
        actual = sha(path)
        if actual != expected:
            raise ValueError("Accepted reference changed: " + name)
        hashes[relative(path)] = actual
    selected = select_catalog(catalog)
    after = {name: sha(ROOT / name) for name in hashes}
    if hashes != after:
        raise ValueError("Source changed while constructing the core catalog")
    selected["study_scope"] = dict(
        gestures=list(GESTURE_IDS), forward_paths=list(PATH_IDS),
        reverse_paths_executed=False, collision_checked_by_generator=False,
        dynamics_executed=False, continuous_collision_guarantee=False,
        meaning="Three forward geometric paths only. This generator copies definitions, not collision or performance approval.",
        generator_source=relative(generator), generator_sha256=hashes[relative(generator)],
        pose_catalog_source=relative(source), pose_catalog_sha256=hashes[relative(source)],
        pose_definition_source=relative(definitions), pose_definition_sha256=hashes[relative(definitions)],
        source_sha256=hashes, inputs_unchanged_during_catalog_construction=True,
        waypoint_policy="Exact deep copies from poses.json; no resampling, clipping, retiming or changed candidate status")
    payload = (json.dumps(selected, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as stream:
        stream.write(payload)
    print(json.dumps(dict(output=str(output), gesture_count=3, forward_path_count=3,
                          waypoint_counts={p["id"]: len(p["waypoints"]) for p in selected["transitions"]},
                          output_sha256=sha(output), collision_test_executed=False)), flush=True)


if __name__ == "__main__":
    main()

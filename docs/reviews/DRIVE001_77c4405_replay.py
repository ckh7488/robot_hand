"""Exact-commit DRIVE001 replay. PREPARED ONLY; invoke after checkpoint_state.json.

Run with the existing hidden-job launcher and pythonw. Every subprocess disables
Windows console creation. Candidate inputs come only from the named Git commit;
this external review script and installed dependency runtimes are separately
identified. This is a same-agent fresh-copy reproduction, not an external audit.

The final passed flag means the declared reproduction checks passed. Preserved
CAD overlaps, unfinished attachment/routing, failed prior convergence and missing
tracking events are NOT converted into physical acceptance.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tarfile
import time
import traceback


WORK = Path(__file__).resolve().parent
DEFAULT_REPO = Path("D:/robot_hand")
GIT = "C:/Program Files/Git/cmd/git.exe"
BASELINE = "coupling1_small5_125us"
FLAGS = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
REPORT_REL = "sim/form_drive_v1/results/drive001"
PACK_REL = "cad/form_drive_v1/generated/packaging"
MASS_REL = "cad/form_drive_v1/generated/exterior_mass_inertia.json"
CONTRACT_REL = "cad/form_drive_v1/generated/dynamics_contract.json"
ROUTE_REL = "cad/form_drive_v1/generated/tendon_route_study.json"


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def within(root, relative):
    if (not isinstance(relative, str) or any(c in relative for c in "\\:*?[]\x00")
            or relative.startswith("-")):
        raise ValueError("Expected a portable relative path: "+repr(relative))
    name = PurePosixPath(relative)
    if (name.is_absolute() or not name.parts or ".." in name.parts
            or name.as_posix() != relative or any(p.rstrip(" .") != p for p in name.parts)):
        raise ValueError("Path escapes declared root: "+relative)
    result = (Path(root)/relative).resolve()
    if not result.is_relative_to(Path(root).resolve()):
        raise ValueError("Resolved path escapes declared root: "+relative)
    return result


def git_capture(repo, arguments):
    result = subprocess.run([GIT, *arguments], cwd=repo, capture_output=True,
                            creationflags=FLAGS, shell=False, check=False)
    if result.returncode:
        raise RuntimeError("git failed: "+result.stderr.decode("utf-8", errors="replace"))
    return result.stdout


def git_json(repo, commit, relative):
    return json.loads(git_capture(repo, ["show", commit+":"+relative]).decode("utf-8"))


def checkpoint_declarations(pointer, manifest, root):
    """Validate local pointer paths and the complete selected-file manifest."""
    path_fields = {"manifest", "candidate", "simulation", "baseline", "accepted_exterior",
                   "exterior_reference", "simulation_results"}
    suffixes = ("_path", "_paths", "_file", "_files", "_dir", "_dirs",
                "_directory", "_directories", "_report")
    validated = {}
    for key, value in pointer.items():
        if key not in path_fields and not key.endswith(suffixes):
            continue
        if value is None:
            continue  # A future post-commit report is explicitly unset.
        values = value if isinstance(value, list) else [value]
        for item in values:
            within(root, item)
        validated[key] = value
    if not isinstance(pointer.get("id"), str) or not pointer["id"] or pointer["id"] != manifest.get("id"):
        raise ValueError("Committed CHECKPOINT and manifest IDs differ or are absent")
    files = manifest.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("Committed manifest must declare a nonempty files mapping")
    if len({name.casefold() for name in files}) != len(files):
        raise ValueError("Manifest file paths alias on Windows")
    for name, record in files.items():
        within(root, name)
        if (not isinstance(record, dict) or type(record.get("bytes")) is not int or record["bytes"] < 0
                or not isinstance(record.get("sha256"), str)
                or not re.fullmatch(r"[0-9a-f]{64}", record["sha256"])):
            raise ValueError("Invalid manifest size/SHA256 record: "+name)
    return files, validated


def compare(expected, actual, *, atol=0., rtol=0., omit=()):
    """Exact structure/booleans/integers; declared finite float tolerance only."""
    differences = []
    numeric_count = 0
    max_abs = 0.
    max_at = None

    def fail(path, a, b):
        if len(differences) < 40:
            differences.append(dict(path=path, expected=a, actual=b))

    def visit(a, b, path):
        nonlocal numeric_count, max_abs, max_at
        if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
            if type(a) is not type(b) or a != b:
                fail(path, a, b)
        elif isinstance(a, (float, int)) and isinstance(b, (float, int)):
            numeric_count += 1
            if not math.isfinite(a) or not math.isfinite(b):
                fail(path, a, b)
                return
            delta = abs(a-b)
            if delta > max_abs:
                max_abs, max_at = delta, path
            if isinstance(a, int) and isinstance(b, int):
                good = a == b
            else:
                good = delta <= atol+rtol*abs(a)
            if not good:
                fail(path, a, b)
        elif isinstance(a, dict) and isinstance(b, dict):
            ka, kb = set(a)-set(omit), set(b)-set(omit)
            if ka != kb:
                fail(path+" keys", sorted(ka), sorted(kb))
            for key in sorted(ka & kb):
                visit(a[key], b[key], path+"."+key)
        elif isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b):
                fail(path+" length", len(a), len(b))
            for i, (x, y) in enumerate(zip(a, b)):
                visit(x, y, f"{path}[{i}]")
        elif type(a) is not type(b) or a != b:
            fail(path, a, b)

    visit(expected, actual, "$")
    return dict(passed=not differences, absolute_tolerance=atol, relative_tolerance=rtol,
                numeric_values_compared=numeric_count, maximum_absolute_difference=max_abs,
                maximum_difference_path=max_at, differences_first_40=differences)


def measured_shape(shape, assumed_mass):
    """Independently measure imported world STEP with exact, non-mesh OCC bounds."""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, g)
    volume = g.Mass()
    if volume <= 0:
        raise ValueError("Imported shape has no positive solid volume")
    centre, inertia = g.CentreOfMass(), g.MatrixOfInertia()
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, box, False, False)
    bounds = box.Get()
    return dict(valid=bool(shape.isValid()), solid_count=len(shape.Solids()), volume_mm3=volume,
                com_world_mm=[centre.X(), centre.Y(), centre.Z()],
                inertia_com_world_kg_m2=[[inertia.Value(i+1,j+1)*assumed_mass/volume*1e-6 for j in range(3)] for i in range(3)],
                bbox_mm=[list(bounds[:3]), list(bounds[3:])])


def check_saved_packaging(root, folder, layout):
    import cadquery as cq
    import numpy as np
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    rows = read_json(folder/"placements_mass_inertia.json")["parts"]
    scene = read_json(folder/"scene.json")
    geometry = read_json(folder/"geometry.json")
    modules = {m["name"]: m for m in layout["modules"]}
    by_name = {r["name"]: r for r in rows}
    scene_by_name = {r["name"]: r for r in scene["parts"]}
    if len(by_name) != len(rows) or len(scene_by_name) != len(scene["parts"]) or set(by_name) != set(scene_by_name):
        raise ValueError("Saved packaging names are duplicated/missing")
    sources = geometry["source_sha256"]
    if sources != scene["source_sha256"] or not geometry["input_start_end_hash_guard_passed"]:
        raise ValueError("Packaging start/end or scene/source hash contract differs")
    for name, digest in sources.items():
        if sha256(within(root, name)) != digest:
            raise ValueError("Stale committed/replayed packaging source: "+name)
    vendor_cache = {}
    details = []
    vendor_count = new_count = 0
    source_per_module = {name: [] for name in modules}
    for row in rows:
        if row.get("source"):
            vendor_count += 1
            source_per_module[row["module"]].append(row["source"])
            source = within(root, row["source"])
            if row["source"] not in sources:
                raise ValueError("Vendor placement source is not hash-bound")
            if row["source"] not in vendor_cache:
                vendor_cache[row["source"]] = cq.importers.importStep(str(source)).val()
            M = np.asarray(row["transform_world_mm"], dtype=float)
            if (M.shape != (4,4) or not np.all(np.isfinite(M))
                    or not np.allclose(M[3], [0,0,0,1], rtol=0, atol=1e-12)
                    or not np.allclose(M[:3,:3].T@M[:3,:3], np.eye(3), rtol=0, atol=1e-12)
                    or abs(np.linalg.det(M[:3,:3])-1) > 1e-12
                    or not np.array_equal(M, np.asarray(modules[row["module"]]["transform_world_mm"]))):
                raise ValueError("Invalid or layout-mismatched vendor rigid transform: "+row["name"])
            transform = gp_Trsf()
            transform.SetValues(*M[:3,:].reshape(-1).tolist())
            shape = cq.Shape.cast(BRepBuilderAPI_Transform(vendor_cache[row["source"]].wrapped, transform, True).Shape())
        else:
            new_count += 1
            native = within(folder, row["saved_world_step"])
            if sha256(native) != row["saved_step_sha256"]:
                raise ValueError("Saved STEP hash mismatch: "+row["name"])
            shape = cq.importers.importStep(str(native)).val()
        measured = measured_shape(shape, row["mass_kg"])
        checks = {
            "volume": compare(row["volume_mm3"], measured["volume_mm3"], atol=1e-5),
            "bounds": compare(row["bbox_mm"], measured["bbox_mm"], atol=1e-6),
            "com": compare(row["com_world_mm"], measured["com_world_mm"], atol=1e-6),
            "inertia": compare(row["inertia_com_world_kg_m2"], measured["inertia_com_world_kg_m2"], atol=1e-12),
        }
        checks["validity"] = dict(passed=measured["valid"] and measured["solid_count"] == row["solid_count"]
            and (row.get("source") is not None or measured["solid_count"] == 1))
        drawing = scene_by_name[row["name"]]
        mesh = within(folder, drawing["mesh"])
        checks["scene"] = dict(passed=sha256(mesh) == drawing["sha256_stl"]
            and drawing["mesh_units"] == "mm" and drawing["body"] == row["body"]
            and drawing["pos_mm"] == [0,0,0] and drawing["R"] == [[1,0,0],[0,1,0],[0,0,1]]
            and compare(row["bbox_mm"], drawing["bbox_mm"], atol=1e-9)["passed"])
        details.append(dict(name=row["name"], source=row.get("source"), checks=checks,
                            passed=all(item["passed"] for item in checks.values())))
    source_sets = list(source_per_module.values())
    if any(len(a) != 6 or len(set(a)) != 6 or set(a) != set(source_sets[0]) for a in source_sets):
        raise ValueError("Each of twenty modules must retain the same six vendor components")
    count_ok = (len(modules) == 20 and vendor_count == 120 and new_count == geometry["new_single_solid_parts"]
                and len(rows) == geometry["part_count"])
    return dict(passed=count_ok and all(d["passed"] for d in details),
                saved_new_world_step_count=new_count, transformed_vendor_component_count=vendor_count,
                verified_scene_mesh_hash_count=len(rows), verified_input_hash_count=len(sources), details=details,
                scope="Imported saved STEP and independently applied layout rigid transforms; declared mass assumptions retained; no thread fit or force-path validation")


def probe(configuration):
    """Runs after replay, in a separate hidden process with the fresh root cwd."""
    cfg = read_json(configuration)
    fresh, out = Path(cfg["fresh_root"]), Path(cfg["replay_root"])
    expected = Path(cfg["expected_root"])
    checks = {}
    old_mass, mass = read_json(expected/"mass.json"), read_json(fresh/MASS_REL)
    checks["exterior_part_count"] = dict(passed=len(mass["parts"]) == len(old_mass["parts"]) == 120,
                                          expected=120, actual=len(mass["parts"]))
    # The full scalar/tensor source rebuild is compared, not only its grand total.
    checks["exterior_mass_rebuild"] = compare(old_mass, mass, atol=1e-9, rtol=1e-10)
    old_contract, contract = read_json(expected/"contract.json"), read_json(out/"dynamics_contract.json")
    checks["contract_body_frames_mass_and_transmission"] = compare(old_contract, contract, atol=1e-10, rtol=1e-10)
    layout = read_json(fresh/"cad/form_drive_v1/layout.json")
    checks["committed_saved_step_and_vendor_placements"] = check_saved_packaging(fresh, fresh/PACK_REL, layout)
    checks["replayed_saved_step_and_vendor_placements"] = check_saved_packaging(fresh, out/"packaging", layout)
    old_placements = read_json(fresh/PACK_REL/"placements_mass_inertia.json")
    new_placements = read_json(out/"packaging/placements_mass_inertia.json")
    # STEP exporter metadata may change bytes; both individual digest contracts
    # were checked above, while world shape and physical properties are compared.
    checks["packaging_properties_reproduction"] = compare(old_placements, new_placements, atol=1e-7, rtol=1e-10,
                                                          omit=("saved_step_sha256",))
    old_geo, new_geo = read_json(fresh/PACK_REL/"geometry.json"), read_json(out/"packaging/geometry.json")
    checks["neutral_exact_geometry_reproduction"] = compare(old_geo, new_geo, atol=1e-5, rtol=1e-10)
    tests = read_json(out/"adapter_tests.json")
    tree = ast.parse((fresh/"sim/form_drive_v1/test_adapter.py").read_text(encoding="utf-8"))
    expected_tests = sum(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name.startswith("test_")
                         for cls in tree.body if isinstance(cls, ast.ClassDef) and cls.name == "AdapterTests" for n in cls.body)
    checks["adapter_tests"] = dict(passed=tests["passed"] and tests["tests_run"] == expected_tests
        and expected_tests >= 13 and tests["failures"] == tests["errors"] == tests["skipped"] == 0
        and tests["actual_contract_sha256"] == sha256(out/"dynamics_contract.json"),
        expected_from_committed_test_source=expected_tests, actual=tests)
    old_route, route = read_json(expected/"route.json"), read_json(out/"tendon_route_study.json")
    checks["route_geometry_reproduction"] = compare(old_route, route, atol=1e-8, rtol=1e-10, omit=("environment",))
    checks["route_arithmetic_and_scope"] = dict(passed=route["checks"]["arithmetic_checks_passed"]
        and not route["actual_complete_transmission_validated"] and not route["latency_evaluated"]
        and route["length_model"]["actual_motor_to_joint_jacobian"] is None
        and route["length_model"]["actual_total_motor_cable_length"] is None,
        sampled_poses=sum(p["samples"] for p in route["pose_sets"]), scope=route["checks"]["scope"])
    old_case = read_json(expected/"small5.json")
    case = read_json(out/"simulation"/(BASELINE+".json"))
    summary = read_json(out/"simulation/summary.json")
    checks["small5_baseline_identity_and_completion"] = dict(passed=
        case["label"] == old_case["label"] == BASELINE and case["scenario"] == old_case["scenario"] == "small5"
        and case["coupling"] == old_case["coupling"] == 1 and case["dt_s"] == old_case["dt_s"] == .000125
        and case["finite"] and old_case["finite"] and summary["inputs_unchanged"]
        and len(summary["cases"]) == 1 and not summary["hardware_100ms_proven"],
        quick_case_count=len(summary["cases"]), convergence_rerun=False)
    checks["small5_full_metrics_reproduction"] = compare(old_case["metrics"], case["metrics"], atol=1e-8, rtol=1e-10)
    checks["small5_parameters_reproduction"] = compare(read_json(expected/"parameters.json"), read_json(out/"simulation/parameters.json"))
    checks["small5_input_manifest_reproduction"] = compare(read_json(expected/"input_manifest.json"), read_json(out/"simulation/input_manifest.json"))
    metric_names = ["event_count", "valid_event_max_ms", "missing_events", "anticipatory_events", "raw_max_error_deg",
                    "unsettled_quality_holds", "simulated_100ms_event_budget_met", "max_tendon_tension_N"]
    result = dict(passed=all(c["passed"] for c in checks.values()), checks=checks,
        unresolved_mechanical_and_performance_findings=dict(
            neutral_geometry_clear=new_geo["new_parts_neutral_volume_clear"],
            neutral_unresolved_overlaps=new_geo["unresolved_overlaps"],
            neutral_boolean_failures=new_geo["boolean_failures"], exact_neutral_pairs=new_geo["exact_pairs"],
            complete_transmission=False, attachment_and_manufacturing_approved=False,
            small5_metrics={k:case["metrics"][k] for k in metric_names}, hardware_100ms_proven=False),
        scope="Same-agent exact-SHA fresh-copy reproduction. 120 exterior mass properties, 21-body/20-joint contract, all current adapter tests, four middle tendon-end routes, all neutral motor packaging pairs and one small5 125us baseline. Individual native parts and vendor transforms are checked; the combined assembly STEP ZIP and rendered images are not regenerated or reimported. No whole-arm dynamics, full17-case rerun, fresh convergence, continuous collision, attachment strength or real performance approval.")
    write_json(out/"comparison.json", result)
    print(json.dumps(dict(passed=result["passed"], output=str(out/"comparison.json"))), flush=True)
    return 0 if result["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--state", type=Path, default=WORK/"checkpoint_state.json")
    parser.add_argument("--fresh", type=Path, help="Must not already exist; default review_<SHA12> beside this script")
    parser.add_argument("--probe", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.probe:
        return probe(args.probe)
    repo = args.repo.resolve()
    state = read_json(args.state)
    commit = state["sha"]
    if not isinstance(commit, str) or not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("checkpoint_state.json must contain a complete lowercase commit SHA")
    resolved = git_capture(repo, ["rev-parse", "--verify", commit+"^{commit}"]).decode().strip()
    if resolved != commit:
        raise ValueError("State SHA did not resolve to the exact named commit")
    fresh = (args.fresh or WORK/("review_"+commit[:12])).resolve()
    if fresh.exists() or fresh.drive.upper() != "D:" or fresh == repo or fresh.is_relative_to(repo):
        raise ValueError("Fresh path must be an absent D: directory outside the live repository")
    fresh.mkdir(parents=True, exist_ok=False)
    out = fresh/"_review"
    out.mkdir()
    expected = out/"expected"
    expected.mkdir()
    progress = WORK/"fresh_review_progress.json"
    result_path = WORK/"postcommit_review.json"
    result = dict(target_sha=commit, fresh_root=str(fresh), passed=False,
        reviewer_script_sha256=sha256(__file__), reviewer_script_committed_with_candidate=False,
        candidate_source="git archive of exact target commit", working_tree_inputs_copied=False,
        same_agent_fresh_copy_review=True, subprocess_console_windows_disabled=os.name == "nt", steps=[])

    def stage(name, status, **details):
        item = dict(stage=name, status=status, **details)
        write_json(progress, dict(target_sha=commit, fresh_root=str(fresh), **item))
        print(json.dumps(item, ensure_ascii=False), flush=True)

    python = Path(sys.executable)
    if python.name.lower() == "python.exe" and python.with_name("pythonw.exe").exists():
        python = python.with_name("pythonw.exe")
    env = os.environ.copy()
    env.update(PYTHONPATH="D:/Codex-work/robot-hand-workspace/work/audit_deps;D:/Codex-work/robot-hand-cache/2026-10-24/cad_deps",
               PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
               NUMEXPR_NUM_THREADS="1", TEMP=str(out/"tmp"), TMP=str(out/"tmp"))
    (out/"tmp").mkdir()
    result["execution_environment"] = {k:env[k] for k in ["PYTHONPATH", "OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "TEMP", "TMP"]}
    result["python_executable"] = str(python)

    def run(label, argv):
        stage(label, "running")
        started = time.monotonic()
        stdout, stderr = out/(label+".stdout.log"), out/(label+".stderr.log")
        with stdout.open("wb") as so, stderr.open("wb") as se:
            child = subprocess.run([str(a) for a in argv], cwd=fresh, env=env, stdin=subprocess.DEVNULL,
                stdout=so, stderr=se, creationflags=FLAGS, shell=False, check=False)
        item = dict(name=label, exit_code=child.returncode, elapsed_s=time.monotonic()-started,
                    argv=[str(a) for a in argv], cwd=str(fresh), stdout=str(stdout), stderr=str(stderr))
        result["steps"].append(item)
        stage(label, "completed" if child.returncode == 0 else "failed", exit_code=child.returncode)
        if child.returncode:
            raise RuntimeError(label+" failed; inspect captured logs")

    try:
        # Dependency names are read from committed JSON, never from live files.
        scene = git_json(repo, commit, "cad/form_candidate_v1/generated/scene.json")
        geometry = git_json(repo, commit, PACK_REL+"/geometry.json")
        pointer = git_json(repo, commit, "CHECKPOINT.json")
        manifest_path = pointer["manifest"]
        within(fresh, manifest_path)  # Validate before passing it to Git.
        manifest = git_json(repo, commit, manifest_path)
        manifest_files, pointer_paths = checkpoint_declarations(pointer, manifest, fresh)
        paths = {"cad/form_candidate_v1", "cad/form_drive_v1", "sim/form_drive_v1",
                 "cad/modular_roots_v9/scene.json", "cad/modular_roots_v9/mass_screen.json",
                 "CHECKPOINT.json", manifest_path, *manifest_files}
        for name in geometry["source_sha256"]:
            if not name.startswith(("cad/form_candidate_v1/", "cad/form_drive_v1/", "sim/form_drive_v1/")):
                within(repo, name)
                paths.add(name)
        for arm in scene["arm_reference"]:
            if not re.fullmatch(r"[A-Za-z0-9_]+", arm["asset"]):
                raise ValueError("Unexpected arm asset identifier")
            paths.add("cad/modular_roots_v9/parts/"+arm["asset"]+".step")
            paths.add("cad/modular_roots_v9/meshes/"+arm["asset"]+".stl")
        # Directory selections already include their descendants. Avoid passing
        # hundreds of redundant file paths beyond Windows' command-line limit.
        paths = sorted(p for p in paths if not any(p != q and p.startswith(q+'/') for q in paths))
        raw_tree = git_capture(repo, ["ls-tree", "-r", "-z", "--full-tree", commit, "--", *paths])
        blobs = {}
        for entry in raw_tree.split(b"\0"):
            if not entry:
                continue
            metadata, path_bytes = entry.split(b"\t", 1)
            mode, kind, oid = metadata.decode().split()
            path = path_bytes.decode("utf-8")
            within(fresh, path)
            if kind != "blob" or mode not in ("100644", "100755"):
                raise ValueError("Archive may contain only regular Git files: "+path)
            blobs[path] = oid
        archive = out/"exact_commit.tar"
        stage("git_archive", "running", file_count=len(blobs), archive=str(archive))
        with archive.open("wb") as target:
            ar = subprocess.run([GIT,"archive","--format=tar",commit,*paths],cwd=repo,stdout=target,
                stderr=subprocess.PIPE,creationflags=FLAGS,shell=False,check=False)
        if ar.returncode:
            raise RuntimeError(ar.stderr.decode("utf-8", errors="replace"))
        stage("git_archive", "written", archive_bytes=archive.stat().st_size, file_count=len(blobs))
        if shutil.disk_usage(fresh).free < archive.stat().st_size+512*1024*1024:
            raise RuntimeError("Insufficient D: free space for extraction and bounded replay outputs")
        extracted = set()
        with tarfile.open(archive) as bundle:
            for member in bundle:
                destination = within(fresh, member.name.rstrip("/"))
                if member.isdir():
                    destination.mkdir(parents=True, exist_ok=True)
                    continue
                if not member.isfile() or member.name not in blobs or member.name in extracted:
                    raise ValueError("Unexpected, duplicate or linked archive entry: "+member.name)
                destination.parent.mkdir(parents=True, exist_ok=True)
                with bundle.extractfile(member) as source, destination.open("xb") as target:
                    shutil.copyfileobj(source, target)
                extracted.add(member.name)
                if len(extracted) % 100 == 0:
                    stage("archive_extract", "running", extracted_files=len(extracted), expected_files=len(blobs))
        if extracted != set(blobs):
            raise ValueError("Archive/file list mismatch, including possible export-ignore rules")
        inventory = {}
        for name, oid in blobs.items():
            path = within(fresh, name)
            git_hash = hashlib.sha1(("blob "+str(path.stat().st_size)+"\0").encode())
            with path.open("rb") as handle:
                for block in iter(lambda:handle.read(1024*1024),b""):
                    git_hash.update(block)
            if git_hash.hexdigest() != oid:
                raise ValueError("Archive bytes differ from committed Git blob: "+name)
            inventory[name] = dict(bytes=path.stat().st_size, sha256=sha256(path), git_blob=oid)
        write_json(out/"archive_inventory.json", inventory)
        stage("git_archive", "verified", file_count=len(inventory), bytes=sum(x["bytes"] for x in inventory.values()))
        result["archive"] = dict(file_count=len(inventory), bytes=sum(x["bytes"] for x in inventory.values()),
            sha256=sha256(archive), inventory_sha256=sha256(out/"archive_inventory.json"), git_blob_bytes_all_verified=True,
            selected_paths=paths)
        # The file table is checked before any generated artifact is replaced.
        # This covers every selected manifest entry, including frozen V9 assets.
        extracted_pointer = read_json(fresh/"CHECKPOINT.json")
        extracted_manifest = read_json(within(fresh, manifest_path))
        mismatches = []
        for name, declared in manifest_files.items():
            actual = inventory.get(name)
            if actual is None or actual["bytes"] != declared["bytes"] or actual["sha256"] != declared["sha256"]:
                mismatches.append(dict(path=name, declared=declared, extracted=actual))
        manifest_check = dict(passed=not mismatches and extracted_pointer == pointer and extracted_manifest == manifest,
            target_sha=commit, checkpoint_id=pointer["id"], pointer_path="CHECKPOINT.json",
            pointer_sha256=inventory["CHECKPOINT.json"]["sha256"], manifest_path=manifest_path,
            manifest_sha256=inventory[manifest_path]["sha256"], declared_scope=manifest.get("scope"),
            manifest_file_count=len(manifest_files), manifest_total_bytes=sum(r["bytes"] for r in manifest_files.values()),
            every_manifest_file_archived_and_git_blob_verified=not any(n not in inventory for n in manifest_files),
            pointer_relative_paths_validated=pointer_paths, mismatches=mismatches,
            scope="Every selected manifest file's exact committed bytes and SHA256 checked after extraction and before replay; hash integrity is not physical validation")
        result["checkpoint_manifest_verification"] = manifest_check
        write_json(out/"checkpoint_manifest_verification.json", manifest_check)
        if not manifest_check["passed"]:
            raise ValueError("Committed CHECKPOINT/manifest or selected file size/SHA256 verification failed")
        stage("checkpoint_manifest", "verified", checkpoint_id=pointer["id"], files=len(manifest_files))
        snapshots = {"mass.json":MASS_REL, "contract.json":CONTRACT_REL, "route.json":ROUTE_REL,
            "small5.json":REPORT_REL+"/"+BASELINE+".json", "parameters.json":REPORT_REL+"/parameters.json",
            "input_manifest.json":REPORT_REL+"/input_manifest.json"}
        for name, relative in snapshots.items():
            shutil.copyfile(within(fresh, relative), expected/name)
        # Every executable candidate source is now read from the archive root.
        run("exterior_mass", [python, fresh/"cad/form_drive_v1/exterior.py"])
        run("dynamics_contract", [python, fresh/"cad/form_drive_v1/build_contract.py", "--output", out/"dynamics_contract.json"])
        run("adapter_tests", [python, fresh/"sim/form_drive_v1/test_adapter.py", "--reference-root",
            fresh/"sim/form_drive_v1/reference_06b9ab4", "--contract", out/"dynamics_contract.json", "--output", out/"adapter_tests.json"])
        run("tendon_route", [python, fresh/"cad/form_drive_v1/tendon_route_study.py", "--output", out/"tendon_route_study.json"])
        run("packaging_exact_neutral", [python, fresh/"cad/form_drive_v1/actuator_packaging.py", "--output", out/"packaging"])
        run("small5_quick", [python, fresh/"sim/form_drive_v1/run_screen.py", "--quick", "--output", out/"simulation", "--traces", out/"traces"])
        configuration = out/"review_config.json"
        write_json(configuration, dict(target_sha=commit, fresh_root=str(fresh), replay_root=str(out), expected_root=str(expected)))
        run("independent_comparison", [python, Path(__file__).resolve(), "--probe", configuration])
        comparison = read_json(out/"comparison.json")
        # Only this declared regenerated artifact may differ in the archive tree.
        changes = [name for name, row in inventory.items() if sha256(within(fresh,name)) != row["sha256"]]
        unexpected_changes = [name for name in changes if name != MASS_REL]
        if unexpected_changes:
            raise ValueError("Replay mutated protected committed inputs: "+repr(unexpected_changes))
        result.update(comparison=comparison, regenerated_original_artifacts=changes,
                      protected_archive_inputs_unchanged=True, passed=comparison["passed"])
        result["evidence_files"] = {p.relative_to(fresh).as_posix():dict(sha256=sha256(p),bytes=p.stat().st_size)
            for p in sorted(out.rglob("*")) if p.is_file() and (p.suffix == ".json" or p.name.endswith(".log"))}
        stage("complete", "passed" if result["passed"] else "failed", physical_acceptance=False)
    except Exception as exc:
        result["error"] = type(exc).__name__+": "+str(exc)
        result["traceback"] = traceback.format_exc()
        comparison_file = out/"comparison.json"
        if comparison_file.exists():
            result["comparison"] = read_json(comparison_file)
        stage("review", "failed", error=result["error"])
    finally:
        result["scope"] = "Exact-commit reproduction only; known collisions and tracking/convergence limitations remain limitations. No git mutation, push, hardware action, whole 17-case or continuous-clearance rerun."
        write_json(result_path, result)
        print(json.dumps(dict(target_sha=commit,passed=result["passed"],report=str(result_path))),flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

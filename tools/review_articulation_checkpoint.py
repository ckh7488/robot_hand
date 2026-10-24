"""Rebuild a named articulation checkpoint in a new, isolated D: directory.

Run this tool through pythonw/the hidden job launcher. It never changes Git or
the source checkout. All tracked input files come from the exact commit archive.
This is a fresh-copy reproduction, not an external or manufacturing approval.
Four physical-dynamics runs and full 13-pose replay are deliberately excluded.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
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

CAD = "cad/articulation_drive_v2"
SIM = "sim/articulation_drive_v2"
CORE_CASES = ("open", "fist", "index_spread_minus22_others_open")
CREATE_FLAGS = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
DEFAULT_PYTHON = "C:/Users/c/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe"
DEFAULT_PYTHONPATH = ("D:/Codex-work/robot-hand-workspace/work/audit_deps;"
                      "D:/Codex-work/robot-hand-cache/2026-10-24/cad_deps")


def utc():
    return datetime.now(timezone.utc).isoformat()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def digest(path, algorithm="sha256", prefix=b""):
    h = hashlib.new(algorithm); h.update(prefix)
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def within(root, relative):
    if not isinstance(relative, str) or any(c in relative for c in "\\:\x00*?[]") or relative.startswith("-"):
        raise ValueError("Expected portable repository-relative path: " + repr(relative))
    name = PurePosixPath(relative)
    if (not name.parts or name.is_absolute() or ".." in name.parts or name.as_posix() != relative
            or any(p.rstrip(" .") != p for p in name.parts)):
        raise ValueError("Invalid relative path: " + relative)
    target = (Path(root) / relative).resolve()
    if not target.is_relative_to(Path(root).resolve()):
        raise ValueError("Path escapes declared root: " + relative)
    return target


def d_path(value, label):
    path = Path(value).resolve()
    if os.name != "nt" or path.drive.casefold() != "d:":
        raise ValueError(label + " must resolve to D: on Windows")
    return path


def compare(expected, actual, *, atol=0., rtol=0., omit=()):
    """Exact structure/integers/booleans, predeclared finite-float tolerance."""
    differences = []; numeric_count = 0; max_abs = 0.; max_at = None
    def fail(path, a, b):
        if len(differences) < 40:
            differences.append(dict(path=path, expected=a, actual=b))
    def visit(a, b, path):
        nonlocal numeric_count, max_abs, max_at
        if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
            if type(a) is not type(b) or a != b: fail(path, a, b)
        elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
            numeric_count += 1
            if not math.isfinite(a) or not math.isfinite(b):
                fail(path, a, b); return
            delta = abs(a - b)
            if delta > max_abs: max_abs, max_at = delta, path
            if (a != b if isinstance(a, int) and isinstance(b, int) else delta > atol + rtol * abs(a)):
                fail(path, a, b)
        elif isinstance(a, dict) and isinstance(b, dict):
            ka, kb = set(a) - set(omit), set(b) - set(omit)
            if ka != kb: fail(path + ".keys", sorted(ka), sorted(kb))
            for key in sorted(ka & kb): visit(a[key], b[key], path + "." + key)
        elif isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b): fail(path + ".length", len(a), len(b))
            for i, (left, right) in enumerate(zip(a, b)): visit(left, right, path + f"[{i}]")
        elif type(a) is not type(b) or a != b:
            fail(path, a, b)
    visit(expected, actual, "$")
    return dict(passed=not differences, atol=atol, rtol=rtol, numeric_values_compared=numeric_count,
                max_absolute_difference=max_abs, max_difference_at=max_at, differences=differences)


def named_rows(rows, key="name"):
    result = {row[key]: row for row in rows}
    if len(result) != len(rows): raise ValueError("Duplicate named records")
    return result


def verify_manifest(repo, pointer):
    for key in ("candidate", "simulation", "exterior_reference", "simulation_results", "manifest"):
        within(repo, pointer[key])
    for key, value in pointer.items():
        if value is not None and key.endswith(("_path", "_report", "_directory", "_file")):
            within(repo, value)
    if pointer["candidate"] != CAD or pointer["simulation"] != SIM:
        raise ValueError("Checkpoint does not select articulation_drive_v2")
    path = within(repo, pointer["manifest"])
    manifest = read_json(path)
    if manifest.get("id") != pointer.get("id"):
        raise ValueError("Pointer and manifest IDs differ")
    files = manifest["files"]
    if not files or len({name.casefold() for name in files}) != len(files):
        raise ValueError("Empty manifest or Windows path alias")
    failures = []; total = 0
    for name, entry in files.items():
        file = within(repo, name)
        if (type(entry.get("bytes")) is not int or entry["bytes"] < 0
                or not re.fullmatch(r"[0-9a-f]{64}", entry.get("sha256", ""))):
            raise ValueError("Invalid manifest record: " + name)
        total += entry["bytes"]
        if not file.is_file() or file.stat().st_size != entry["bytes"] or digest(file) != entry["sha256"]:
            failures.append(name)
    return dict(passed=not failures, id=manifest["id"], manifest_path=pointer["manifest"],
                manifest_sha256=digest(path), file_count=len(files), bytes=total, failures=failures), files


def verify_frozen(repo):
    index = read_json(repo / "AUDIT_EVIDENCE_INDEX.json")
    files = {}; declarations = 0
    for group in ("geometry_sha256", "code_sha256", "report_sha256"):
        for name, sha in index[group].items():
            declarations += 1
            if name in files and files[name] != sha: raise ValueError("Conflicting frozen hash")
            files[name] = sha
    failures = [name for name, sha in files.items() if digest(within(repo, name)) != sha]
    return dict(passed=declarations == 222 and not failures, declarations=declarations,
                unique_files=len(files), index_sha256=digest(repo / "AUDIT_EVIDENCE_INDEX.json"), failures=failures), files


class Review:
    def __init__(self, options):
        self.options = options
        self.source = d_path(options.repo, "Source repository")
        self.work = d_path(options.work_root, "Fresh review directory")
        if self.work.exists():
            raise FileExistsError("Refusing existing review directory; no files are deleted: " + str(self.work))
        if self.work == self.source or self.work.is_relative_to(self.source):
            raise ValueError("Review directory must be outside source repository")
        self.work.mkdir(parents=True, exist_ok=False)
        self.repo = self.work / "repository"
        self.logs = self.work / "logs"; self.logs.mkdir()
        self.evidence = self.work / "evidence"; self.evidence.mkdir()
        self.baseline = self.work / "baseline"; self.baseline.mkdir()
        self.temp = self.work / "temp"; self.temp.mkdir()
        self.env = os.environ.copy()
        self.env.update(PYTHONPATH=options.pythonpath, PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
                        OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
                        NUMEXPR_NUM_THREADS="1", TEMP=str(self.temp), TMP=str(self.temp))
        self.report = dict(schema_version=1, target_sha=options.sha, started_utc=utc(), completed=False,
            reproduction_passed=False, hardware_approved=False, manufacturing_approved=False,
            review_kind="Same-agent fresh-copy reproduction, not external audit", steps=[], checks={}, errors=[],
            scope=dict(rebuilt=["Saved source hand to complete new CAD", "Exact part mass and full inertia tensors",
                               "Three all-part core poses and finite root/mount geometry", "New 20-axis dynamics contract",
                               "Adapter/compiled-model tests and command/metric protocol tests"],
                       not_reexecuted=["Four dynamics cases", "Timestep-convergence physics runs", "13-pose full path",
                                       "Motor packaging screen", "Continuous assembly or motion", "Hardware latency/strength"]),
            runtime=dict(python=sys.version, executable=sys.executable, child_python=options.python,
                         pythonpath=options.pythonpath, subprocess_console_hidden=True,
                         blas_openmp_threads=1, temp_directory=str(self.temp)),
            runner=dict(path=str(Path(__file__).resolve()), sha256=digest(__file__)))
        self.guard = {}; self.frozen = {}
        self.flush()

    def flush(self):
        save(self.work / "review.json", self.report)

    def run(self, label, argv, *, cwd=None):
        cwd = d_path(cwd or self.repo, "Subprocess cwd")
        out = self.logs / (label + ".stdout.log")
        err = self.logs / (label + ".stderr.log")
        item = dict(label=label, argv=[str(a) for a in argv], cwd=str(cwd), started_utc=utc(),
                    stdout=str(out.relative_to(self.work)), stderr=str(err.relative_to(self.work)), status="running")
        self.report["steps"].append(item); self.flush(); started = time.monotonic()
        print(label + " started", flush=True)
        try:
            with out.open("wb") as stdout, err.open("wb") as stderr:
                completed = subprocess.run(item["argv"], cwd=cwd, env=self.env, stdin=subprocess.DEVNULL,
                    stdout=stdout, stderr=stderr, shell=False, creationflags=CREATE_FLAGS,
                    timeout=self.options.timeout_seconds, check=False)
            item.update(returncode=completed.returncode, status="completed" if completed.returncode == 0 else "failed")
        except subprocess.TimeoutExpired:
            item.update(returncode=None, status="timeout")
            raise
        except BaseException as exc:
            item.update(returncode=None, status="launch_or_wait_failed", exception=type(exc).__name__ + ": " + str(exc))
            raise
        finally:
            item.update(ended_utc=utc(), elapsed_s=time.monotonic() - started)
            self.flush()
        print(label + " " + item["status"], flush=True)
        if completed.returncode:
            raise RuntimeError(f"{label} failed with code {completed.returncode}; see {err}")
        return out

    def require(self, name, value):
        self.report["checks"][name] = value; self.flush()
        if not value.get("passed", False):
            raise ValueError("Reproduction check failed: " + name)

    def archive(self):
        sha = self.options.sha
        resolved = self.run("git_resolve_commit", [self.options.git, "rev-parse", "--verify", sha + "^{commit}"], cwd=self.source).read_text().strip()
        if resolved != sha: raise ValueError("Requested SHA is not the exact commit object")
        tree_file = self.run("git_tree", [self.options.git, "ls-tree", "-r", "-l", "-z", "--full-tree", sha], cwd=self.source)
        tree = {}
        for entry in tree_file.read_bytes().split(b"\0"):
            if not entry: continue
            left, encoded = entry.split(b"\t", 1); mode, kind, blob, size = left.decode("ascii").split()
            name = encoded.decode("utf-8"); within(self.work, name)
            if kind != "blob" or mode not in ("100644", "100755"):
                raise ValueError("Unsupported link/submodule archive member: " + name)
            tree[name] = dict(mode=mode, blob_sha1=blob, git_blob_bytes=int(size))
        if len(tree) != len({n.casefold() for n in tree}): raise ValueError("Windows tree aliases")
        required_bytes = math.ceil(sum(row["git_blob_bytes"] for row in tree.values()) * 2.5) + 512 * 1024 * 1024
        free_bytes = shutil.disk_usage(self.work).free
        self.require("disk_headroom", dict(passed=free_bytes >= required_bytes,
            available_bytes=free_bytes, required_bytes=required_bytes,
            rule="2.5 times archived tracked bytes plus 512 MiB for fresh CAD and logs"))
        archive = self.work / "input_commit.tar"
        self.run("git_archive", [self.options.git, "archive", "--format=tar", "--output=" + str(archive), sha], cwd=self.source)
        self.repo.mkdir(); seen = set(); byte_count = 0
        with tarfile.open(archive, "r:") as tar:
            for member in tar:
                name = member.name.rstrip("/")
                if not name: continue
                dest = within(self.repo, name)
                if member.isdir():
                    dest.mkdir(parents=True, exist_ok=True); continue
                if not member.isfile() or name not in tree or name in seen:
                    raise ValueError("Unexpected archive member: " + member.name)
                seen.add(name); dest.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(member) as src, dest.open("xb") as dst:
                    shutil.copyfileobj(src, dst, 1024 * 1024)
                if dest.stat().st_size != member.size: raise ValueError("Truncated member: " + name)
                if member.size != tree[name]["git_blob_bytes"]: raise ValueError("Archive/blob length differs: " + name)
                actual = digest(dest, "sha1", f"blob {member.size}\0".encode("ascii"))
                if actual != tree[name]["blob_sha1"]: raise ValueError("Archive differs from Git blob: " + name)
                tree[name].update(bytes=member.size, sha256=digest(dest)); byte_count += member.size
                if len(seen) % 500 == 0: print(f"archive verified {len(seen)}/{len(tree)} files", flush=True)
        if seen != set(tree): raise ValueError("Archive omitted tracked files")
        save(self.evidence / "git_tree_verified.json", tree)
        self.require("archive", dict(passed=True, commit=sha, files=len(tree), bytes=byte_count,
            archive_sha256=digest(archive), all_files_verified_against_git_blob=True,
            note="No checkout, hooks, untracked files, working-tree edits or source checkout outputs are used"))
        # Only this candidate's generated outputs and generated design may change.
        self.guard = {name: row["sha256"] for name, row in tree.items()
                      if not name.startswith(CAD + "/generated/") and name != CAD + "/design.json"}
        return tree

    def preserve(self, relative):
        source = within(self.repo, relative); target = within(self.baseline, relative)
        if not source.is_file(): raise FileNotFoundError("Missing committed baseline: " + relative)
        target.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(source, target)
        if digest(source) != digest(target): raise ValueError("Baseline copy differs")
        return dict(path=relative, bytes=source.stat().st_size, sha256=digest(source))

    def execute(self):
        tree = self.archive()
        pointer = read_json(self.repo / "CHECKPOINT.json")
        manifest, files = verify_manifest(self.repo, pointer); self.require("manifest", manifest)
        if not set(files).issubset(tree): raise ValueError("Manifest includes untracked inputs")
        frozen, self.frozen = verify_frozen(self.repo); self.require("frozen_222_before", frozen)
        if not set(self.frozen).issubset(files):
            raise ValueError("Checkpoint manifest does not cover every frozen evidence input")
        execution_inputs = ("tools/review_articulation_checkpoint.py", CAD + "/build.py", CAD + "/root_module.py",
            CAD + "/verify.py", "cad/form_drive_v1/exterior.py", SIM + "/model_adapter.py",
            SIM + "/build_contract.py", SIM + "/test_adapter.py", SIM + "/test_screen.py", SIM + "/run_screen.py",
            SIM + "/scope.json", SIM + "/transmission.json", "sim/form_drive_v1/model_adapter.py",
            SIM + "/reference_webcam_31ba43f/provenance.json",
            SIM + "/reference_webcam_31ba43f/hand_follow/cad_hand.py",
            SIM + "/reference_webcam_31ba43f/hand_follow/retarget.py",
            SIM + "/reference_webcam_31ba43f/hand_follow/__init__.py")
        uncovered = [name for name in execution_inputs if name not in files]
        self.require("executed_source_manifest_coverage", dict(passed=not uncovered, uncovered=uncovered,
                                                               required_paths=list(execution_inputs)))
        committed_runner = within(self.repo, "tools/review_articulation_checkpoint.py")
        self.require("runner_matches_checkpoint", dict(passed=digest(__file__) == digest(committed_runner),
            actual_sha256=digest(__file__), committed_sha256=digest(committed_runner)))
        required = ("CHECKPOINT.json", pointer["manifest"], CAD + "/design.json", CAD + "/generated/scene.json",
                    CAD + "/generated/mass_inertia.json", CAD + "/generated/build_report.json",
                    CAD + "/generated/assembly_review.json", CAD + "/generated/dynamics_contract.json",
                    pointer["simulation_results"] + "/summary.json")
        self.report["preserved_baseline"] = [self.preserve(name) for name in required]; self.flush()
        baseline_report = read_json(self.baseline / CAD / "generated/assembly_review.json")
        if not baseline_report.get("completed") or not baseline_report.get("input_hash_guard", {}).get("passed"):
            raise ValueError("Committed CAD verification is incomplete/stale")
        if not set(CORE_CASES).issubset(baseline_report["cases"]): raise ValueError("Baseline lacks core cases")
        baseline_summary = read_json(self.baseline / pointer["simulation_results"] / "summary.json")
        self.report["preserved_dynamics_not_reexecuted"] = dict(
            source=pointer["simulation_results"] + "/summary.json", cases=baseline_summary.get("cases"),
            convergence=baseline_summary.get("convergence"), numerical_results_reproduced_by_this_run=False)
        py = self.options.python
        self.run("rebuild_cad", [py, str(self.repo / CAD / "build.py")])
        generated = self.repo / CAD / "generated"
        build = read_json(generated / "build_report.json")
        self.require("cad_build_guard", dict(passed=build.get("inputs_unchanged") is True,
                                              part_count=build.get("part_count")))
        self.compare_mass()
        scene_before = read_json(self.baseline / CAD / "generated/scene.json")
        scene_after = read_json(generated / "scene.json")
        self.require("scene_geometry", compare(scene_before, scene_after, atol=1e-6, rtol=1e-9,
                                                omit=("sha256_step", "sha256_stl")))
        self.require("generated_design", compare(read_json(self.baseline / CAD / "design.json"), read_json(self.repo / CAD / "design.json")))
        core_path = self.evidence / "fresh_core_geometry.json"
        self.run("verify_core_geometry", [py, str(self.repo / CAD / "verify.py"), "--poses", "core", "--output", str(core_path)])
        self.compare_core(baseline_report, read_json(core_path))
        contract_path = generated / "dynamics_contract.json"
        self.run("regenerate_contract", [py, str(self.repo / SIM / "build_contract.py"),
            "--cad-root", str(self.repo / CAD), "--step", str(generated / "hand.step"),
            "--transmission", str(self.repo / SIM / "transmission.json"),
            "--scope", str(self.repo / SIM / "scope.json"), "--repo-root", str(self.repo), "--output", str(contract_path)])
        self.compare_contract()
        adapter_output = self.evidence / "adapter_tests.json"
        self.run("adapter_tests", [py, str(self.repo / SIM / "test_adapter.py"),
            "--reference-root", str(self.repo / "sim/form_drive_v1/reference_06b9ab4"),
            "--contract", str(contract_path), "--output", str(adapter_output)])
        adapter = read_json(adapter_output)
        self.require("adapter_tests", dict(passed=adapter.get("passed") is True and adapter.get("skipped") == 0
            and adapter.get("failures") == 0 and adapter.get("errors") == 0,
            tests_run=adapter.get("tests_run"), skipped=adapter.get("skipped"), evidence=adapter.get("evidence")))
        protocol_output = self.evidence / "protocol_tests.json"
        self.run("protocol_tests", [py, str(self.repo / SIM / "test_screen.py"), "--output", str(protocol_output)])
        protocol = read_json(protocol_output)
        self.require("protocol_tests", dict(passed=protocol.get("passed") is True and protocol.get("failures") == 0
            and protocol.get("errors") == 0, tests_run=protocol.get("tests_run")))
        self.report["completed"] = True

    def compare_mass(self):
        old = read_json(self.baseline / CAD / "generated/mass_inertia.json")
        new = read_json(self.repo / CAD / "generated/mass_inertia.json")
        a, b = named_rows(old["parts"]), named_rows(new["parts"])
        if set(a) != set(b): raise ValueError("Changed mass part identities")
        checks = {}
        groups = (("mass_kg", 1e-12, 1e-9), ("com_world_mm", 1e-7, 1e-10),
                  ("inertia_com_world_kg_m2", 1e-13, 1e-8), ("volume_mm3", 1e-5, 1e-9), ("bbox_mm", 1e-6, 1e-10))
        for key, atol, rtol in groups:
            checks[key] = compare({n:r[key] for n,r in a.items()}, {n:r[key] for n,r in b.items()}, atol=atol, rtol=rtol)
        residual_omit = ("source_sha256", *[g[0] for g in groups])
        checks["remaining_fields"] = compare(old, new, atol=1e-12, rtol=1e-9, omit=residual_omit)
        self.require("exact_cad_mass_tensors", dict(passed=all(c["passed"] for c in checks.values()),
            part_count=len(a), comparisons=checks, note="Exact BRep mass-property method, declared numerical tolerance; STEP bytes/timestamps need not reproduce"))

    def compare_core(self, old, new):
        if not new.get("completed") or not new.get("input_hash_guard", {}).get("passed"):
            raise ValueError("Fresh core verification incomplete/stale")
        checks = {}
        checks["parts"] = compare(named_rows(old["parts"]), named_rows(new["parts"]), atol=1e-5, rtol=1e-9)
        checks["mesh_topology"] = compare(named_rows(old["mesh_topology"]), named_rows(new["mesh_topology"]))
        checks["assembly_roundtrip"] = compare(old["assembly_roundtrip"], new["assembly_roundtrip"], atol=1e-5, rtol=1e-9)
        checks["finite_root_and_mount_geometry"] = compare(old["root_interfaces"], new["root_interfaces"],
            atol=1e-5, rtol=1e-8, omit=("face_index",))
        for name in CORE_CASES:
            # Every pair identity, overlap flag, error and threshold remains in
            # comparison; timing/cache counts alone are execution details.
            checks[name] = compare(old["cases"][name], new["cases"][name], atol=1e-5, rtol=1e-8,
                omit=("seconds", "exact_operations_this_pose", "same_body_results_reused", "reused_rigid_body_invariant"))
        self.require("core_geometry_reproduction", dict(passed=all(c["passed"] for c in checks.values()),
            comparisons=checks, preserved_failures={name:dict(overlaps=new["cases"][name]["overlap_count"],
                different_body_overlaps=new["cases"][name]["different_body_overlap_count"],
                boolean_failures=len(new["cases"][name]["boolean_failures"])) for name in CORE_CASES},
            note="Reproducing unresolved overlaps is not a clearance or fabrication pass"))

    def compare_contract(self):
        old = read_json(self.baseline / CAD / "generated/dynamics_contract.json")
        new = read_json(self.repo / CAD / "generated/dynamics_contract.json")
        checks = {}
        a, b = named_rows(old["bodies"]), named_rows(new["bodies"])
        for key, atol, rtol in (("mass_kg",1e-12,1e-9), ("com_local_mm",1e-7,1e-10),
                               ("inertia_com_local_kg_m2",1e-13,1e-8)):
            checks[key] = compare({n:r[key] for n,r in a.items()}, {n:r[key] for n,r in b.items()}, atol=atol, rtol=rtol)
        checks["frames_joints_routing_scope"] = compare(old,new,atol=1e-9,rtol=1e-10,
            omit=("source_sha256","mass_kg","com_local_mm","inertia_com_local_kg_m2"))
        self.require("new_contract_reproduction", dict(passed=all(c["passed"] for c in checks.values()),
            bodies=len(a), joints=len(new["joints"]), comparisons=checks,
            baseline_contract_sha256=digest(self.baseline / CAD / "generated/dynamics_contract.json"),
            fresh_contract_sha256=digest(self.repo / CAD / "generated/dynamics_contract.json"),
            hash_difference_expected_for_regenerated_native_STEP=True))

    def finish(self):
        changed = []
        for name, sha in self.guard.items():
            try:
                if digest(within(self.repo, name)) != sha: changed.append(name)
            except OSError: changed.append(name)
        self.report["source_end_guard"] = dict(passed=bool(self.guard) and not changed,
            files=len(self.guard), changed=changed,
            permitted_output_scope=[CAD + "/generated/", CAD + "/design.json"],
            note="All other tracked bytes, including source webcam, sim_3rd and frozen evidence, remain unchanged")
        if self.frozen:
            try:
                frozen, _ = verify_frozen(self.repo); self.report["checks"]["frozen_222_after"] = frozen
            except Exception as exc:
                self.report["checks"]["frozen_222_after"] = dict(passed=False,error=str(exc))
        self.report["ended_utc"] = utc()
        self.report["reproduction_passed"] = bool(self.report["completed"] and not self.report["errors"]
            and self.report["source_end_guard"]["passed"] and all(c["passed"] for c in self.report["checks"].values()))
        self.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sha", required=True, help="Exact full 40-character commit SHA")
    parser.add_argument("--work-root", required=True, type=Path, help="New nonexistent D: review directory; never deleted")
    parser.add_argument("--repo", type=Path, default=Path("D:/robot_hand"))
    parser.add_argument("--git", default="C:/Program Files/Git/cmd/git.exe")
    parser.add_argument("--python", default=DEFAULT_PYTHON)
    parser.add_argument("--pythonpath", default=DEFAULT_PYTHONPATH)
    parser.add_argument("--timeout-seconds", type=int, default=3600)
    options = parser.parse_args()
    if not re.fullmatch(r"[0-9a-f]{40}", options.sha): parser.error("--sha requires exact lowercase 40-hex commit")
    if options.timeout_seconds <= 0: parser.error("Positive per-process timeout required")
    review = Review(options)
    try:
        review.execute()
    except BaseException as exc:
        review.report["errors"].append(dict(type=type(exc).__name__,message=str(exc),traceback=traceback.format_exc()))
    finally:
        review.finish()
    print(json.dumps(dict(report=str(review.work / "review.json"), completed=review.report["completed"],
                          reproduction_passed=review.report["reproduction_passed"])),flush=True)
    return 0 if review.report["reproduction_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

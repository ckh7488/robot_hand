"""Aggregate explicit articulated CAD part metadata and exact OCP mass tensors.

No CAD generation, subprocess or simulation runs here. Missing mass/routing
inputs are errors, not invitations to reuse old form/link defaults.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from model_adapter import ROOT, SCHEMA_VERSION, aggregate_mass, digest, validate_contract, _resolve_source


def build_contract(cad_root, *, step_path, transmission, mass_scope, limitations, repo_root=ROOT):
    repo=Path(repo_root).resolve();cad=Path(cad_root).resolve();step=Path(step_path).resolve()
    files={"design":cad/"design.json","scene":cad/"generated/scene.json",
           "mass":cad/"generated/mass_inertia.json","step":step}
    if any(not p.is_relative_to(repo) for p in files.values()):
        raise ValueError("CAD/mass/STEP inputs must belong to the declared repository")
    relative={k:p.relative_to(repo).as_posix() for k,p in files.items()}
    before={rel:digest(repo/rel) for rel in relative.values()}
    design=json.loads(files["design"].read_text(encoding="utf-8"))
    scene=json.loads(files["scene"].read_text(encoding="utf-8"))
    mass=json.loads(files["mass"].read_text(encoding="utf-8"))
    report_path=cad/"generated/build_report.json"
    report_hash=digest(report_path)
    report=json.loads(report_path.read_text(encoding="utf-8"))
    if scene.get("inputs_unchanged") is not True:
        raise ValueError("CAD export must report a successful unchanged-input guard")
    if report.get("inputs_unchanged") is not True or report.get("source_sha256")!=scene.get("source_sha256"):
        raise ValueError("CAD build report must match the successful scene source guard")
    if report.get("part_count")!=len(scene["hand"]) or report.get("all_parts_valid_single_solid") is not True:
        raise ValueError("CAD build report must confirm the same valid single-solid parts")
    before[report_path.relative_to(repo).as_posix()]=report_hash
    # Preserve the generator's provenance, not merely hashes of stale outputs.
    for document in (scene,mass):
        if not document.get("source_sha256"):
            raise ValueError("CAD scene and exact-mass outputs need generator source hashes")
        for rel,expected in document["source_sha256"].items():
            path=_resolve_source(repo,rel)
            if rel in before and before[rel]!=expected:
                raise ValueError("Conflicting CAD/export source hash: "+rel)
            if digest(path)!=expected:
                raise ValueError("Stale CAD/export source hash: "+rel)
            before[rel]=expected
    parts=[{k:p[k] for k in ("name","body","volume_mm3") if k in p} for p in scene["hand"]]
    bp=aggregate_mass(design,parts,mass["parts"])
    # Hash this helper and the exact new adapter used to produce the aggregate.
    for path in [Path(__file__).resolve(),Path(__file__).with_name("model_adapter.py").resolve()]:
        before[path.relative_to(repo).as_posix()]=digest(path)
    c=dict(schema_version=SCHEMA_VERSION,units={"length":"mm","mass":"kg","inertia":"kg*m^2","joint_angle":"deg"},
        design=design,part_metadata=parts,bodies=bp["bodies"],joints=bp["joints"],
        source_files=relative,source_sha256=before,transmission=transmission,
        mass_scope=mass_scope,limitations=bp["assumptions"]+list(limitations))
    after={rel:digest(repo/rel) for rel in before}
    if before!=after:
        raise ValueError("Input changed during mass aggregation")
    validate_contract(c,repo_root=repo)
    return c


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cad-root",type=Path,required=True)
    p.add_argument("--step",type=Path,required=True)
    p.add_argument("--transmission",type=Path,required=True,help="Explicit transmission JSON; no automatic old Jacobian")
    p.add_argument("--scope",type=Path,required=True,help="JSON with mass_scope and limitations")
    p.add_argument("--repo-root",type=Path,default=ROOT)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    declared_paths=[a.scope.resolve(),a.transmission.resolve()]
    if any(not p.is_relative_to(a.repo_root.resolve()) for p in declared_paths):
        raise ValueError("Scope and transmission declarations must be saved inside the repository")
    declared_hashes={p.relative_to(a.repo_root.resolve()).as_posix():digest(p) for p in declared_paths}
    scope=json.loads(a.scope.read_text(encoding="utf-8"))
    c=build_contract(a.cad_root,step_path=a.step,transmission=json.loads(a.transmission.read_text(encoding="utf-8")),
        mass_scope=scope["mass_scope"],limitations=scope["limitations"],repo_root=a.repo_root)
    c["source_sha256"].update(declared_hashes)
    validate_contract(c,repo_root=a.repo_root)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(c,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps(dict(output=str(a.output),parts=len(c["part_metadata"]),bodies=len(c["bodies"]),
        modeled_mass_kg=sum(b["mass_kg"] for b in c["bodies"]),hardware_performance_validated=False)))


if __name__=="__main__":
    main()

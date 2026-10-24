"""Snapshot only the new hand candidate and declared dependencies, never dirty CP007."""
from pathlib import Path
from datetime import datetime,timezone
import argparse
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--id',default='DRIVE001')
    p.add_argument('--results',default='sim/form_drive_v1/results/drive001');a=p.parse_args()
    if not a.id.isalnum():raise ValueError('Alphanumeric checkpoint id required')
    selected=set()
    for name in ['cad/form_candidate_v1','cad/form_drive_v1','sim/form_drive_v1']:
        selected.update(f.relative_to(ROOT).as_posix() for f in (ROOT/name).rglob('*') if f.is_file() and '__pycache__' not in f.parts)
    selected.update(['AGENTS.md','README.md','docs/FORM_REFERENCE.md','docs/REMOTE_DRIVE_PLAN.md',
        'docs/CHECKPOINT_WORKFLOW.md','checkpoints/'+a.id+'.md','tools/write_form_drive_checkpoint.py','AUDIT_EVIDENCE_INDEX.json'])
    geometry=json.loads((ROOT/'cad/form_drive_v1/generated/packaging/geometry.json').read_text())
    selected.update(geometry['source_sha256'])
    frozen=json.loads((ROOT/'AUDIT_EVIDENCE_INDEX.json').read_text());count=0
    for group in ['geometry_sha256','code_sha256','report_sha256']:
        for name,digest in frozen[group].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise ValueError('Frozen evidence changed: '+name)
            selected.add(name);count+=1
    rows={}
    for name in sorted(selected):
        file=(ROOT/name).resolve()
        if not file.is_relative_to(ROOT):raise ValueError('Path outside repository')
        rows[name]=dict(bytes=file.stat().st_size,sha256=hashlib.sha256(file.read_bytes()).hexdigest())
    manifest=ROOT/'checkpoints'/(a.id+'.manifest.json')
    value=dict(id=a.id,created_utc=datetime.now(timezone.utc).isoformat(),
        scope='New form-drive candidate, unchanged exterior, declared vendor and frozen V9 evidence dependencies, and selected documentation. Excludes unrelated unfinished CP007/webcam/scale work, PROGRESS.md, CHECKPOINT.json, this manifest and later review artifacts.',
        frozen_evidence_hashes_checked=count,files=rows)
    if manifest.exists():
        old=json.loads(manifest.read_text())
        if old['files']!=rows:raise ValueError('Existing manifest differs; preserve committed snapshots and choose a new id')
    else:manifest.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    results=(ROOT/a.results).resolve()
    if not results.is_relative_to(ROOT/'sim/form_drive_v1/results'):raise ValueError('Results must remain in the new simulation candidate')
    summary=json.loads((results/'summary.json').read_text())
    state=dict(id=a.id,candidate='cad/form_drive_v1',simulation='sim/form_drive_v1',baseline='cad/modular_roots_v9',
        exterior_reference='cad/form_candidate_v1',simulation_results=results.relative_to(ROOT).as_posix(),manifest=manifest.relative_to(ROOT).as_posix(),
        manufacturing_release=False,motor_performance_validated=False,
        status=f'New 20-motor neutral packaging and exact exterior-inertia hand model. Assembly/routing unfinished; {len(summary["cases"])} conceptual runs do not establish 50/100ms performance.',
        documentation_only=False,previous_checkpoint='CP006',previous_form_reference_sha='6475033ddaca54767eeba6b396a8c677fe0f8465',
        archive_branch='archive/direct-drive-v13',archive_sha='accd6c8d8d9791828233e9461ef1b1e8dccc8b0b',
        tracking_requirement_ms=dict(target=50,upper_limit=100,boundary='Robot target receipt to actual joint tracking; camera, perception and glove excluded'),
        numerical_convergence=dict(passed=summary['passed_convergence'],pairs=len(summary['convergence'])),
        post_commit_results_reproduced=False,post_commit_review_target=None,post_commit_review_report=None)
    (ROOT/'CHECKPOINT.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(manifest=str(manifest),files=len(rows),frozen_evidence_hashes_checked=count)),flush=True)

if __name__=='__main__':main()

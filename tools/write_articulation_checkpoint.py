"""Snapshot the new articulated drive candidate and explicit read-only dependencies."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]


def checked_inputs(mapping, selected):
    for name, expected in mapping.items():
        path=(ROOT/name).resolve()
        if not path.is_relative_to(ROOT):raise ValueError('Input outside repository: '+name)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise ValueError('Stale report input: '+name)
        selected.add(path.relative_to(ROOT).as_posix())


def main():
    selected=set()
    for folder in ('cad/articulation_drive_v2','sim/articulation_drive_v2',
                   'cad/webcam_articulation_v1','sim/form_drive_v1/reference_06b9ab4'):
        selected.update(p.relative_to(ROOT).as_posix() for p in (ROOT/folder).rglob('*')
                        if p.is_file() and '__pycache__' not in p.parts)
    selected.update(['AGENTS.md','README.md','docs/FORM_REFERENCE.md','docs/REMOTE_DRIVE_PLAN.md',
        'docs/CHECKPOINT_WORKFLOW.md','AUDIT_EVIDENCE_INDEX.json','checkpoints/DRIVE002.md',
        'tools/write_articulation_checkpoint.py','tools/review_articulation_checkpoint.py','cad/form_drive_v1/exterior.py',
        'sim/form_drive_v1/model_adapter.py','sim/form_drive_v1/results/drive001/parameters.json'])
    for filename in ('cad/articulation_drive_v2/generated/packaging_screen.json',
                     'cad/articulation_drive_v2/generated/render_inputs.json'):
        report=json.loads((ROOT/filename).read_text())
        if report.get('inputs_unchanged') is not True:raise ValueError('Incomplete report: '+filename)
        checked_inputs(report['input_sha256'],selected)
    for row in json.loads((ROOT/'sim/articulation_drive_v2/results/drive002/input_manifest.json').read_text()):
        checked_inputs({row['path']:row['sha256']},selected)
    review=json.loads((ROOT/'cad/articulation_drive_v2/generated/assembly_review.json').read_text())
    if not review.get('completed') or not review.get('input_hash_guard',{}).get('passed'):
        raise ValueError('Incomplete or stale geometry verification')
    checked_inputs(review['input_sha256'],selected)
    for filename in ('scene.json','build_report.json','mass_inertia.json','dynamics_contract.json'):
        report=json.loads((ROOT/'cad/articulation_drive_v2/generated'/filename).read_text())
        checked_inputs(report['source_sha256'],selected)
    summary=json.loads((ROOT/'sim/articulation_drive_v2/results/drive002/summary.json').read_text())
    if summary.get('inputs_unchanged') is not True:raise ValueError('Stale simulation inputs')
    image_report=json.loads((ROOT/'cad/articulation_drive_v2/generated/render_inputs.json').read_text())
    checked_inputs({'cad/articulation_drive_v2/generated/robot_hand_drive002.png':image_report['image_sha256']},selected)
    # Validate frozen baseline independently, without staging any baseline edits.
    frozen=json.loads((ROOT/'AUDIT_EVIDENCE_INDEX.json').read_text());count=0
    for group in ('geometry_sha256','code_sha256','report_sha256'):
        for name,digest in frozen[group].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:
                raise ValueError('Frozen evidence changed: '+name)
            selected.add(name);count+=1
    files={}
    for name in sorted(selected):
        path=(ROOT/name).resolve()
        if not path.is_relative_to(ROOT):raise ValueError('Input outside repository')
        files[path.relative_to(ROOT).as_posix()]={'bytes':path.stat().st_size,
            'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    value=dict(id='DRIVE002',created_utc=datetime.now(timezone.utc).isoformat(),
        scope='Articulated CAD root repairs, explicit new dynamics, saved source CAD, packaging dependencies, frozen V9 evidence and selected documents. Excludes unrelated dirty CP007/scale/webcam application work, PROGRESS, CHECKPOINT, this manifest and later review.',
        frozen_evidence_hashes_checked=count,files=files)
    manifest=ROOT/'checkpoints/DRIVE002.manifest.json'
    if manifest.exists():
        if json.loads(manifest.read_text())['files']!=files:
            raise ValueError('Preserve existing checkpoint manifest; choose a new id')
    else:manifest.write_text(json.dumps(value,indent=2)+'\n')
    state=dict(id='DRIVE002',candidate='cad/articulation_drive_v2',simulation='sim/articulation_drive_v2',
        exterior_reference='cad/webcam_articulation_v1',manifest='checkpoints/DRIVE002.manifest.json',
        simulation_results='sim/articulation_drive_v2/results/drive002',
        status='Changed thumb/fist CAD integrated into root assembly repairs and new20axis mass/kinematics. Remaining fabrication and drive failures are retained.',
        manufacturing_release=False,motor_performance_validated=False,
        previous_checkpoint='DRIVE001',source_geometry_audit='WEBCAM003',
        archive_branch='archive/direct-drive-v13',archive_sha='accd6c8d8d9791828233e9461ef1b1e8dccc8b0b',
        tracking_requirement_ms=dict(target=50,upper_limit=100,boundary='Robot target receipt to actual joint tracking; perception excluded'),
        post_commit_results_reproduced=False,post_commit_review_target=None,post_commit_review_report=None)
    (ROOT/'CHECKPOINT.json').write_text(json.dumps(state,indent=2)+'\n')
    print(json.dumps(dict(files=len(files),bytes=sum(x['bytes'] for x in files.values()),frozen=count)))


if __name__=='__main__':main()

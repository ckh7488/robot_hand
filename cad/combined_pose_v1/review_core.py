"""Exact-SHA replay of one combined hand-shape example and its core route."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys

import cadquery as cq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',type=Path,required=True)
    a = ap.parse_args()
    if a.output.exists() or a.output.resolve().is_relative_to(ROOT):
        raise ValueError('New external output required')
    def git(*args):
        return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf-8',
            creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD') != a.commit or git('status','--porcelain=v1'):
        raise ValueError('Wrong or dirty target clone')
    spec = importlib.util.spec_from_file_location('_combined_core',HERE/'audit.py')
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    p = m.p
    manifest = p.read(ROOT/'checkpoints/EXAMPLE007.manifest.json')
    def guard():
        return all(p.sha(ROOT/r)==row['sha256'] for r,row in manifest['files'].items())
    frozen = {}
    index = p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')
    for key in ['geometry_sha256','code_sha256','report_sha256']:
        frozen.update(index[key])
    if not guard() or not all(p.sha(ROOT/r)==h for r,h in frozen.items()):
        raise ValueError('Changed manifest or frozen evidence')
    source = p.read(HERE/'reviews/audit.json')
    if not source['completed'] or not source['input_hash_guard']:
        raise ValueError('Incomplete source')
    audit = m.m.ObjectAudit()
    obj = cq.importers.importStep(str(ROOT/source['object']['step'])).val()
    probes = []
    for path in source['paths']:
        samples = path['forward_observations']
        indices = {0,len(samples)//4,len(samples)//2,3*len(samples)//4,len(samples)-1} if path['status']=='sampled_approach_release_clear' else {len(samples)-1}
        probes.extend(dict(id=path['id']+'@'+str(i),expected=samples[i]['native']) for i in sorted(indices))
    if not probes:
        probes = [dict(id=source['cases'][0]['id'],expected=source['cases'][0]['native'])]
    def signature(native):
        return ({tuple(sorted([r['a'],r['b']])):r['material_intersection_mm3'] for r in native['hand']['collision_details']},
            {r['part']:r['material_intersection_mm3'] for r in native['object_collisions']})
    differences = []
    results = []
    for probe in probes:
        expected = probe['expected']
        got,moved,_ = audit.evaluate(expected['hand']['angles_deg'],expected['object_centre_mm'],12.5,obj)
        got['contacts'] = [dict(part=row['part'],gap_mm=float(moved[row['part']].distance(obj))) for row in expected['contacts']]
        for before,after in zip(signature(expected),signature(got)):
            if before.keys()!=after.keys() or any(abs(before[k]-after[k])>1e-5 for k in before.keys() & after.keys()):
                differences.append(dict(id=probe['id'],reason='Material intersection changed'))
        if len(expected['hand']['operation_failures'])!=len(got['hand']['operation_failures']) or len(expected['object_operation_failures'])!=len(got['object_operation_failures']):
            differences.append(dict(id=probe['id'],reason='Uncertainty changed'))
        if any(abs(x['gap_mm']-y['gap_mm'])>1e-5 for x,y in zip(expected['contacts'],got['contacts'])):
            differences.append(dict(id=probe['id'],reason='Contact distance changed'))
        results.append(dict(id=probe['id'],native=got))
        print(probe['id'],got['hand']['collision_count'],len(got['object_collisions']),flush=True)
    clean = not git('status','--porcelain=v1')
    report = dict(target_commit=a.commit,passed=not differences and clean and guard(),
        clean_checkout=clean,input_hash_guard=guard(),manifest_files=len(manifest['files']),frozen_files=len(frozen),
        core_probes=results,differences=differences,independent_audit=False,
        scope='Five full364hand/object core configurations on each clear forward route or first reported failed sample. Contact distances and snapshot/frozen hashes. Not full route, rerender, continuous motion, physical force or manufacturing approval.')
    p.write(a.output,report)
    if not report['passed']:
        raise ValueError('Changed core reproduction')


if __name__=='__main__':
    main()

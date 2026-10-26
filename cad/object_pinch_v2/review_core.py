"""Replay saved hand/object core probes from a clean exact-SHA checkout."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import cadquery as cq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--commit', required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    if a.output.exists() or a.output.resolve().is_relative_to(ROOT):
        raise ValueError('New external output required')
    def git(*args):
        return subprocess.check_output(['git', *args], cwd=ROOT, text=True, encoding='utf-8',
            creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse', 'HEAD') != a.commit or git('status', '--porcelain=v1'):
        raise ValueError('Wrong or dirty checkout')
    spec = importlib.util.spec_from_file_location('_pinch_replay', HERE/'audit.py')
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    p = m.p
    manifest = p.read(ROOT/'checkpoints/PINCH006.manifest.json')
    def guard():
        return all(p.sha(ROOT/r) == v['sha256'] for r,v in manifest['files'].items())
    frozen = {}
    index = p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')
    for key in ['geometry_sha256', 'code_sha256', 'report_sha256']:
        frozen.update(index[key])
    if not guard() or not all(p.sha(ROOT/r) == h for r,h in frozen.items()):
        raise ValueError('Changed snapshot or frozen evidence')
    source = p.read(HERE/'reviews/audit.json')
    if not source['completed'] or not source['input_hash_guard']:
        raise ValueError('Incomplete evidence')
    audit = m.previous.ObjectAudit()
    cases = {c['id']:c for c in source['cases']}
    probes = [dict(id=c['id'], case=c['id'], expected=c['native']) for c in source['cases']]
    for path in source['paths']:
        obs = path['forward_observations']
        indices = {0, len(obs)//4, len(obs)//2, 3*len(obs)//4, len(obs)-1} if path['status'] == 'sampled_approach_release_clear' else {len(obs)-1}
        for i in sorted(indices):
            probes.append(dict(id=path['id']+'@'+str(i), case=path['target'], expected=obs[i]['native']))
    differences = []
    results = []
    def signatures(native):
        return ({tuple(sorted([r['a'], r['b']])):r['material_intersection_mm3'] for r in native['hand']['collision_details']},
            {r['part']:r['material_intersection_mm3'] for r in native['object_collisions']})
    objects = {}
    for case in cases.values():
        obj = cq.importers.importStep(str(ROOT/case['object_step'])).val()
        if not obj.isValid() or len(obj.Solids()) != 1 or abs(obj.Volume()-4/3*math.pi*12.5**3) > 1e-5:
            raise ValueError('Invalid saved full sphere')
        objects[case['id']] = obj
    for probe in probes:
        case = cases[probe['case']]
        expected = probe['expected']
        actual, moved, obj = audit.evaluate(expected['hand']['angles_deg'], expected['object_centre_mm'], 12.5, objects[case['id']])
        actual['contacts'] = [dict(part=c['part'], gap_mm=float(moved[c['part']].distance(obj))) for c in expected['contacts']]
        for before, after in zip(signatures(expected), signatures(actual)):
            if before.keys() != after.keys() or any(abs(before[k]-after[k]) > 1e-5 for k in before.keys() & after.keys()):
                differences.append(dict(id=probe['id'], reason='Material intersection changed'))
        if len(expected['hand']['operation_failures']) != len(actual['hand']['operation_failures']) or len(expected['object_operation_failures']) != len(actual['object_operation_failures']):
            differences.append(dict(id=probe['id'], reason='Uncertainty set changed'))
        if any(abs(x['gap_mm']-y['gap_mm']) > 1e-5 for x,y in zip(expected['contacts'], actual['contacts'])):
            differences.append(dict(id=probe['id'], reason='Surface distance changed'))
        results.append(dict(id=probe['id'], native=actual))
        print(json.dumps(dict(replayed=probe['id'], hand=actual['hand']['collision_count'], object=len(actual['object_collisions']))), flush=True)
    clean = not git('status', '--porcelain=v1')
    result = dict(target_commit=a.commit, passed=not differences and clean and guard(),
        clean_checkout=clean, input_hash_guard=guard(), manifest_files=len(manifest['files']),
        frozen_files=len(frozen), probes=results, differences=differences, independent_audit=False,
        scope='All saved candidate endpoints; five points per selected clear approach; first failure of failed paths; full364hand and object checks/contact distances. Not optimizer, all route samples, renderer, continuous motion, physical holding or manufacturing release.')
    p.write(a.output, result)
    if not result['passed']:
        raise ValueError('Changed reproduction')


if __name__ == '__main__':
    main()

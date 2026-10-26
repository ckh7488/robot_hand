"""Replay a saved exact-commit functional checkpoint, including its failures.

Reproducible findings are not a design approval or an independent audit.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(ROOT):raise ValueError('New external output required')
    def git(*a):return subprocess.check_output(['git',*a],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=args.commit or git('status','--porcelain=v1'):raise ValueError('Wrong or dirty checkout')
    manifest=read(ROOT/'checkpoints/FUNCTION003.manifest.json')
    def guard():return all(sha(ROOT/p)==r['sha256'] for p,r in manifest['files'].items())
    if not guard():raise ValueError('Manifest closure changed')
    spec=importlib.util.spec_from_file_location('_function_replay_audit',HERE/'audit.py')
    audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    native,v,cat,scene,parts=audit.load();engine=native.PairScreen(v,parts,scene['hand'],cat['design'],0.,True)
    source=read(HERE/'reviews/audit.json')
    if not source['completed'] or not source['input_hash_guard']:raise ValueError('Incomplete source')
    differences=[];results=[]
    def signature(rows):return {tuple(sorted([r['a'],r['b']])):r['material_intersection_mm3'] for r in rows}
    wanted={'gesture_open','count_1','four_hook','four_tabletop','little_curl','fan_half',
        'pinch_index','pinch_middle','pinch_ring','pinch_little','finger_heart',
        'cross_old_reference','cross_middle_14','cross_middle_18','cross_index_14','cross_index_22'}
    probes=[dict(id=r['id'],expected=r['native']) for r in source['cases'] if r['id'] in wanted]
    for p in source['paths']:
        if 'observations' in p:
            # Replay a true intermediate, not just its stored clear endpoint.
            rows=p['observations'];mid=rows[max(1,len(rows)//4)]
            probes.append(dict(id=p['id']+'@intermediate',expected=mid['native']))
    # A separately saved practical pose/path correction can be added without
    # rewriting the original broad audit or its failing input poses.
    fixpath=HERE/'reviews/motion_fix.json'
    if fixpath.exists():
        fixes=read(fixpath)
        for p in fixes['cases']:probes.append(dict(id=p['id'],expected=p['native']))
        for path in fixes['paths']:
            obs=path['observations']
            for ix in sorted({0,len(obs)//4,len(obs)//2,3*len(obs)//4,len(obs)-1}):
                probes.append(dict(id=path['id']+'@'+str(ix),expected=obs[ix]['native']))
    for p in probes:
        got=audit.compact(engine,p['expected']['angles_deg']);before=signature(p['expected']['collision_details']);after=signature(got['collision_details'])
        if before.keys()!=after.keys() or len(p['expected']['operation_failures'])!=len(got['operation_failures']):
            differences.append(dict(id=p['id'],reason='pair set or uncertainty changed'))
        for pair in before.keys() & after.keys():
            if abs(before[pair]-after[pair])>1e-5:differences.append(dict(id=p['id'],pair=pair,expected=before[pair],actual=after[pair]))
        results.append(dict(id=p['id'],native=got));print(json.dumps(dict(replayed=p['id'],collisions=got['collision_count'])),flush=True)
    spec=importlib.util.spec_from_file_location('_function_replay_fk',ROOT/'cad/gesture_hand_v3/poses.py')
    fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk);contact_checks=[]
    for case in cat['cases']:
        if 'contact_digit' not in case:continue
        values=[]
        frames=fk.forward_kinematics(cat['design'],case['angles_deg'])['bodies']
        for digit in [case['contact_digit'],'thumb']:
            anchor=cat['pad_anchors'][digit];t=frames[anchor['body']]
            values.append((t[:3,:3]@anchor['local_point_mm']+t[:3,3],t[:3,:3]@anchor['local_outward_normal']))
        distance=float(np.linalg.norm(values[0][0]-values[1][0]));dot=float(values[0][1]@values[1][1])
        old=case['functional_metrics']
        if abs(distance-old['landmark_separation_mm'])>1e-8 or abs(dot-old['outward_normal_dot'])>1e-10:
            differences.append(dict(id=case['id'],reason='contact metric changed'))
        contact_checks.append(dict(id=case['id'],landmark_separation_mm=distance,outward_normal_dot=dot))
    index=read(ROOT/'AUDIT_EVIDENCE_INDEX.json');frozen=index['geometry_sha256']
    frozen_ok=all(sha(ROOT/p)==h for p,h in frozen.items())
    status=git('status','--porcelain=v1');final_guard=guard()
    report=dict(target_commit=args.commit,passed=not differences and frozen_ok and final_guard and not status,
        manifest_files=len(manifest['files']),frozen_files=len(frozen),frozen_hashes_passed=frozen_ok,
        whole_hand_core_probes=results,contact_metric_replays=contact_checks,differences=differences,
        input_hash_guard=final_guard,clean_checkout=not status,
        scope='Selected full364part native poses and stored intermediates; all six contact metrics. Not the full59endpoint/allpath audit, all optimizer starts, all33 object grasps, continuous or hardware validation.',
        independent_audit=False,design_approved=False)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8')
    if not report['passed']:raise ValueError('Core reproduction changed')


if __name__=='__main__':main()

"""Reimport and rebuild the saved single-motor tail from one exact clean SHA."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',required=True,type=Path)
    a=ap.parse_args()
    if a.output.exists() or a.output.resolve().is_relative_to(ROOT):
        raise ValueError('New external review output required')
    def git(*args):
        return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf-8',
            creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=a.commit or git('status','--porcelain=v1'):
        raise ValueError('Wrong or dirty checkout')
    spec=importlib.util.spec_from_file_location('_corrected_replay',HERE/'corrected.py')
    c=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(c)
    b,m,p=c.b,c.m,c.p
    manifest=p.read(ROOT/'checkpoints/WIRE009.manifest.json')
    frozen={}
    for key,value in p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json').items():
        if key in ['geometry_sha256','code_sha256','report_sha256']:
            frozen.update(value)
    def guard():
        return all(b.sha(ROOT/r)==v['sha256'] for r,v in manifest['files'].items()) and all(b.sha(ROOT/r)==h for r,h in frozen.items())
    if not guard():
        raise ValueError('Changed manifest/frozen evidence')
    expected=p.read(HERE/'reviews/corrected.json')
    if not expected['completed'] or not expected['nominal_leads_clear']:
        raise ValueError('No successful nominal geometric lead to replay')
    native,v,cat,scene,hand=p.load()
    saved=p.read(HERE/'generated/scene.json')
    shapes={r['name']:cq.importers.importStep(str(ROOT/r['step'])).val() for r in saved['parts']}
    leads=[dict(name=r['name'],shape=shapes[r['name']]) for r in saved['parts'] if r['role']=='nominal_electrical_wire_space']
    obstacles=c.load_obstacles()
    chosen=next(r for r in expected['candidates'] if r['id']==expected['selected'])
    differences=[]
    def compare(label,got,want):
        def signature(report):
            return {(r['lead'],r['part']):r['material_intersection_mm3'] for r in report['overlaps']}
        x,y=signature(got),signature(want)
        if x.keys()!=y.keys() or any(abs(x[k]-y[k])>1e-5 for k in x.keys() & y.keys()) or len(got['operation_failures'])!=len(want['operation_failures']):
            differences.append(dict(id=label,reason='Material overlaps or operation uncertainty changed'))
    rebuilding=[]
    for r in saved['parts']:
        if r['role']!='nominal_electrical_wire_space':
            continue
        local=r['local']
        rebuilt,_=c.lead_shape(local['straight_stub_mm'],local['start_mm'][1],local['centreline_radius_mm'])
        rebuilt=m.pack._transform(rebuilt,expected['transform_world_mm'])
        delta=float(rebuilt.cut(shapes[r['name']],tol=1e-6).Volume()+shapes[r['name']].cut(rebuilt,tol=1e-6).Volume())
        rebuilding.append(dict(name=r['name'],symmetric_difference_mm3=delta))
        if delta>.001:
            differences.append(dict(id=r['name'],reason='Rebuilt and stored wire differ'))
    package=b.test_against(leads,obstacles,v)
    compare('all225packaging',package,chosen['packaging'])
    wire_pairs=[]
    for i,row in enumerate(leads):
        for other in leads[i+1:]:
            vol=float(row['shape'].intersect(other['shape'],tol=1e-6).Volume())
            gap=float(row['shape'].distance(other['shape']))
            want=next(r for r in chosen['wire_pairs'] if r['a']==row['name'] and r['b']==other['name'])
            wire_pairs.append(dict(a=row['name'],b=other['name'],material_intersection_mm3=vol,surface_gap_mm=gap))
            if abs(vol-want['material_intersection_mm3'])>1e-5 or abs(gap-want['surface_gap_mm'])>1e-5:
                differences.append(dict(id=row['name']+'/'+other['name'],reason='Wire-wire geometry changed'))
    neutral=v.frames(cat['design'],np.zeros(20))
    handchecks=[]
    for case in expected['hand_pose_checks']:
        frames=v.frames(cat['design'],case['angles_deg'])
        current=[dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
        got=b.test_against(leads,current,v)
        compare(case['id'],got,case['native'])
        handchecks.append(dict(id=case['id'],native=got))
    arm=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(r['asset']+'.step'))).val())
        for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
    armcheck=b.test_against(leads,arm,v)
    compare('neutral10arm',armcheck,expected['neutral_arm_check'])
    plugcheck=b.test_against([dict(name='provisional_plug_outer_envelope',shape=shapes['provisional_plug_outer_envelope'])],obstacles,v)
    compare('plug_envelope',plugcheck,expected['provisional_plug_check'])
    bs=importlib.util.spec_from_file_location('_batch_replay',HERE/'batch.py')
    batch=importlib.util.module_from_spec(bs)
    bs.loader.exec_module(batch)
    batch_expected=p.read(HERE/'reviews/batch.json')
    batch_leads=[]
    batch_plugs=[]
    for row in p.read(HERE/'generated/batch_scene.json')['parts']:
        loaded=dict(name=row['name'],shape=cq.importers.importStep(str(ROOT/row['step'])).val())
        (batch_leads if row['role']=='nominal_electrical_wire_space' else batch_plugs).append(loaded)
    batch_package=b.test_against(batch_leads,obstacles,v)
    compare('batch_packaging',batch_package,batch_expected['packaging'])
    batch_pairs=batch.wire_pairs(batch_leads,v)
    compare('batch_wire_pairs',batch_pairs,batch_expected['wire_pairs'])
    batch_hand=[]
    for case in batch_expected['hand_pose_checks']:
        frames=v.frames(cat['design'],case['angles_deg'])
        current=[dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
        got=b.test_against(batch_leads,current,v)
        compare('batch_'+case['id'],got,case['native'])
        batch_hand.append(dict(id=case['id'],native=got))
    batch_arm=b.test_against(batch_leads,arm,v)
    compare('batch_neutral_arm',batch_arm,batch_expected['neutral_arm_check'])
    batch_plug=b.test_against(batch_plugs,obstacles,v)
    compare('batch_plug',batch_plug,batch_expected['provisional_plug_check'])
    clean=not git('status','--porcelain=v1')
    report=dict(target_commit=a.commit,passed=not differences and clean and guard(),clean_checkout=clean,input_hash_guard=guard(),
        manifest_files=len(manifest['files']),frozen_files=len(frozen),differences=differences,
        wire_rebuilding=rebuilding,packaging=package,wire_pairs=wire_pairs,hand_pose_checks=handchecks,
        neutral_arm_check=armcheck,provisional_plug_check=plugcheck,independent_audit=False,
        batch=dict(packaging=batch_package,wire_pairs=batch_pairs,hand_pose_checks=batch_hand,
            neutral_arm_check=batch_arm,provisional_plug_check=batch_plug),
        scope='Source3tails rebuilt; all48saved remote tails reimported against225packaging,5hand endpoints/all364parts,neutral10arm and1128wire pairs;16plug outer boxes checked. Fixed reference only, no dynamic harness/mating/crimp/installation/physical approval.')
    p.write(a.output,report)
    if not report['passed']:
        raise ValueError('Exact-SHA replay differs')


if __name__=='__main__':
    main()

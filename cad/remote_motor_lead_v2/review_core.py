"""Exact-SHA saved-wire collisions and analytic source/STEP roundtrip checks."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys

import cadquery as cq
import numpy as np
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(ROOT):
        raise ValueError('New external output required')
    def git(*xs):
        return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=args.commit or git('status','--porcelain=v1'):
        raise ValueError('Wrong/dirty target checkout')
    spec=importlib.util.spec_from_file_location('_post_review',HERE/'route.py')
    t=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(t)
    b,m,p=t.b,t.m,t.p
    analytic=t.module('_post_analytic',ROOT/'cad/remote_motor_lead_v1/roundtrip_review.py')
    manifest=p.read(ROOT/'checkpoints/WIRE010.manifest.json')
    frozen={}
    for key,value in p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json').items():
        if key in ['geometry_sha256','code_sha256','report_sha256']:
            frozen.update(value)
    def guard():
        return all(b.sha(ROOT/r)==row['sha256'] for r,row in manifest['files'].items()) and all(b.sha(ROOT/r)==h for r,h in frozen.items())
    if not guard():
        raise ValueError('Changed manifest/frozen evidence')
    expected=p.read(HERE/'reviews/check.json')
    if not expected['completed'] or not expected['all_nominal_leads_clear']:
        raise ValueError('No clear final route')
    native,v,cat,scene,hand=p.load()
    obstacles=t.c.load_obstacles()
    rows=p.read(HERE/'generated/scene.json')['parts']
    leads=[]
    plugs=[]
    differences=[]
    rebuilding=[]
    for row in rows:
        shape=cq.importers.importStep(str(ROOT/row['step'])).val()
        if row['role']!='nominal_electrical_wire_space':
            plugs.append(dict(name=row['name'],shape=shape))
            continue
        leads.append(dict(name=row['name'],shape=shape))
        if row['module'] not in t.CHANGED:
            continue
        fresh,_=t.lead_shape(row['local'],expected['selected'])
        fresh=m.pack._transform(fresh,row['transform_world_mm'])
        a,bx=analytic.signatures(fresh,m.pack._bbox),analytic.signatures(shape,m.pack._bbox)
        values=[]
        mismatch=[]
        if not a['valid'] or not bx['valid'] or a['solids']!=1 or bx['solids']!=1:
            mismatch.append('solid validity')
        for key in ['global_values','vertices']:
            aa,bb=np.asarray(a[key]),np.asarray(bx[key])
            if aa.shape!=bb.shape or not np.allclose(aa,bb,rtol=0,atol=1e-5):
                mismatch.append(key)
            else:
                values.extend(abs(aa-bb).ravel().tolist())
        for key in ['faces','edges']:
            if len(a[key])!=len(bx[key]):
                mismatch.append(key+' count')
                continue
            for aa,bb in zip(a[key],bx[key]):
                if aa['kind']!=bb['kind'] or len(aa['values'])!=len(bb['values']) or not np.allclose(aa['values'],bb['values'],rtol=0,atol=1e-5):
                    mismatch.append(key+' analytic parameters')
                else:
                    values.extend(abs(np.asarray(aa['values'])-np.asarray(bb['values'])).tolist())
        loc=row['local']
        positions=[(np.asarray(loc['start_mm'])+np.asarray(loc['arc_start_mm']))/2,np.asarray(loc['arc_middle_mm']),
            (np.asarray(loc['arc_end_mm'])+np.asarray(loc['end_mm']))/2]
        for pos in positions:
            xyz=(np.asarray(row['transform_world_mm'])@np.r_[pos,1.])[:3]
            if any(BRepClass3d_SolidClassifier(s.Solids()[0].wrapped,gp_Pnt(*xyz),1e-6).State()!=TopAbs_IN for s in [fresh,shape]):
                mismatch.append('span interior')
        rebuilding.append(dict(name=row['name'],matched=not mismatch,max_numeric_difference=max(values),mismatch=mismatch,
            source_faces=a['faces'],saved_faces=bx['faces'],source_edges=a['edges'],saved_edges=bx['edges']))
        if mismatch:
            differences.append(dict(id=row['name'],reason=mismatch))
    def compare(label,got,want):
        def signature(report):
            return {(r['lead'],r['part']):r['material_intersection_mm3'] for r in report['overlaps']}
        a,bx=signature(got),signature(want)
        if a.keys()!=bx.keys() or any(abs(a[k]-bx[k])>1e-5 for k in a.keys() & bx.keys()) or len(got['operation_failures'])!=len(want['operation_failures']):
            differences.append(dict(id=label,reason='Material overlaps or uncertainty changed'))
    before=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/r['step'])).val()) for r in t.saved_rows()
        if r['role']=='nominal_electrical_wire_space' and r['module'] in t.CHANGED]
    baseline=b.test_against(before,obstacles,v)
    compare('before8post_clashes',baseline,expected['before'])
    package=b.test_against(leads,obstacles,v)
    chosen=next(r for r in expected['candidates'] if r['bend_plane_angle_deg']==expected['selected'])
    compare('all225packaging',package,chosen['packaging'])
    pairs=t.old.wire_pairs(leads,v)
    compare('all1128wirepairs',pairs,chosen['wire_pairs'])
    envelope=b.test_against(leads,plugs,v)
    compare('all16connector_envelopes',envelope,chosen['wire_vs_connector_envelopes'])
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
    clean=not git('status','--porcelain=v1')
    result=dict(target_commit=args.commit,passed=not differences and clean and guard(),clean_checkout=clean,input_hash_guard=guard(),
        manifest_files=len(manifest['files']),frozen_files=len(frozen),differences=differences,analytic_source_roundtrip=rebuilding,
        before=baseline,packaging=package,wire_pairs=pairs,wire_vs_connector_envelopes=envelope,hand_pose_checks=handchecks,
        neutral_arm_check=armcheck,independent_audit=False,
        scope='All12changed wires analytic rebuild/STEP comparison;48saved wires vs225packaging/16plug envelopes,1128wire pairs,5hand endpoints/all364parts,neutral10arm and prior8post clashes. No coincident-solid Boolean equivalence claim, mounting/harness dynamics/physical approval.')
    p.write(args.output,result)
    if not result['passed']:
        raise ValueError('Exact-SHA core differs')


if __name__=='__main__':
    main()

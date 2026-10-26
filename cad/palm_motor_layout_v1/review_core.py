"""Replay exact-SHA native placement probes and rigid source/STEP signatures."""
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


def signature(shape, bounds):
    def values(s):
        return [*np.asarray(bounds(s)).ravel(),*s.Center().toTuple()]
    def order(row):return tuple(round(float(v),4) for v in row)
    faces=sorted([(f.geomType(),[f.Area(),*values(f)]) for f in shape.Faces()],key=lambda z:(z[0],order(z[1])))
    edges=sorted([(e.geomType(),[e.Length(),*values(e)]) for e in shape.Edges()],key=lambda z:(z[0],order(z[1])))
    vertices=sorted([list(v.toTuple()) for v in shape.Vertices()],key=order)
    return dict(valid=shape.isValid(),solids=len(shape.Solids()),global_values=[shape.Volume(),shape.Area(),*values(shape)],
                faces=faces,edges=edges,vertices=vertices)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(ROOT):raise ValueError('New external output required')
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=args.commit or git('status','--porcelain=v1'):raise ValueError('Wrong/dirty target')
    spec=importlib.util.spec_from_file_location('_palm_exact_core',HERE/'check.py')
    c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
    p,b,m=c.p,c.b,c.m
    manifest=p.read(ROOT/'checkpoints/PALM014.manifest.json')
    frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(b.sha(ROOT/r)==v['sha256'] for r,v in manifest['files'].items()) and all(b.sha(ROOT/r)==v for r,v in frozen.items())
    if not guard():raise ValueError('Changed manifest/frozen inputs')
    expected=p.read(HERE/'reviews/check2.json');first=p.read(HERE/'reviews/check.json')
    assert expected['completed'] and expected['input_hash_guard'] and expected['nominal_placement_clear'] and expected['changed_feet_seated']
    _,v,cat,scene,hand=p.load()
    saved=p.read(HERE/'selected/generated/scene.json')
    changed=c.load(saved['changed_parts']);retained=c.load(saved['retained_packaging_guide'])
    oldmeta=p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')['retained_packaging']
    plan=p.read(HERE/'placement2.json')
    old=c.load([r for r in oldmeta if r.get('module') in plan['changes']])
    oldmap={r['name']:r for r in old}
    differences=[];rebuilding=[];samples=[]
    for row in changed:
        fresh=c.transform(oldmap[row['name']],plan['changes'])['shape']
        a,z=signature(fresh,m.pack._bbox),signature(row['shape'],m.pack._bbox)
        mismatch=[];numeric=[]
        if not a['valid'] or not z['valid'] or a['solids']!=z['solids']:mismatch.append('valid/solid count')
        for key in ['global_values','vertices']:
            aa,zz=np.asarray(a[key]),np.asarray(z[key])
            if aa.shape!=zz.shape or not np.allclose(aa,zz,rtol=0,atol=1e-5):mismatch.append(key)
            else:numeric.extend(abs(aa-zz).ravel().tolist())
        for key in ['faces','edges']:
            if len(a[key])!=len(z[key]):mismatch.append(key+' count');continue
            for aa,zz in zip(a[key],z[key]):
                if aa[0]!=zz[0] or not np.allclose(aa[1],zz[1],rtol=0,atol=1e-5):mismatch.append(key+' signature')
                else:numeric.extend(abs(np.asarray(aa[1])-np.asarray(zz[1])).tolist())
        rebuilding.append(dict(name=row['name'],matched=not mismatch,mismatch=mismatch,max_numeric_difference=max(numeric or [0.])))
        if mismatch:differences.append('source_saved_'+row['name'])
    neutral=v.frames(cat['design'],np.zeros(20))
    def hand_at(q):
        frames=v.frames(cat['design'],q)
        return [dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
    def compare(label,got,want):
        def sig(value):return {(r['lead'],r['part']):r['material_intersection_mm3'] for r in value['overlaps']}
        aa,zz=sig(got),sig(want)
        matched=aa.keys()==zz.keys() and all(abs(aa[k]-zz[k])<=1e-5 for k in aa.keys()&zz.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        samples.append(dict(id=label,native=got,matched=matched))
        if not matched:differences.append(label)
        print(label,matched,flush=True)
    compare('original_fixed8',c.material(old,hand_at(np.zeros(20))),expected['before_open'])
    for key in ['open','combined']:
        case=next(r for r in expected['cases'] if r['id']==key)
        compare(key,c.material(changed,hand_at(case['angles_deg'])),case['native'])
    compare('other329_packaging_guides',c.material(changed,retained),expected['other_packaging'])
    for pair in expected['between_moved_modules']:
        a,z=pair['modules']
        compare('between_'+a+'_'+z,c.material([r for r in changed if r['module']==a],[r for r in changed if r['module']==z]),pair['native'])
    wires=c.load(p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'])
    compare('48fixed_leads_16plug_envelopes',c.material(changed,wires),expected['fixed_remote_wires_and_plug_envelopes'])
    arm=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(r['asset']+'.step'))).val())
         for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
    compare('neutral10arm',c.material(changed,arm),expected['neutral_arm'])
    # Preserve the first alternative as a failed placement, not a hidden edit.
    firstparts=c.load(p.read(HERE/'generated/scene.json')['changed_parts'])
    firstmap={r['name']:r for r in firstparts}
    firstchecks=[]
    for overlap in first['cases'][0]['native']['overlaps']:
        got=c.material([firstmap[overlap['lead']]],[dict(name=overlap['part'],shape=hand[overlap['part']])])
        want=dict(pairs_in_scope=1,overlaps=[overlap],operation_failures=[])
        compare('failed_first_'+overlap['lead'],got,want)
        firstchecks.append(overlap['lead'])
    contacts=c.floor_contacts(changed,hand['Palm_tapered_chassis'])
    contact_checks=[]
    for a,z in zip(contacts,expected['floor_contacts']):
        witnesses=all(np.allclose(aa['point_mm'],zz['point_mm'],rtol=0,atol=1e-5)
                      and aa['below_in_palm']==zz['below_in_palm'] and aa['above_in_palm']==zz['above_in_palm']
                      for aa,zz in zip(a['samples'],z['samples']))
        matched=a['module']==z['module'] and a['seated']==z['seated'] and abs(a['distance_mm']-z['distance_mm'])<=1e-5 and abs(a['foot_area_mm2']-z['foot_area_mm2'])<=1e-5 and witnesses
        contact_checks.append(dict(module=a['module'],matched=matched,native=a))
        if not matched:differences.append('contact_'+a['module'])
    clean=not git('status','--porcelain=v1')
    report=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,
        differences=differences,key_samples=samples,rigid_source_step_signatures=rebuilding,floor_contact_checks=contact_checks,
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,
        independent_audit=False,changed_parts=24,retained_parts=329,hand_parts=364,
        signature_absolute_tolerance=1e-5,
        signature_scope='All24rigid transforms from saved old parts compared by validity,solid count,volume/area/centres/bounds,allface/edge types and metrics,and vertices. Geometric signatures,not coincident-solid Boolean equivalence proof.',
        scope='Original8fixed palm clashes,finalopen/combined endpoints,other329packaging/guide,betweenmovedmodules,48leads/16plug envelopes,neutral10arm,firstalternative4failedpairs,and3feet/15inside-outside witnesses. Other3endpoints and images not rerun. No insertion/fasteners/tools/strength/coupling/continuous or electrical/physical approval.')
    p.write(args.output,report)
    if not report['passed']:raise ValueError('Exact-SHA replay differs')


if __name__=='__main__':main()

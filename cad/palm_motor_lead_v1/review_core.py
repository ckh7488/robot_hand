"""Replay scoped palm-lead evidence from an unchanged exact-commit checkout."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys

import cadquery as cq
import numpy as np
from OCP.OSD import OSD_ThreadPool

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(ROOT):raise ValueError('New external output required')
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=args.commit or git('status','--porcelain=v1'):raise ValueError('Wrong/dirty target')
    g=module('_palm_lead_review',HERE/'model.py');c,b,m,p=g.c,g.b,g.m,g.p
    sig=module('_palm_wire_analytic',ROOT/'cad/remote_motor_lead_v1/roundtrip_review.py').signatures
    manifest=p.read(ROOT/'checkpoints/PALMLEAD016.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(b.sha(ROOT/z)==r['sha256'] for z,r in manifest['files'].items()) and all(b.sha(ROOT/z)==h for z,h in frozen.items())
    if not guard():raise ValueError('Changed input closure')
    expected=p.read(HERE/'reviews/assessment.json');assert expected['completed'] and expected['input_hash_guard'] and not expected['nominal_leads_clear']
    saved=p.read(HERE/'generated/scene.json');rows=c.load(saved['parts'])
    wires=[z for z in rows if z['role']=='provisional_electrical_lead_space'];plugs=[z for z in rows if z not in wires]
    motors=[z for z in p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')['modules'] if z['zone']=='palm']
    fresh={r['name']:r for motor in motors for r in g.wires(motor,**expected['candidate_parameters'])}
    fresh.update({r['name']:r for r in g.plugs(motors)})
    differences=[];analytic=[];probes=[]
    for row in rows:
        shape=fresh[row['name']]['shape'];a,z=sig(shape,m.pack._bbox),sig(row['shape'],m.pack._bbox)
        bad=[];numeric=[]
        if not a['valid'] or not z['valid'] or a['solids']!=1 or z['solids']!=1:bad.append('valid/solid')
        for key in ['global_values','vertices']:
            aa,zz=np.asarray(a[key]),np.asarray(z[key])
            if aa.shape!=zz.shape or not np.allclose(aa,zz,rtol=0,atol=1e-5):bad.append(key)
            else:numeric.extend(abs(aa-zz).ravel().tolist())
        for key in ['faces','edges']:
            if len(a[key])!=len(z[key]):bad.append(key+' count');continue
            for aa,zz in zip(a[key],z[key]):
                if aa['kind']!=zz['kind'] or len(aa['values'])!=len(zz['values']) or not np.allclose(aa['values'],zz['values'],rtol=0,atol=1e-5):bad.append(key+' analytic')
                else:numeric.extend(abs(np.asarray(aa['values'])-np.asarray(zz['values'])).tolist())
        states=[]
        if row in wires:
            local=row['local'];T=np.asarray(row['transform_world_mm'])
            positions=[np.asarray(local['arc_middle_mm']),(np.asarray(local['arc_end_mm'])+local['end_mm'])/2]
            if local['straight_stub_mm']>0:positions.insert(0,(np.asarray(local['start_mm'])+local['arc_start_mm'])/2)
            for point in positions:
                world=(T@np.r_[point,1])[:3]
                state=[bool(s.isInside(cq.Vector(*world),1e-6)) for s in [shape,row['shape']]];states.append(state)
                if state!=[True,True]:bad.append('span interior')
        analytic.append(dict(name=row['name'],matched=not bad,mismatches=bad,max_numeric_difference=max(numeric or [0.]),
            face_count=len(a['faces']),edge_count=len(a['edges']),span_interior_states=states))
        if bad:differences.append('analytic_'+row['name'])
    old=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json');pack=c.load(old['new_parts']+old['retained_parts'])
    remote=c.load(p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'])
    scene=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    fixedpalm=c.load([z for z in scene['hand'] if z['body']=='palm'])
    def compare(label,got,want):
        def volumes(r):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in r['overlaps']}
        a,z=volumes(got),volumes(want)
        matched=a.keys()==z.keys() and all(abs(a[k]-z[k])<=1e-5 for k in a.keys()&z.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        probes.append(dict(id=label,matched=matched,native=got))
        if not matched:differences.append(label)
        print(label,matched,flush=True)
    compare('360_current_packaging',c.material(wires,pack),expected['packaging'])
    compare('fixed_palm_only',c.material(wires,fixedpalm),expected['fixed_palm'])
    compare('48_remote_leads_16_outerboxes',c.material(wires,remote),expected['existing_wiring'])
    compare('66_palm_wire_pairs',g.pairs(wires),expected['new_wire_pairs'])
    compare('four_palm_outerboxes',c.material(wires,plugs),expected['new_plug_boxes'])
    arm=[dict(name=z['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(z['asset']+'.step'))).val())
         for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
    compare('neutral_arm10',c.material(wires,arm),expected['neutral_arm'])
    baseline=[r for motor in motors for r in g.wires(motor,radius=10.,stub=5.,straight=18.,angle=0.)]
    compare('preserved_remote_copy_packaging',c.material(baseline,pack),expected['remote_copy_packaging'])
    compare('preserved_remote_copy_floor',c.material(baseline,fixedpalm),expected['remote_copy_fixed_palm'])
    original=p.read(HERE/'reviews/check.json');first=original['screens'][0]
    motor=next(z for z in motors if z['name']==first['module'])
    compare('preserved_R6_carrier_obstruction',c.material(g.wires(motor,**first['parameters']),pack),first['checks']['packaging'])
    compare('unresolved_outerbox_packaging',c.material(plugs,pack),expected['plug_vs_packaging'])
    compare('outerbox_fixed_palm',c.material(plugs,fixedpalm),expected['plug_vs_fixed_palm'])
    clean=not git('status','--porcelain=v1')
    result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,
        differences=differences,key_samples=probes,analytic_comparisons=analytic,manifest_files=len(manifest['files']),frozen_files=len(frozen),
        input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,
        absolute_analytic_tolerance=1e-5,fixed_palm_parts_imported=len(fixedpalm),hand_parts_hashed=364,
        scope='16source/STEP analytic surfaces/curves/vertices/properties;36wire-span interior checks;all12unadopted tails and static comparisons,including copied-route failures and unresolved solid plug boxes.Only fixed palm components imported;364hand input hashes checked.No full hand endpoints/motion,supplier radius,mating,fixture,axial restraint,bus,electrical or physical approval.')
    p.write(args.output,result)
    if not result['passed']:raise ValueError('Palm lead replay differs')


if __name__=='__main__':main()

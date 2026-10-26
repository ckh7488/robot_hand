"""Exact-SHA guide geometry, material, seating and sampled service replay."""
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
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(ROOT):
        raise ValueError('New external review output required')
    def git(*xs):
        return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=args.commit or git('status','--porcelain=v1'):
        raise ValueError('Wrong/dirty target')
    spec=importlib.util.spec_from_file_location('_guide_replay',HERE/'model.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
    b,m,p=g.b,g.m,g.p
    analytic=g.t.module('_guide_analytic',ROOT/'cad/remote_motor_lead_v1/roundtrip_review.py')
    service=g.t.module('_guide_service_replay',HERE/'service_check.py')
    order=g.t.module('_guide_order_replay',HERE/'assembly_order_v2.py')
    manifest=p.read(ROOT/'checkpoints/WIRE011.manifest.json')
    frozen={}
    for key,value in p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json').items():
        if key in ['geometry_sha256','code_sha256','report_sha256']:
            frozen.update(value)
    def guard():
        return all(b.sha(ROOT/r)==row['sha256'] for r,row in manifest['files'].items()) and all(b.sha(ROOT/r)==h for r,h in frozen.items())
    if not guard():
        raise ValueError('Changed source/frozen evidence')
    expected=p.read(HERE/'reviews/check.json');expected_service=p.read(HERE/'reviews/service.json');expected_order=p.read(HERE/'reviews/assembly_order_v2.json')
    if not expected['completed'] or not expected['final_nominal_clear']:
        raise ValueError('No final nominal guide candidate')
    saved=p.read(HERE/'generated/scene.json')
    new=service.load(saved['new_parts']);retained=service.load(saved['retained_packaging'])
    wiresmeta=p.read(ROOT/saved['wire_scene'])['parts'];wires=service.load(wiresmeta)
    onlywires=[r for r in wires if r['role']=='nominal_electrical_wire_space'];plugs=[r for r in wires if r['role']!='nominal_electrical_wire_space']
    assert len(new)==160 and len(retained)==193 and len(onlywires)==48 and len(plugs)==16
    differences=[];rebuild=[]
    freshmap={r['name']:r for r in g.build()[0]}
    for row in new:
        a=analytic.signatures(freshmap[row['name']]['shape'],m.pack._bbox);bx=analytic.signatures(row['shape'],m.pack._bbox)
        bad=[];values=[]
        if not a['valid'] or not bx['valid'] or a['solids']!=1 or bx['solids']!=1:
            bad.append('valid single solid')
        for key in ['global_values','vertices']:
            aa,bb=np.asarray(a[key]),np.asarray(bx[key])
            if aa.shape!=bb.shape or not np.allclose(aa,bb,rtol=0,atol=1e-5):bad.append(key)
            else:values.extend(abs(aa-bb).ravel().tolist())
        for key in ['faces','edges']:
            if len(a[key])!=len(bx[key]):bad.append(key+' count');continue
            for aa,bb in zip(a[key],bx[key]):
                if aa['kind']!=bb['kind'] or len(aa['values'])!=len(bb['values']) or not np.allclose(aa['values'],bb['values'],rtol=0,atol=1e-5):bad.append(key+' analytic parameters')
                else:values.extend(abs(np.asarray(aa['values'])-np.asarray(bb['values'])).tolist())
        rebuild.append(dict(name=row['name'],matched=not bad,max_numeric_difference=max(values),mismatch=bad,
            face_count=len(a['faces']),edge_count=len(a['edges']),vertex_count=len(a['vertices'])))
        if bad:differences.append(dict(id=row['name'],reason=bad))
    native,v,cat,scene,hand=p.load()
    def compare(label,got,want):
        signature=lambda r:{(x['lead'],x['part']):x['material_intersection_mm3'] for x in r['overlaps']}
        a,bx=signature(got),signature(want)
        if a.keys()!=bx.keys() or any(abs(a[k]-bx[k])>1e-5 for k in a.keys() & bx.keys()) or len(got['operation_failures'])!=len(want['operation_failures']):
            differences.append(dict(id=label,reason='Material overlaps or uncertainty changed'))
    final=dict(new_vs_retained=b.test_against(new,retained,v),new_pairs=g.t.old.wire_pairs(new,v),
        new_vs_wires=b.test_against(new,onlywires,v),new_vs_plug_envelopes=b.test_against(new,plugs,v))
    for key,value in final.items():compare(key,value,expected['final_checks'][key])
    mounts=[];supports=[];newmap={r['name']:r for r in new};oldmap={r['name']:r for r in retained}
    for group,want in zip(saved['groups'],expected['nominal_mount_checks']):
        name=group['module'];base=newmap[name+'_wire_guide_base']['shape'];adapter=oldmap[name+'_adapter']['shape']
        witnesses=[]
        for xy in group['bolt_axis_xy_mm']:
            for dx,dy in [(2.3,0),(-2.3,0),(0,2.3),(0,-2.3),(2.3,2.3),(-2.3,-2.3)]:
                witnesses.append([BRepClass3d_SolidClassifier(s.Solids()[0].wrapped,gp_Pnt(xy[0]+dx,xy[1]+dy,z),1e-6).State()==TopAbs_IN
                    for s,z in [(base,-79.05),(adapter,-78.95)]])
        gap=float(base.distance(adapter));passed=all(all(r) for r in witnesses) and gap<=1e-5
        mounts.append(dict(module=name,base_adapter_gap_mm=gap,passed=passed,bearing_witness_states=witnesses))
        if passed!=want['passed'] or abs(gap-want['base_adapter_gap_mm'])>1e-5:differences.append(dict(id=name,reason='Nominal mounting contact changed'))
        for wire in [r for r in onlywires if r['module']==name]:
            distance=float(wire['shape'].distance(base));supports.append(dict(wire=wire['name'],base_gap_mm=distance))
            want_support=next(r for r in expected['nominal_wire_support'] if r['wire']==wire['name'])
            if abs(distance-want_support['base_gap_mm'])>1e-5:differences.append(dict(id=wire['name'],reason='Nominal support contact changed'))
    neutral=v.frames(cat['design'],np.zeros(20));handchecks=[]
    for case in expected['hand_pose_checks']:
        frames=v.frames(cat['design'],case['angles_deg'])
        current=[dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
        got=b.test_against(new,current,v);compare(case['id'],got,case['native']);handchecks.append(dict(id=case['id'],native=got))
    arm=[]
    for row in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        arm.append(dict(name=row['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(row['asset']+'.step'))).val()))
    armcheck=b.test_against(new,arm,v);compare('neutral_arm',armcheck,expected['neutral_arm_check'])
    left={r['module'] for r in saved['groups'] if r['motor_centre_mm'][0]<0}
    right={r['module'] for r in saved['groups'] if r['motor_centre_mm'][0]>0}
    bases=[r for r in new if r['kind']=='guide_base']
    probes=[('lid_screw_removal',4,[r for r in new if r['kind']=='M2x6_envelope'],[r for r in new if r['kind']!='M2x6_envelope']+retained+wires+arm),
        ('lid_removal_with_neutral_arm',4,[r for r in new if r['kind']=='removable_lid'],[r for r in new if r['kind'] not in ['removable_lid','M2x6_envelope']]+retained+wires+arm),
        ('M3_insertion',2,[r for r in new if r['name']=='index_spread_actuator_rack_screw_1'],[r for r in new if r['kind']!='replacement_M3x12_envelope']+retained+wires+arm),
        ('full_population_base_attempt',2,[r for r in bases if r['module']=='index_dip_actuator'],retained+arm),
        ('right_base_staged',0,[r for r in bases if r['module'] in right],[r for r in retained if r.get('module') not in left]+arm),
        ('left_base_staged',0,[r for r in bases if r['module'] in left],retained+[r for r in new if r['module'] in right]+arm)]
    paths=[]
    for key,index,moving,fixed in probes:
        want=expected_service[key][index];vector=want['translation_mm']
        got=b.test_against([dict(name=r['name'],shape=r['shape'].translate(vector)) for r in moving],fixed,v)
        moving_names={r['name'] for r in moving}
        want_native=dict(want['native'],overlaps=[r for r in want['native']['overlaps'] if r['lead'] in moving_names],
            operation_failures=[r for r in want['native']['operation_failures'] if r['lead'] in moving_names])
        compare(key+str(index),got,want_native);paths.append(dict(path=key,translation_mm=vector,moving_names=sorted(moving_names),native=got))
    conditional=[]
    for key,moving,fixed,positions in order.configurations(saved,new,retained,wires,arm):
        if key not in ['right_lift','right_shift_y','M3_before_module_base_hardware','module_base_screws_vs_new_only','module_base_nuts_vs_new_only']:continue
        index=3 if key=='right_lift' else 4 if key=='right_shift_y' else 2
        vector=positions[index];want=expected_order['paths'][key][index]
        got=b.test_against([dict(name=r['name'],shape=r['shape'].translate(vector)) for r in moving],fixed,v)
        compare('order_'+key+str(index),got,want['native']);conditional.append(dict(path=key,translation_mm=vector,native=got))
    clean=not git('status','--porcelain=v1')
    report=dict(target_commit=args.commit,passed=not differences and clean and guard(),clean_checkout=clean,input_hash_guard=guard(),
        manifest_files=len(manifest['files']),frozen_files=len(frozen),differences=differences,analytic_source_roundtrip=rebuild,
        final_checks=final,nominal_mount_checks=mounts,nominal_wire_support=supports,hand_pose_checks=handchecks,
        neutral_arm_check=armcheck,service_paths=paths,conditional_order_paths=conditional,independent_audit=False,
        scope='160new parts rebuilt/reimported;new material,16bearing seats/48wire contacts,5hand endpoints,neutral10arm and11key original/conditional service samples reproduced. Original/alternative service studies not fully rerun. Later factory M2 paths compared to NEW parts only. No fixtures/full module assembly,axial strain relief,tool/torque/preload,continuous motion or hardware approval.')
    p.write(args.output,report)
    if not report['passed']:raise ValueError('Exact-SHA guide core differs')


if __name__=='__main__':
    main()

"""Exact-commit replay of nominal mount geometry, contacts and assembly probes."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys

import cadquery as cq
import numpy as np
from cadquery.occ_impl.shapes import setThreads
from OCP.OSD import OSD_ThreadPool

setThreads(1)
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
    f=module('_mount_core',HERE/'selected3/check.py');g,c,p,b,m=f.g,f.c,f.p,f.b,f.m
    oldf=module('_mount_original',HERE/'check.py')
    sig=module('_mount_signatures',ROOT/'cad/palm_motor_layout_v1/review_core.py').signature
    manifest=p.read(ROOT/'checkpoints/MOUNT015.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(b.sha(ROOT/z)==r['sha256'] for z,r in manifest['files'].items()) and all(b.sha(ROOT/z)==h for z,h in frozen.items())
    if not guard():raise ValueError('Changed manifest/frozen input')
    expected=p.read(HERE/'selected3/reviews/check.json');initial=p.read(HERE/'reviews/check.json')
    assert expected['completed'] and expected['input_hash_guard'] and not initial['completed']
    saved=p.read(HERE/'selected3/generated/scene.json');new=c.load(saved['new_parts']);retained=c.load(saved['retained_parts'])
    adapters=c.load(saved['replaced_adapters']);fresh,_=g.build(adapters);freshmap={r['name']:r['shape'] for r in fresh}
    differences=[];signatures=[];probes=[]
    for row in new:
        a,z=sig(freshmap[row['name']],m.pack._bbox),sig(row['shape'],m.pack._bbox);bad=[];errors=[]
        if not a['valid'] or not z['valid'] or a['solids']!=z['solids']:bad.append('valid/solids')
        for key in ['global_values','vertices']:
            aa,zz=np.asarray(a[key]),np.asarray(z[key])
            if aa.shape!=zz.shape or not np.allclose(aa,zz,rtol=0,atol=1e-5):bad.append(key)
            else:errors.extend(abs(aa-zz).ravel().tolist())
        for key in ['faces','edges']:
            if len(a[key])!=len(z[key]):bad.append(key+' count');continue
            for aa,zz in zip(a[key],z[key]):
                if aa[0]!=zz[0] or not np.allclose(aa[1],zz[1],rtol=0,atol=1e-5):bad.append(key+' signature')
                else:errors.extend(abs(np.asarray(aa[1])-np.asarray(zz[1])).tolist())
        signatures.append(dict(name=row['name'],matched=not bad,mismatch=bad,max_numeric_difference=max(errors or [0.])))
        if bad:differences.append('source_step_'+row['name'])
    _,v,cat,scene,hand=p.load();neutral=v.frames(cat['design'],np.zeros(20))
    def posed(q):
        frames=v.frames(cat['design'],q)
        return {r['name']:v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']]) for r in scene['hand']}
    def compare(label,got,want):
        def values(r):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in r['overlaps']}
        a,z=values(got),values(want)
        matched=a.keys()==z.keys() and all(abs(a[k]-z[k])<=1e-5 for k in a.keys()&z.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        probes.append(dict(id=label,native=got,matched=matched))
        if not matched:differences.append(label)
        print(label,matched,flush=True)
    for key in ['open','combined']:
        row=next(r for r in expected['cases'] if r['id']==key);hh=posed(row['angles_deg'])
        compare('final_'+key,c.material(new,[dict(name=n,shape=s) for n,s in hh.items()]),row['native'])
    compare('retained349',c.material(new,retained),expected['retained_packaging'])
    results=[c.material([r],new[i+1:]) for i,r in enumerate(new) if i+1<len(new)]
    pair=dict(pairs_in_scope=sum(r['pairs_in_scope'] for r in results),exact_pairs=sum(r['exact_pairs'] for r in results),
              overlaps=[z for r in results for z in r['overlaps']],operation_failures=[z for r in results for z in r['operation_failures']])
    compare('new55pairs',pair,expected['new_pairs'])
    wires=c.load(p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'])
    compare('48wires_16plug_envelopes',c.material(new,wires),expected['wires_and_plug_envelopes'])
    arm=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(r['asset']+'.step'))).val())
         for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
    compare('neutral10arm',c.material(new,arm),expected['neutral_arm'])
    layout=p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    hh=posed(expected['assembly_hand_angles_deg'])
    configs={r['id']:r for r in f.configurations(new,retained,scene['hand'],hh,saved['groups'],saved['params'],layout)}
    keys=[('bench_unit_middle_mcp_actuator',0),('bench_unit_ring_mcp_actuator',2),('bench_unit_thumb_base_sweep_actuator',2),('bench_unit_thumb_opposition_actuator',2),
          ('bench_base_nuts',2),('populated_carrier_down_into_palm',0),('populated_carrier_down_into_palm',7),('populated_carrier_down_into_palm',13),('populated_carrier_down_into_palm',14),
          ('cap_Palm_collar_1_cap',3),('cap_Palm_collar_2_cap',3),('clamp_nuts_from_above',2),('clamp_screws_from_north',2),('declared_tool_stems',2)]
    for key,index in keys:
        conf=configs[key];record=expected['paths'][key]
        assert record['moving']==[r['name'] for r in conf['moving']] and record['obstacles']==[r['name'] for r in conf['fixed']]
        compare(key+'_'+str(index),c.material(f.moved(conf,index),conf['fixed']),record['samples'][index]['native'])
    for key,index in [('clamp_nuts_from_above',3),('declared_tool_stems',0)]:
        conf=configs[key]
        compare('preserved_failed_'+key,c.material(f.moved(conf,index),conf['fixed']),expected['paths'][key]['samples'][index]['native'])
    correction=module('_mount_corrected_service',HERE/'service_correction.py')
    corrected=p.read(HERE/'reviews/final_service.json')
    assert corrected['completed'] and corrected['input_hash_guard'] and corrected['selected_sampled_order_clear']
    three,side,tool,_=correction.configurations(new,retained,scene['hand'],hh,saved['groups'],saved['params'],layout)
    added={r['id']:r for r in [three,side,tool]}
    extra=[('three_nuts_offset_drop_then_seat',4),('three_nuts_offset_drop_then_seat',10),
           ('one_nut_side_then_south',0),('one_nut_side_then_south',4),('one_nut_side_then_south',8)]
    if corrected['tool_entry_changed']:extra.append(('tool_stems_short_entry',0))
    for key,index in extra:
        conf=added[key];record=corrected['paths'][key]
        assert record['moving']==[r['name'] for r in conf['moving']] and record['obstacles']==[r['name'] for r in conf['fixed']]
        compare('corrected_'+key+'_'+str(index),c.material(f.moved(conf,index),conf['fixed']),record['samples'][index]['native'])
    oldsaved=p.read(HERE/'generated/scene.json');oldnew=c.load(oldsaved['new_parts'])
    om={r['name']:r for r in oldnew}
    overlap=initial['new_pairs']['overlaps'][0]
    compare('original_cap_carrier_overlap',c.material([om[overlap['lead']]],[om[overlap['part']]]),dict(pairs_in_scope=1,overlaps=[overlap],operation_failures=[]))
    oldconfigs={r['id']:r for r in oldf.configurations(oldnew,retained,scene['hand'],hand,oldsaved['groups'],oldsaved['params'],layout)}
    for key,index in [('bench_unit_middle_mcp_actuator',0),('populated_carrier_down_into_palm',0)]:
        conf=oldconfigs[key]
        compare('original_'+key,c.material(oldf.moved(conf,index),conf['fixed']),initial['paths'][key]['samples'][index]['native'])
    contacts=f.contacts(new,hand['Palm_tapered_chassis'],saved['params'])
    contactmatch=all(a['states']==z['states'] and a['matched']==z['matched'] and np.allclose(a['inner_mm'],z['inner_mm'],rtol=0,atol=1e-5) and np.allclose(a['outer_mm'],z['outer_mm'],rtol=0,atol=1e-5)
                     for a,z in zip(contacts,expected['post_contacts']))
    if not contactmatch:differences.append('post_contacts')
    floor=next(r['shape'] for r in new if r['kind']=='four_adapter_carrier').distance(hand['Palm_tapered_chassis'])
    if abs(floor-expected['carrier_floor_distance_mm'])>1e-5:differences.append('floor_distance')
    clean=not git('status','--porcelain=v1')
    report=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,
        key_samples=probes,source_step_signatures=signatures,post_contacts=contacts,post_contacts_matched=contactmatch,carrier_floor_distance_mm=floor,
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        independent_audit=False,scope='Native key comparisons include14original service samples,6final comparisons,3first-candidate failures,2preserved failed nut/tool entries and selected corrected nut/tool samples;11source-STEP signatures;8post witnesses/floor distance. Other3endpoints,remaining intermediate samples and renders not rerun. No thread/torque/friction/strength/fulltool/fixture/coupling/fullrobot/electrical/physical approval.')
    p.write(args.output,report)
    if not report['passed']:raise ValueError('Exact-SHA core differs')


if __name__=='__main__':main()

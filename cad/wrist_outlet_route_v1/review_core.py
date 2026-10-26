"""Scoped exact-checkout replay of the local guide correction and failures."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import inboard_model as a
from outlet_survey import m,g,c,p,np,cq,ROOT,HERE
round_model=m.drive.module('_outlet029_review_round',HERE/'round_exit_model.py');s_model=m.drive.module('_outlet029_review_s',HERE/'s_exit_model.py');base=m.drive.module('_outlet029_review_inputs',ROOT/'cad/long_housing_route_v1/build_check.py');metrics=m.drive.module('_outlet029_metric_helpers',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool

def combined(xs):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in xs),overlaps=[v for z in xs for v in z['overlaps']],operation_failures=[v for z in xs for v in z['operation_failures']])
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1');manifest=p.read(ROOT/'checkpoints/OUTLET029.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();selected=p.read(HERE/'inboard/reviews/final.json');service=p.read(HERE/'inboard/reviews/service.json');new,retained,design=a.build();saved=c.load(p.read(ROOT/selected['scene'])['parts']);byname={z['name']:z for z in new};geo=[];differences=[];native=[]
    def geometry(fresh,stored):
        got=dict(metrics.match(metrics.metrics(fresh['shape']),metrics.metrics(stored['shape'])),name=stored['name']);geo.append(got)
        if not got['matched']:differences.append('geometry_'+stored['name'])
    for z in saved:geometry(byname[z['name']],z)
    probe=p.read(HERE/'reviews/probe.json');scheck=p.read(HERE/'selected/reviews/check.json');failed=[]
    for sign,item,name in [(1,probe['alternatives'][0],'left_exit_housing_0_2'),(-1,probe['alternatives'][1],'right_exit_housing_0_1')]:
        fresh,_=round_model.build(sign);stored=c.load(p.read(ROOT/item['scene'])['parts']);geometry(next(z for z in fresh if z['name']==name),next(z for z in stored if z['name']==name));failed.append((item['id'],item['cases'][0],stored,item['internal']))
    s_case=scheck['cases'][0];s_rows=c.load(p.read(ROOT/s_case['scene'])['parts']);fresh,_=s_model.build(s_case['angles_deg']);name='s_exit_housing_1_3';geometry(next(z for z in fresh if z['name']==name),next(z for z in s_rows if z['name']==name));failed.append(('S',s_case,s_rows,s_case['internal']))
    data,pack,guide=base.inputs();old_hardware=c.load([z for z in p.read(ROOT/'cad/wrist_housing_guide_v1/selected2/generated/parts/scene.json')['parts'] if z['installed_part_candidate']]);rows=saved+retained
    def compare(label,got,want):
        def mapping(x):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']}
        aa,bb=mapping(got),mapping(want);ok=aa.keys()==bb.keys() and all(abs(aa[k]-bb[k])<1e-5 for k in aa.keys()&bb.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    for label,case,parts,internal in failed:
        hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
        for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_guide_hardware',old_hardware)]:compare(label+'_'+key,c.material(parts,ob),case['native'][key])
        compare(label+'_internal',combined([c.material([z],parts[i+1:]) for i,z in enumerate(parts)]),combined(internal))
    for case in [selected['cases'][0],selected['cases'][-1]]:
        hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
        for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]:compare('inboard_'+case['id']+'_'+key,c.material(rows,ob),case['native'][key])
    compare('inboard_internal45',combined([c.material([z],rows[i+1:]) for i,z in enumerate(rows)]),combined(selected['internal']))
    hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));fixed=hand+pack+data[6]+data[7];installed=[]
    for i,path in enumerate(service['paths']):
        part=next(z for z in rows if z['name']==path['name']);assert path['installed_before']==[z['name'] for z in installed]
        for j in [0,4]:sample=path['samples'][j];compare('service_'+str(i)+'_'+str(j),c.material([dict(part,shape=part['shape'].translate(tuple(sample['translation_mm'])))],fixed+installed),sample['native'])
        installed.append(part)
    bound=selected['carrier_rotation_bound'];O=np.asarray(bound['axis_origin_mm']);envelope=cq.Solid.makeCylinder(18,60,cq.Vector(O[0],O[1],-20),cq.Vector(0,0,1));cat=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');carrier=c.load([next(z for z in cat['hand'] if z['name']=='thumb_root_spread_carrier')])[0]['shape'];outside=carrier.cut(envelope,tol=1e-6);volume=outside.Volume() if outside.Solids() else 0.;bound_matched=abs(volume-bound['carrier_material_outside_mm3'])<1e-8
    if not bound_matched:differences.append('carrier_containment')
    compare('root_yaw_bound',c.material(saved,[dict(name='carrier_R18_conservative_axis_space',shape=envelope)]),bound['new_parts_vs_envelope']);clip=next(z['shape'] for z in saved if z['name']=='wrist_inboard_guide_clip')
    for j,case in enumerate(service['radial_stops']):
        z=next(z for z in saved if z['name']=='inboard_local_housing_'+str(case['line']));compare('radial_'+str(j),c.material([dict(z,shape=z['shape'].translate(tuple(case['translation_mm'])))],[dict(name='actual_inboard_bore',shape=clip)]),case['native'])
    saddle=next(z['shape'] for z in data[7] if z['name']=='Palm_wrist_saddle');cap=next(z['shape'] for z in retained if z['name']=='wrist_edge_clip_front_cap');contacts=[]
    for case in service['contacts']:
        first=cap if case['id']=='front_face' else clip;pt=np.asarray(case['point_mm']);n=np.asarray(case['normal']);dist=[sh.distance(cq.Vertex.makeVertex(*pt)) for sh in [first,saddle]];material=[[sh.isInside(cq.Vector(*(pt+s*.05*n)),1e-6) for s in [-1,1]] for sh in [first,saddle]];ok=max(abs(np.asarray(dist)-case['distance_mm']))<1e-5 and material==case['material'];contacts.append(dict(id=case['id'],matched=bool(ok),distance_mm=dist,material=material))
        if not ok:differences.append('contact_'+case['id'])
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geo,carrier_containment_volume_mm3=volume,carrier_containment_matched=bound_matched,contacts=contacts,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),whole_hand_drive_approved=False,independent_audit=False,inherited_OFFSET024_face_comparison_inconclusive=True,inherited_HOUSING027_initial_core_passed=False,scope='44nativegroups:3failed alternatives5obstacle+internal each;inboardopen/fullparent8groups+internal45;6hardwarepaths start/end12of30;1rootcarrier bound/containment+4radialstops;3seatcontacts.8source/STEP adaptive metric signatures abs1e-5:5newinboard+3selectedfailedtubes.Not all49geometry signatures,remaining6static/middle18service/sections/render,continuousfullhand/wrist/wholehousing/threading/length/tension/load/physicalapproval.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()

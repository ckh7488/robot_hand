"""Exact-SHA scoped repeatability, not a whole-hand or load approval."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import final_edge_model as a
from wrist_survey import base
from guide_service import sequence
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
metrics=m.drive.module('_wrist028_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool

def combined(xs):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in xs),overlaps=[v for z in xs for v in z['overlaps']],operation_failures=[v for z in xs for v in z['operation_failures']])

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/WRIST028.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();selected=p.read(HERE/'selected2/reviews/check.json');service=p.read(HERE/'selected2/reviews/service.json');fresh,design=a.build();saved=c.load(p.read(ROOT/selected['scene'])['parts']);lookup={z['name']:z for z in fresh};differences=[];geo=[];native=[]
    for z in saved:
        got=dict(metrics.match(metrics.metrics(lookup[z['name']]['shape']),metrics.metrics(z['shape'])),name=z['name']);geo.append(got)
        if not got['matched']:differences.append('geometry_'+z['name'])
    data,pack,guide=base.inputs()
    def compare(label,got,want):
        def mapping(x):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']}
        aa,bb=mapping(got),mapping(want);ok=aa.keys()==bb.keys() and all(abs(aa[k]-bb[k])<1e-5 for k in aa.keys()&bb.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    compare('new_internal45',combined([c.material([z],saved[i+1:]) for i,z in enumerate(saved)]),combined(selected['internal']))
    for case in [selected['cases'][0],selected['cases'][-1]]:
        hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
        for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]:compare('selected_'+case['id']+'_'+key,c.material(saved,ob),case['native'][key])
    follow=p.read(HERE/'selected2/reviews/hardware_follow.json');case=follow['new_cases'][-1];hardware=[z for z in saved if z['installed_part_candidate']];hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
    for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]:compare('hardware_fullparent_'+key,c.material(hardware,ob),case['native'][key])
    hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));survey=p.read(HERE/'reviews/survey.json');probes=c.load(p.read(ROOT/survey['probe_scene'])['parts'])
    for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]:compare('straight_probes_'+key,c.material(probes,ob),survey['native'][key])
    raw=p.read(HERE/'reviews/check.json');first=c.load(p.read(ROOT/raw['scene'])['parts']);compare('first_upper_cap_failure',c.material(first,hand),raw['cases'][0]['native']['hand'])
    middle=p.read(HERE/'selected/reviews/check.json');second=c.load(p.read(ROOT/middle['scene'])['parts']);compare('second_lower_cap_palm_failure',c.material(second,hand),middle['cases'][0]['native']['hand'])
    installed=[];fixed=hand+pack+data[6]+data[7]
    for j,case in enumerate(service['paths']):
        part=next(z for z in saved if z['name']==case['name']);assert case['installed_before']==[z['name'] for z in installed]
        for index in sorted({0,len(case['samples'])-1}):
            sample=case['samples'][index];compare('service_'+str(j)+'_'+str(index),c.material([dict(part,shape=part['shape'].translate(tuple(sample['translation_mm'])))],fixed+installed),sample['native'])
        installed.append(part)
    clip=next(z for z in saved if z['name']=='wrist_oblique_guide_clip')['shape'];cap=next(z for z in saved if z['name']=='wrist_edge_clip_front_cap')['shape'];saddle=next(z['shape'] for z in data[7] if z['name']=='Palm_wrist_saddle');contacts=[]
    for case in service['seat_contacts']:
        sh=cap if case['id']=='front_face' else clip;pt=np.asarray(case['point_mm']);n=np.asarray(case['normal']);dist=[z.distance(cq.Vertex.makeVertex(*pt)) for z in [sh,saddle]];material=[[z.isInside(cq.Vector(*(pt+s*.05*n)),1e-6) for s in [-1,1]] for z in [sh,saddle]];ok=max(abs(np.asarray(dist)-case['distance_mm']))<1e-5 and material==case['material_at_minus_plus_0p05_mm'];contacts.append(dict(id=case['id'],distance_mm=dist,material=material,matched=bool(ok)))
        if not ok:differences.append('contact_'+case['id'])
    for j,case in enumerate(service['radial_stop_probes']):
        z=next(z for z in saved if z['name']=='wrist_local_housing_'+str(case['line']));compare('radial_stop_'+str(j),c.material([dict(z,shape=z['shape'].translate(tuple(case['normal_displacement_mm'])))],[dict(name='actual_guide_clip',shape=clip)]),case['native'])
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geo,seat_contacts=contacts,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),inherited_OFFSET024_face_comparison_inconclusive=True,inherited_HOUSING027_initial_core_passed=False,inherited_HOUSING027_precision_pair_passed=True,independent_audit=False,whole_hand_drive_approved=False,scope='10new source/STEP adaptive global and face/edge/vertex metric signatures abs1e-5;all10newshapes open/blockedparentquarter2of5poses,6hardware fullparent1of3followposes;all6hardwarepaths start/end12of30;originalstraight and bothcap failures;3seats/4radialstop probes.35nativegroups repeatability only.No allsurface equations/equivalence proof,no earlier OFFSET024/HOU027 replay,no remainingstatic/18middleservice/render,fullhousingconnection/threading/wristmotion/tension/friction/preload/load/manufacturing or physical approval.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()

"""Unique opaque CAD images of original kit assembly and complete carrier entry."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm073_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm073_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();raw=n.p.read(HERE/'reviews/check.json');assert raw['completed']
    source=dict(raw['source_sha256']);meta,matrix=old.metadata();meta={z['name']:z for z in meta};queries=[];common_scene=None;commonmeta=[]
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        return n.g.original.transform(n.c.load([z],source)[0]['shape'],matrix(name))
    key=raw.get('first_blocked_sample')
    if key and raw['native'][key]['overlaps']:
        p=next(p for p in raw['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in p['samples'] if s['id']==key);q=max(raw['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3']);M=np.array(s['transform'])
        sh=n.g.original.transform(native(q['lead']),M).intersect(native(q['part']),tol=1e-6);assert sh.isValid() and sh.Solids()
        name='arm073_first_new_service_common_0';queries=[dict(name=name,id=key,lead=q['lead'],part=q['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=q['material_intersection_mm3'])]
        n.a.save(HERE/'reviews/common_prepared.json',dict(completed=False,common_queries=queries,new_common_queries=1,common_scene=None))
        common_scene=n.a.a.prior.export([dict(name=name,shape=sh,body='forearm',evidence_only=True,color=[.9,.1,.08])],HERE/'generated/actual_common');commonmeta=n.p.read(ROOT/common_scene)['parts']
    n.a.save(HERE/'reviews/common_prepared.json',dict(completed=True,common_queries=queries,new_common_queries=len(queries),common_scene=common_scene))
    def scene(names,poses=None,extras=None,bright=None):
        parts=[];poses=poses or {};bright=set(bright or [])
        for name in sorted(set(names)):
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@matrix(name),color=[.95,.55,.1] if name in bright else z.get('color',[.3,.5,.75])))
        for z in extras or []:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.eye(4),color=[.9,.1,.08]))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def save_sheet(panels,cols,size,title,foot,file):
        target=HERE/'images'/file;assert not target.exists();v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.)
        r.sheet([(label,render(s,v,size)) for label,s in panels],cols,size,title,foot).save(target)
    (HERE/'images').mkdir(exist_ok=True)
    old72=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/continuation_check.json');oldp=old72['paths'][-1];last=raw['paths'][-1]
    save_sheet([('ARM072 뒤늦게 죔쇠 진입,실패',scene(['arm059_straight_jaw_0','thumb_ip_barrel_adjuster_0'],{'arm059_straight_jaw_0':oldp['samples'][-1]['transform']},bright=['arm059_straight_jaw_0'])),
        ('새 원래7 벤치'+(' 완료' if len(raw['final_kit_bench_present_names'])==7 else ' 실제마지막'),scene(raw['final_kit_bench_present_names'],bright=raw['kit7_names']))],2,1100,
        '포착된 조절부와 죔쇠를 먼저 조립','기존실패 삭제0 /원래부품·설치형상 변경0 /수동지지·조임힘 미검증','arm073_kit_before_after.png')
    save_sheet([('기존25와 미리 둔 양너트',scene(raw['initial_carrier_bench_present_names'])),('새 실제마지막 carrier',scene(raw['final_carrier_bench_present_names'],bright=raw['kit7_names']))],2,1100,
        '첫 조절부 묶음의 실제 벤치 복구','앞78+양너트14=같은92승계·새0 /두번째 조절부 최종겹침은 기존보류 /세계특수48 미복구','arm073_carrier_before_after.png')
    p=next((p for p in raw['paths'] if p['stage']=='carrier_bench'),last)
    save_sheet([(label,scene(p['present_names']+p['moving_names'],{k:s['transform'] for k in p['moving_names']},bright=p['moving_names'])) for label,s in [('전체 장애물 밖 시작',p['samples'][0]),('실제 마지막'+(' /통과' if p['completed'] else ' /중단'),p['samples'][-1])]],2,1050,
        '원래 묶음 전체의 진입 경로','독립벤치→입구 등록/운반은 별도조건 /실제이동묶음 전체와 해당장애물 검사 /연속경로·공구 미검증','arm073_whole_kit_entry.png')
    if queries:
        q=queries[0];pair=scene([q['lead'],q['part']],{q['lead']:q['lead_transform']},bright=[q['lead']]);red=scene([],extras=commonmeta)
        vp=r.view_fit([pair],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);vr=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);target=HERE/'images/arm073_new_common.png';assert not target.exists()
        r.sheet([('새로 막힌 두 부품',render(pair,vp,1100)),('같은 공통재료,별도 확대',render(red,vr,1100))],2,1100,'첫 새막힘의 실제 공통','공통 '+format(q['volume_mm3'],'.10g')+' mm3 /증거는 설치부품 아님').save(target)
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/survey.json',ROOT/'cad/special_thumb_module_service_v1/reviews/continuation_check.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries),new_hardware_geometry=0,old_native_path_groups_repeated=0,old_common_Boolean_repeated=0,axis_reads_repeated_for_render=0,
        output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('arm073_*.png')},opaque=True,all_selected_geometry_inside_frame=True,view_omissions_not_test_exclusions=True,unique_image_names_no_overwrite=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('rendered',len(list((HERE/'images').glob('arm073_*.png'))),'newcommon',len(queries),flush=True)


if __name__=='__main__':main()

"""Opaque saved special48 service scenes and one actual first-block common."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm072_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm072_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();raw=n.p.read(HERE/'reviews/check.json');assert raw['completed']
    source=dict(raw['source_sha256']);meta,matrix=old.metadata();meta={z['name']:z for z in meta}
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        return n.g.original.transform(n.c.load([z],source)[0]['shape'],matrix(name))
    queries=[];common_scene=None;commonmeta=[];key=raw.get('first_blocked_sample')
    if key and raw['native'][key]['overlaps']:
        p=next(p for p in raw['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in p['samples'] if s['id']==key);q=max(raw['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3']);M=np.array(s['transform'])
        sh=n.g.original.transform(native(q['lead']),M).intersect(native(q['part']),tol=1e-6);assert sh.isValid() and sh.Solids()
        name='arm072_failed_service_common_0';queries=[dict(name=name,id=key,lead=q['lead'],part=q['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=q['material_intersection_mm3'])]
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
    def sheet(panels,cols,size,title,foot,file,direction=[.65,-.5,.6]):
        v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],np.array(direction),1.)
        r.sheet([(label,render(s,v,size)) for label,s in panels],cols,size,title,foot).save(HERE/'images'/file)
    (HERE/'images').mkdir(exist_ok=True)
    special=raw['original_special_packet']['names'];guide='arm059_integrated_electrical_R5_base'
    sheet([('원래48부품,배치참고',scene(special,bright=[guide])),('전선뚜껑을 뺀 보기',scene([k for k in special if k!='thumb_ip_actuator_wire_guide_lid'],bright=[guide]))],2,1100,
        '특수 엄지 모듈의 원래48부품,설치형상 변경0','body44+랙체결4 /오른쪽은 홈을 보여주는 표시 생략 /전체48 조립완료 그림 아님','special48_reference.png')
    first=raw['paths'][0];last=raw['paths'][-1]
    sheet([('첫 벤치 시작',scene(first['present_names'])),('실제 마지막 검사'+(' /통과' if last['completed'] else ' /중단'),scene(last['present_names']+last['moving_names'],{k:last['samples'][-1]['transform'] for k in last['moving_names']},bright=last['moving_names']))],2,1100,
        '형상 그대로,실제 벤치 조립 전후','수동지지·출력12 참고조립 조건 /뒤8·670행 유지 /모든 원래부품 복구는 별도','service_before_after.png')
    chosen=next((p for p in raw['paths'] if not p['completed']),last)
    sheet([(label,scene(chosen['present_names']+chosen['moving_names'],{k:s['transform'] for k in chosen['moving_names']},bright=chosen['moving_names'])) for label,s in [('전체 장애물 밖 시작',chosen['samples'][0]),('실제 마지막',chosen['samples'][-1])]],2,1050,
        '마지막으로 검사한 전체 진입 경로','시작부터 전체 이동묶음 검사 /표시 부품만 보기 /native에는 해당시점 전체장애물 포함','entry_path.png')
    if queries:
        q=queries[0];pair=scene([q['lead'],q['part']],{q['lead']:q['lead_transform']},bright=[q['lead']]);red=scene([],extras=commonmeta)
        vp=r.view_fit([pair],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);vr=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.)
        r.sheet([('막힌 실제 두 부품',render(pair,vp,1100)),('같은 공통재료,별도 확대',render(red,vr,1100))],2,1100,'첫 막힘의 실제 공통재료','공통 '+format(q['volume_mm3'],'.10g')+' mm3 /공통증거는 설치부품 아님 /모서리절삭0').save(HERE/'images/actual_common.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries),new_hardware_geometry=0,
        old_native_path_groups_repeated=0,old_common_Boolean_repeated=0,axis_reads_repeated_for_render=0,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},opaque=True,all_selected_geometry_inside_frame=True,view_omissions_not_test_exclusions=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('rendered',len(list((HERE/'images').glob('*.png'))),'newcommon',len(queries),flush=True)


if __name__=='__main__':main()

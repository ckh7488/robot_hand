"""Opaque actual CAD poses;two temporary bolt references and one new failed common."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm067_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm067_render_old',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py')
r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['bench_completed']
    rows,membership,source=m.context();lookup={z['name']:z for z in rows};meta,matrix=old.metadata();meta={z['name']:z for z in meta}
    first=next(p for p in raw['paths'] if p['id']=='front_tilt_0');poses=[];contracts=[]
    for i,index in enumerate([6,26]):
        s=first['samples'][index];name=first['moving_names'][0];M=np.array(s['transform']);sh=n.g.original.transform(lookup[name]['shape'],M)
        poses.append(dict(name='arm067_temporary_front_bolt_pose_'+str(i),shape=sh,body='forearm',color=[.7,.7,.73],role='temporary executed assembly pose,not new installed fastener'))
        contracts.append(dict(name=poses[-1]['name'],source_name=name,sample_id=s['id'],transform=M))
    pose_scene=n.a.a.prior.export(poses,HERE/'generated/pose_references')
    key=raw['first_blocked_sample'];path=next(p for p in raw['paths'] if any(s['id']==key for s in p['samples']));sample=next(s for s in path['samples'] if s['id']==key)
    records=sorted(raw['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3'],reverse=True)[:2]
    commons=[];queries=[]
    for i,q in enumerate(records):
        M=np.array(sample['transform']);sh=n.g.original.transform(lookup[q['lead']]['shape'],M).intersect(lookup[q['part']]['shape'],tol=1e-6)
        assert sh.isValid() and sh.Solids();name='arm067_packet_entry_common_'+str(i)
        commons.append(dict(name=name,shape=sh,body='forearm',color=[.9,.1,.08],role='failed module fastener entry common,not installed'))
        queries.append(dict(name=name,lead=q['lead'],part=q['part'],lead_transform=M,id=key,volume_mm3=sh.Volume(),solids=len(sh.Solids())))
    common_scene=n.a.a.prior.export(commons,HERE/'generated/actual_common') if commons else None
    def scene(names,poses=None,extra=None,bright=None):
        parts=[];poses=poses or {};bright=set(bright or [])
        for name in names:
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@matrix(name),
                color=[.95,.55,.1] if name in bright else z.get('color',[.3,.5,.75])))
        for z in extra or []:
            path=ROOT/z['mesh'];source[z['mesh']]=r.sha(path)
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.eye(4),color=z.get('color',[.9,.1,.08])))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    images=HERE/'images';images.mkdir(exist_ok=True)
    bolt=first['moving_names'][0];base=raw['initial_bench_present_names']
    oldM=m.translate([0,0,-40.]);panels=[]
    states=[('ARM066 수직 진입 Z-40,실패기록',oldM),('기울여 진입 45도/끝 Z-26',np.array(first['samples'][6]['transform'])),
        ('세우며 진입 15도/끝 Z-21',np.array(first['samples'][18]['transform'])),('축맞춤 후 착좌,명령변경0',np.eye(4))]
    # Camera is deliberately fitted to the real bolt poses for a labeled local
    # view. The full uncut root mesh is still rendered;no CAD cutting occurs.
    ss=[scene(base+[bolt],{bolt:M},bright=[bolt]) for _,M in states]
    fits=[scene([bolt],{bolt:M}) for _,M in states];view=r.view_fit(fits,[dict(angles_deg=[0.]*20)],np.array([.5,.3,-1.]),1.)
    r.sheet([(label,render(s,view,850)) for (label,_),s in zip(states,ss)],2,850,
        '실제 나사 진입 자세 전후,원래 부품 형상 유지','국부 확대,지지부 일부 화면밖 /기울임은 조립 자세 /실제 공구·연속조립·토크 승인 아님').save(images/'oriented_entry.png')
    ss=[scene(raw['initial_bench_present_names']),scene(raw['bench_final_present_names'])]
    view=r.view_fit(ss,[dict(angles_deg=[0.]*20)],np.array([.55,-.7,.65]),1.)
    r.sheet([(label,render(s,view,950)) for label,s in zip(['이전 조건부10행,나사12개 대기','새 조건부22행,체결 진입 표본완료'],ss)],2,950,
        '실제 지지부 선조립 진행,설치 형상 변경0','앞나사60/뒤나사28/출력나사18 새표본 /기존 경로 반복0 /실제 조임력·전체공장 승인 아님').save(images/'bench_before_after.png')
    packet=raw['packets'][0];body_path=next(p for p in raw['paths'] if p['id']=='packet_'+packet['module']);local=[m.ROOT_NAME,*[k for k in raw['bench_names'] if k!='Forearm_box_beam'],'Forearm_box_beam']
    s0=body_path['samples'][0];ss=[scene(local+packet['body_names'],{k:s0['transform'] for k in packet['body_names']}),scene(local+packet['body_names']+['index_mcp_actuator_rack_nut_1','index_mcp_actuator_rack_nut_2'])]
    view=r.view_fit(ss,[dict(angles_deg=[0.]*20)],np.array([.55,-.7,.65]),1.)
    r.sheet([(label,render(s,view,950)) for label,s in zip(['첫packet 몸체,방사+160/+Y8 시작','몸체20/너트2 원래위치,나사2 대기'],ss)],2,950,
        '첫 모터 묶음의 실제 복구 경로,나머지 몸체는 보기에서 생략','검사는 가정478행 전체를 장애물로 사용 /첫 묶음 고정나사 진입 실패 /전체16복구 성공 아님').save(images/'packet_restoration.png')
    moving=path['moving_names'];s=scene([*packet['body_names'],*packet['mounting_names']],{k:sample['transform'] for k in moving},bright=moving)
    focused=scene([queries[0]['lead'],queries[0]['part']],{queries[0]['lead']:sample['transform']},bright=moving)
    view=r.view_fit([focused],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,.7]),1.)
    left=render(s,view,1000)
    cm=n.p.read(ROOT/common_scene)['parts'];red=scene([],extra=cm);rv=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,.7]),1.)
    r.sheet([('고정나사 +Y20,자기 base_nuts에 걸림',left),('실제 공통 재료,별도 확대',render(red,rv,1000))],2,1000,
        '첫packet 고정나사 진입의 실제 실패','서로다른 부품/선언 진입 자세의 기록 /작은 재료 절삭·검사제외0 /다른 순서 불가능 증명 아님').save(images/'packet_fastener_block.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,pose_scene=pose_scene,pose_contracts=contracts,common_scene=common_scene,common_queries=queries,
        new_common_queries=len(queries),native_path_groups_repeated=0,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        opaque=True,intentional_local_camera_crop=['oriented_entry.png','packet_fastener_block.png'],hardware_geometry_changed=False,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

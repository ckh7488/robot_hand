"""Render saved CAD after border-guard failure;no repeated Booleans or exports."""
import importlib.util,shutil,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm067_saved_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm067_saved_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py')
r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence_final.json';assert not out.exists() and not (HERE/'reviews/evidence.json').exists()
    raw=n.p.read(HERE/'reviews/check.json');source=dict(raw['source_sha256']);meta,matrix=old.metadata();meta={z['name']:z for z in meta}
    history=HERE/'history';history.mkdir(exist_ok=True);shutil.copyfile(HERE/'render_evidence.py',history/'render_evidence_initial.py.txt')
    pose_scene=(HERE/'generated/pose_references/scene.json').relative_to(ROOT).as_posix()
    common_scene=(HERE/'generated/actual_common/scene.json').relative_to(ROOT).as_posix()
    pm=n.p.read(ROOT/pose_scene)['parts'];cm=n.p.read(ROOT/common_scene)['parts'];assert len(pm)==2 and len(cm)==1
    saved=n.c.load(cm,source)[0]
    first=next(p for p in raw['paths'] if p['id']=='front_tilt_0');bolt=first['moving_names'][0]
    contracts=[dict(name=z['name'],source_name=bolt,sample_id=first['samples'][i]['id'],transform=first['samples'][i]['transform']) for z,i in zip(pm,[6,26])]
    key=raw['first_blocked_sample'];path=next(p for p in raw['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in path['samples'] if s['id']==key)
    q=raw['native'][key]['overlaps'][0]
    queries=[dict(name=cm[0]['name'],id=key,lead=q['lead'],part=q['part'],lead_transform=s['transform'],
        volume_mm3=q['material_intersection_mm3'],volume_basis='Saved check native record,not recovered unsaved first-render scalar.',
        saved_STEP_volume_new_property_read_mm3=saved['shape'].Volume(),solids=len(saved['shape'].Solids()))]
    attempt=dict(completed=False,code= 'cad/oriented_fastener_entry_v1/render_evidence.py',failure='render_one border guard on first local view with full root;no first image saved',
        native_path_groups_repeated=0,common_Boolean_queries_executed=1,common_raw_numerics_saved=False,pose_STEP_exports=2,common_STEP_exports=1,
        raw_check_reference_volume_preserved=q['material_intersection_mm3'],saved_3_exports_reused_without_reexport=True,
        planned_last_scene_would_have_included_uninstalled_screw2_but_not_executed=True,
        unused_recover_render_helper_not_executed=True)
    n.a.save(HERE/'reviews/render_attempt1.json',attempt)
    old66=n.p.read(ROOT/'cad/ordered_roll_root_assembly_v1/reviews/evidence.json')
    previous=[z for z in n.p.read(ROOT/old66['common_scene'])['parts'] if z['name']=='arm066_failed_entry_common_1']
    def scene(names,poses=None,extra=None,bright=None):
        parts=[];poses=poses or {};bright=set(bright or [])
        for name in names:
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@matrix(name),
                color=[.95,.55,.1] if name in bright else z.get('color',[.3,.5,.75])))
        for z in extra or []:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.eye(4),color=[.9,.1,.08]))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def sheet(panels,cols,size,title,foot,name,vec=[.55,-.7,.65]):
        view=r.view_fit([z[1] for z in panels],[dict(angles_deg=[0.]*20)],np.array(vec),1.)
        r.sheet([(label,render(s,view,size)) for label,s in panels],cols,size,title,foot).save(HERE/'images'/name)
    (HERE/'images').mkdir(exist_ok=True)
    states=[('이전 수직 Z-40,실패',m.translate([0,0,-40.])),('기울여 진입 45도/끝 Z-26',np.array(first['samples'][6]['transform'])),
        ('세우며 진입 15도/끝 Z-21',np.array(first['samples'][18]['transform'])),('최종 착좌,원래 위치',np.eye(4))]
    sheet([(label,scene([bolt],{bolt:M},extra=previous,bright=[bolt])) for label,M in states],2,850,
        '실제 나사 자세 전후와 이전 실패 공통','빨강=ARM066 수직실패 공통 /지지부 전체는 다음 그림 /통과검사는 실제 root전체 사용','oriented_entry.png',[.5,.3,-1.])
    sheet([('이전 조건부10행,나사12개 대기',scene(raw['initial_bench_present_names'])),('새 조건부22행,진입 표본완료',scene(raw['bench_final_present_names']))],2,950,
        '실제 지지부 선조립 진행,설치 형상 변경0','앞나사60/뒤나사28/출력나사18 새표본 /기존 경로 반복0 /실제 조임력·전체공장 승인 아님','bench_before_after.png')
    packet=raw['packets'][0];body_path=next(p for p in raw['paths'] if p['id']=='packet_'+packet['module']);s0=body_path['samples'][0]
    local=raw['bench_names'];nuts=[k for k in packet['mounting_names'] if '_nut_' in k]
    sheet([('첫 몸체 방사+160/+Y8 시작',scene(local+packet['body_names'],{k:s0['transform'] for k in packet['body_names']})),
        ('몸체20/너트2 원래위치,나사2 대기',scene(local+packet['body_names']+nuts))],2,950,
        '첫 모터 묶음의 실제 복구 경로,나머지 몸체는 보기 생략','검사는 가정478행 전체가 장애물 /첫 묶음 고정나사 진입 실패 /전체16복구 성공 아님','packet_restoration.png')
    lead=scene([q['lead'],q['part']],{q['lead']:s['transform']},bright=[q['lead']]);red=scene([],extra=cm)
    v=r.view_fit([lead],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,.7]),1.);vr=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,.7]),1.)
    r.sheet([('실제 나사/자기 base_nuts 두행만',render(lead,v,1000)),('저장 실제 공통재료,별도 확대',render(red,vr,1000))],2,1000,
        '첫packet 고정나사+Y20 진입 실패','다른 packet부품·미설치 screw2는 보기 생략 /검사에는 실제500행 사용 /새 공통 반복0').save(HERE/'images/packet_fastener_block.png')
    for f in [Path(__file__),HERE/'render_evidence.py',HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/render_attempt1.json',ROOT/pose_scene,ROOT/common_scene]:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,pose_scene=pose_scene,pose_contracts=contracts,common_scene=common_scene,common_queries=queries,
        new_common_queries=1,first_render_common_scalar_saved=False,saved_common_STEP_reused=True,recovery_common_queries=0,recovery_native_path_queries=0,
        recovery_new_geometry=0,first_render_attempt_preserved=True,source_STEP_exports_repeated=0,
        output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},opaque=True,all_rendered_geometry_inside_frame=True,
        old_failure_common_is_evidence_only=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('saved CAD rendered,4images,sourceexports0/native0/common0',flush=True)


if __name__=='__main__':main()

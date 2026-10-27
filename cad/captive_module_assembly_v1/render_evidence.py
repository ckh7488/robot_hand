"""Render executed assembly states using unchanged saved meshes only."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm068_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm068_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py')
r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();raw=n.p.read(HERE/'reviews/selected_check.json');first=n.p.read(HERE/'reviews/check.json');assert raw['completed'] and first['completed']
    source=dict(raw['source_sha256']);meta,matrix=old.metadata();meta={z['name']:z for z in meta};contracts=[]
    def scene(names,poses=None,extras=None,bright=None):
        parts=[];poses=poses or {};bright=set(bright or [])
        for name in sorted(set(names)):
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@matrix(name),
                color=[.95,.55,.1] if name in bright else z.get('color',[.3,.5,.75])))
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
    def sheet(panels,cols,size,title,footer,name,vec=[.65,-.7,.6]):
        v=r.view_fit([z[1] for z in panels],[dict(angles_deg=[0.]*20)],np.array(vec),1.)
        r.sheet([(label,render(s,v,size)) for label,s in panels],cols,size,title,footer).save(HERE/'images'/name)
    (HERE/'images').mkdir(exist_ok=True)
    panels=[]
    for path in first['paths'][:2]+[p for p in raw['paths'] if p['stage']!='world']:
        chosen=sorted({max(0,len(path['samples'])-2),len(path['samples'])-1})
        for i in chosen:
            s=path['samples'][i];names=path['present_names']+path['moving_names'];poses={k:s['transform'] for k in path['moving_names']}
            panels.append((path['id']+' / '+str(s['translation_mm'][0])+'mm',scene(names,poses,bright=path['moving_names'])))
            contracts.append(dict(path=path['id'],sample_id=s['id'],shown_names=sorted(names),moving_names=path['moving_names'],transform=s['transform']))
    sheet(panels,2,850,'원래 형상으로 M2 나사 선조립,실행한 상태만 표시','주황=이동 묶음 /나머지886은 분리벤치 조건 /전기·공급품 내부 조립은 참고묶음','bench_order.png')
    worlds=[p for p in raw['paths'] if p['stage']=='world']
    if worlds:
        panels=[]
        for path in worlds:
            s=path['samples'][min(2,len(path['samples'])-1)]
            local=[k for k in path['present_names'] if k in raw['packets'][0]['names']]+path['moving_names']
            panels.append((path['id'].replace('rack_M3_before_base_nuts_','')+' / '+str(s['translation_mm']),scene(local,{k:s['transform'] for k in path['moving_names']},bright=path['moving_names'])))
        panels.append(('실제 마지막 상태,완료 묶음 '+str(len(raw['restored_modules']))+'/16',scene([k for k in raw['packet_final_present_names'] if k in raw['packets'][0]['names']])))
        sheet(panels,2,900,'랙 M3 먼저,받침 M2 너트는 마지막','다른 부품은 보기 생략 /검사는 해당 시점 전체 장애물 /원래 설치위치 복구 여부는 check기록','module_order.png')
    p0=raw['packets'][0];local=raw['bench_names']+[k for k in raw['packet_final_present_names'] if k in p0['names']]
    sheet([('이번 시작,지지부22행만 표시',scene(raw['bench_names'])),('실제 마지막 조건부 상태',scene(local))],2,1050,
        '지지부와 첫 모터 묶음의 실제 CAD 전후','비packet478 조립가정 /다른15묶음 아직 없음 /현재862 교체·외형 채택 아님','restoration_before_after.png')
    ev=n.p.read(ROOT/'cad/oriented_fastener_entry_v1/reviews/evidence_final.json');cm=n.p.read(ROOT/ev['common_scene'])['parts'];q=ev['common_queries'][0]
    s1=scene([q['lead'],q['part']],{q['lead']:q['lead_transform']},bright=[q['lead']]);s2=scene([],extras=cm)
    v1=r.view_fit([s1],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,.7]),1.);v2=r.view_fit([s2],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,.7]),1.)
    r.sheet([('ARM067 실패,같은 나사와 원래 너트',render(s1,v1,900)),('ARM067 저장 공통,별도 확대',render(s2,v2,900))],2,900,
        '이전 실패는 그대로 보존,새 공통 질의0','기존 0.36207124135856594 mm3 /이번에는 받침 너트를 뒤에 조립 /작은 모서리 절삭0').save(HERE/'images/previous_block_preserved.png')
    failed=first['paths'][-1];fs=failed['samples'][-1]
    fixed_leads=[k for k in failed['present_names'] if '_lead' in k]
    moving=first['module']+'_cradle';selected=raw['paths'][0]
    ss=next((s for s in selected['samples'] if s['translation_mm']==fs['translation_mm']),selected['samples'][-1])
    sheet([('처음:전선 고정,받침만 이동하여 실패',scene([moving]+fixed_leads,{moving:fs['transform']},bright=[moving])),
        ('새 순서:모터와 자기 전선을 함께 이동',scene([moving]+fixed_leads,{k:ss['transform'] for k in [moving]+fixed_leads},bright=[moving]+fixed_leads))],2,950,
        '분리벤치에서 전선을 함께 옮기는 순서로 교정','처음 X-5에서 전선3가닥 각1.5639339002 mm3 /처음20검사 보존 /새 검사는 전체9행 상대','electrical_order_correction.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/selected_check.json',ROOT/ev['common_scene']]:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,shown_pose_contracts=contracts,new_native_queries=0,new_common_Boolean_queries=0,new_STEP_STL_exports=0,
        reused_old_failure_common_scene=ev['common_scene'],output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},
        opaque=True,all_selected_geometry_inside_frame=True,view_omissions_are_not_test_exclusions=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('rendered',len(list((HERE/'images').glob('*.png'))),'native0 common0 export0',flush=True)


if __name__=='__main__':main()

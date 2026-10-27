"""Actual before/after mounting poses and first new failure if present."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm071_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm071_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();raw=n.p.read(HERE/'reviews/check.json');assert raw['completed']
    source=dict(raw['source_sha256']);meta,matrix=old.metadata();meta={z['name']:z for z in meta}
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        return n.g.original.transform(n.c.load([z],source)[0]['shape'],matrix(name))
    queries=[];common_scene=None;commonmeta=[]
    key=raw.get('first_blocked_sample')
    if key and raw['native'][key]['overlaps']:
        p=next(p for p in raw['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in p['samples'] if s['id']==key);q=max(raw['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3']);M=np.array(s['transform'])
        sh=n.g.original.transform(native(q['lead']),M).intersect(native(q['part']),tol=1e-6);assert sh.isValid() and sh.Solids()
        name='arm071_failed_entry_common_0';queries=[dict(name=name,id=key,lead=q['lead'],part=q['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=q['material_intersection_mm3'])]
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
    def sheet(panels,cols,size,title,foot,file,direction=[.55,-.7,.65]):
        v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],np.array(direction),1.)
        r.sheet([(label,render(s,v,size)) for label,s in panels],cols,size,title,foot).save(HERE/'images'/file)
    (HERE/'images').mkdir(exist_ok=True)
    c=raw['module_contracts'][0];body=c['body19_names'];root='arm065_shared_output_beam_rack_root';p=next(p for p in raw['paths'] if p['id'].endswith('__body19_final_datum_radial'));s=p['samples'][-1]
    old_pose=m.translate(8.*np.array(c['normal']))
    sheet([('ARM070 막힌 진입,+Y8',scene(body+[root],{k:old_pose for k in body},bright=body)),('새 경로 실제 마지막'+(' /착좌완료' if p['completed'] else ' /중단'),scene(body+[root],{k:s['transform'] for k in body},bright=body))],2,1100,
        '설치위치 그대로,최종 장착면에 맞춘 진입','형상·설치축·명령 변경0 /옛+8 경로 재검사0 /다른부품 보기 생략,native는 당시 전체 장애물','datum_before_after.png')
    bench=n.p.read(ROOT/'cad/captive_module_assembly_v1/reviews/selected_check.json')['bench_names'];packet_names={k for p in raw['original16packets'] for k in p['names']}
    before=bench+sorted(set(raw['initial_world_present_names'])&packet_names);after=bench+sorted(set(raw['packet_final_present_names'])&packet_names)
    sheet([('이번 시작,완료5/16',scene(before)),('실제 마지막,완료'+str(len(raw['restored_modules']))+'/16',scene(after))],2,1100,'원래 뒤쪽 묶음의 실제 복구 상태','비packet478 조립가정 /부분복구와완전24 구별 /현재862 교체·방사랙 외형채택 아님','restoration_before_after.png')
    names=bench+sorted(set(p['present_names'])&packet_names)+body
    sheet([(label,scene(names,{k:z['transform'] for k in body},bright=body)) for label,z in [('전체 장애물 밖 시작',p['samples'][0]),('실제 마지막',s)]],2,1050,
        '약지PIP 완성 body19의 전체 바깥 진입','원래body19 벤치92범위 승계·새0 /방사 마지막20구간5mm표본 /공구·연속이동 미검증','whole_body_entry.png')
    wp=[p for p in raw['paths'] if p['stage']=='world' and '__rack_M3_' in p['id']]
    if wp:
        chosen=wp[-2:];panels=[]
        for p in chosen:
            for label,z in [('바깥 시작',p['samples'][0]),('실제 마지막',p['samples'][-1])]:
                names=bench+sorted(set(p['present_names'])&packet_names)+p['moving_names'];panels.append((p['module'].replace('_actuator','')+' M3 '+label,scene(names,{k:z['transform'] for k in p['moving_names']},bright=p['moving_names'])))
        sheet(panels,2,950,'원래 랙 M3 고정나사의 실제 진입','원래4M2받침너트는 M3뒤 복구 /모든 실제 설치·누락목록 저장 /나사산토크 미검증','rack_M3_entry.png')
    if queries:
        q=queries[0];pair=scene([q['lead'],q['part']],{q['lead']:q['lead_transform']},bright=[q['lead']]);red=scene([],extras=commonmeta)
        vp=r.view_fit([pair],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);vr=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.)
        r.sheet([('막힌 실제 두 부품',render(pair,vp,1100)),('같은 공통재료,별도 확대',render(red,vr,1100))],2,1100,'새 전체진입에서 처음 막힌 위치','공통 '+format(q['volume_mm3'],'.10g')+' mm3 /작은 모서리 절삭0 /원래 실패 보존').save(HERE/'images/first_block.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries),new_hardware_geometry=0,
        old_native_path_groups_repeated=0,old_common_Boolean_repeated=0,actual_new_axis_reads=raw['actual_new_axis_reads'],axis_reads_repeated_for_render=0,
        output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},opaque=True,all_selected_geometry_inside_frame=True,view_omissions_not_test_exclusions=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('rendered',len(list((HERE/'images').glob('*.png'))),'newcommon',len(queries),flush=True)


if __name__=='__main__':main()

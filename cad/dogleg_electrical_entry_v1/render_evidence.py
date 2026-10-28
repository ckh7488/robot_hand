"""Save new common scalars before exports and render opaque actual assembly states."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm080_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm080_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();q=n.p.read(HERE/'reviews/final_check.json');assert q['completed'];source=dict(q['source_sha256']);metas,matrix=old.metadata();meta={z['name']:z for z in metas};meta.update({z['name']:z for z in n.p.read(ROOT/n.p.read(ROOT/'cad/branched_terminal_support_v1/reviews/check.json')['scene'])['parts']})
    def pose(name):return np.eye(4) if name==m.NAME else matrix(name)
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        return n.g.original.transform(n.c.load([z],source)[0]['shape'],pose(name))
    queries=[];common_scene=None;extras=[];key=q.get('first_blocked_sample');path=q['paths'][-1];last=path['samples'][-1]
    obstacle_properties={}
    for part in sorted({z['part'] for z in q['native'][last['id']]['overlaps']}):
        sh=native(part);obstacle_properties[part]=dict(bbox=n.c.m.pack._bbox(sh),volume_mm3=sh.Volume(),center_mm=sh.Center().toTuple())
    if key:
        for i,v in enumerate(sorted(q['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3'],reverse=True)[:2]):
            M=np.array(last['transform']);sh=n.g.original.transform(native(v['lead']),M).intersect(native(v['part']),tol=1e-6);assert sh.isValid() and sh.Solids();queries.append(dict(name='arm080_actual_common_'+str(i),id=key,lead=v['lead'],part=v['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=v['material_intersection_mm3']));extras.append(dict(name=queries[-1]['name'],shape=sh,body='forearm',color=[.92,.08,.07],evidence_only=True))
        n.a.save(HERE/'reviews/common_prepared.json',dict(completed=False,common_queries=queries,common_scene=None))
        if extras:common_scene=n.a.a.prior.export(extras,HERE/'generated/actual_common');meta.update({z['name']:z for z in n.p.read(ROOT/common_scene)['parts']})
    n.a.save(HERE/'reviews/common_prepared.json',dict(completed=True,common_queries=queries,common_scene=common_scene,new_common_queries=len(queries)))
    neutral={z['name'] for z in queries};red=set(neutral)
    def scene(names,poses=None,highlight=None):
        result=[];poses=poses or {};highlight=set(highlight or [])
        for name in sorted(set(names)):
            z=meta[name];p=ROOT/z['mesh'];assert r.sha(p)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];M=np.eye(4) if name in neutral else pose(name)
            result.append(dict(name=name,body='palm',mesh_path=p,points=r.read_stl_points(p),neutral_inverse=np.array(poses.get(name,np.eye(4)))@M,color=[.92,.08,.07] if name in red else [.94,.61,.12] if name in highlight else z.get('color',[.3,.5,.75])))
        return dict(parts=result,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.;return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def sheet(panels,title,footer,name,separate=False):
        target=HERE/'images'/('arm080_'+name+'.png');assert not target.exists();size=1100;direction=np.array([.65,-.5,.6]);v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],direction,1.);images=[]
        for label,s in panels:
            vv=r.view_fit([s],[dict(angles_deg=[0.]*20)],direction,1.) if separate else v;images.append((label,render(s,vv,size)))
        r.sheet(images,2,size,title,footer).save(target)
    (HERE/'images').mkdir(exist_ok=True);name=path['module'];packet=next(z for z in q['original16packets'] if z['module']==name);moving=path['moving_names'];present=path['present_names'];local=set(present)&set(packet['names']);blockparts={z['part'] for z in q['native'][last['id']]['overlaps']};local|=blockparts
    sheet([('직전 고정 부품,이동 묶음은 아직 없음',scene(local)),('실제 마지막 도달 자세,주황색 이동 묶음',scene(local|set(moving),{k:last['transform'] for k in moving},moving))],'원래 전기 부품의 실제 조립 전후','형상·설치축 유지 /국부 그림의 표시 생략과 native 장애물 목록은 별개 /첫 실패 뒤 미실행','assembly_before_after')
    prior=n.p.read(ROOT/'cad/open_electrical_world_assembly_v1/reviews/check.json');oldpath=prior['paths'][-1];newpath=q['paths'][0];fixed={m.ROOT_PART,'Elbow_fixed_cradle'};M=newpath['samples'][-1]['transform'];oldM=oldpath['samples'][-1]['transform']
    sheet([('이전,직선 뒤진입의 실제 막힘',scene(fixed|set(oldpath['moving_names']),{k:oldM for k in oldpath['moving_names']},oldpath['moving_names'])),('새,옆진입 경로의 실제 마지막',scene(fixed|set(newpath['moving_names']),{k:M for k in newpath['moving_names']},newpath['moving_names']))],'세 전선의 실제 진입 경로 전후','시험대 받침·공동 root 유지 /이전88 native 반복0 /새 실제 도달만 표시 /전체 경로 성공 여부는 원시 결과 참조','route_before_after',True)
    world=set(q['packet_final_present_names']);initial=set(q['initial_world731_names']);show=set(q['mapped_special_packet']['names'])|set(packet['names']);show|={k for k in world if k.startswith('arm065_') or k.startswith('arm064_')}
    sheet([('시작,실제 세계731의 국부 구성',scene(show&initial)),('끝,실제 세계'+str(len(world))+'의 국부 구성',scene(show&world,highlight=world-initial))],'특수 엄지48을 유지한 앞쪽 모듈 복구','모든731은 세계진입의 실제 장애물 /그림은 엄지·대상·지지부만 /전체공장·16모터설치·구동·외형 미승인','world_before_after')
    if queries:
        for i,z in enumerate(queries):sheet([('실제 막힘 자세,두 부품',scene([z['lead'],z['part']],{z['lead']:z['lead_transform']},[z['lead']])),('같은 두 부품의 실제 공통재료',scene([z['name']]))],'첫 경로의 실제 공통재료 '+str(i+1),'공통 '+format(z['volume_mm3'],'.11g')+' mm3 /대표 증거이며 새 설치물 아님 /한 경로 실패를 구조적 불가능으로 확대하지 않음','common_'+str(i),True)
    else:
        sheet([('첫 경로의 전체장애물 밖 시작',scene(set(q['paths'][0]['present_names'])|set(q['paths'][0]['moving_names']),{k:q['paths'][0]['samples'][0]['transform'] for k in q['paths'][0]['moving_names']})),('마지막 경로의 실제 도달 자세',scene(local|set(moving),{k:last['transform'] for k in moving},moving))],'원래 부품의 전체 진입과 실제 마지막','유한 표본 /수동 유지·운반·공구·체결력·연속 경로 미검증','entry_result',True)
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/final_check.json',HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    outputs={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('arm080_*.png')};n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries),new_hardware_geometry=0,new_failed_obstacle_properties=obstacle_properties,output_sha256=outputs,opaque=True,unique_image_names_no_overwrite=True,view_omissions_not_native_exclusions=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('render',len(outputs),'newcommon',len(queries),flush=True)


if __name__=='__main__':main()

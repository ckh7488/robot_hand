"""Unique opaque images of the whole support change and any first new block."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm074_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm074_render_engine',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();q=n.p.read(HERE/'reviews/continuation_check.json');assert q['completed']
    source=dict(q['source_sha256']);metas,matrix=old.metadata();meta={z['name']:z for z in metas};fresh=n.p.read(ROOT/q['scene'])['parts'];meta.update({z['name']:z for z in fresh});common=[];queries=[];common_scene=None
    def pose(name):return np.eye(4) if name==m.NAME else matrix(name)
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        return n.g.original.transform(n.c.load([z],source)[0]['shape'],pose(name))
    key=q.get('first_new_blocked_group')
    if key and q['native'][key]['overlaps']:
        contract=next(z for z in q['native_contracts'] if z['id']==key);rec=max(q['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3'])
        L=np.array(contract['left_transform']);R=np.array((contract['right_frame_transforms'] or {}).get(rec['part'],np.eye(4)))
        sh=n.g.original.transform(native(rec['lead']),L).intersect(n.g.original.transform(native(rec['part']),R),tol=1e-6);assert sh.isValid() and sh.Solids()
        name='arm074_first_new_support_common';queries=[dict(name=name,id=key,lead=rec['lead'],part=rec['part'],lead_transform=L,part_transform=R,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=rec['material_intersection_mm3'])]
        n.a.save(HERE/'reviews/common_prepared.json',dict(completed=False,common_queries=queries,common_scene=None))
        common_scene=n.a.a.prior.export([dict(name=name,shape=sh,body='forearm',evidence_only=True,color=[.9,.1,.08])],HERE/'generated/actual_common');common=n.p.read(ROOT/common_scene)['parts'];meta.update({z['name']:z for z in common})
    n.a.save(HERE/'reviews/common_prepared.json',dict(completed=True,common_queries=queries,new_common_queries=len(queries),common_scene=common_scene))
    def scene(names,poses=None,highlight=None):
        parts=[];poses=poses or {};highlight=set(highlight or [])
        for name in sorted(names):
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            M=pose(name) if name not in {z['name'] for z in common} else np.eye(4)
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@M,
                color=[.9,.1,.08] if name in {z['name'] for z in common} else [.93,.48,.1] if name in highlight else z.get('color',[.3,.5,.75])))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def sheet(panels,title,footer,file,direction=[.65,-.5,.6]):
        target=HERE/'images'/file;assert not target.exists();v=r.view_fit([z for _,z in panels],[dict(angles_deg=[0.]*20)],np.array(direction),1.)
        r.sheet([(label,render(z,v,1100)) for label,z in panels],2,1100,title,footer).save(target)
    (HERE/'images').mkdir(exist_ok=True)
    hardware=set(next(p for p in q['original16packets'] if p['module']=='thumb_ip_actuator')['names'])
    barrel={k for k in hardware if k.startswith(('thumb_ip_barrel_','motor_housing_clamp_','arm059_straight_jaw_'))}
    sheet([('이전 공용 연결대',scene({m.OLD}|barrel,highlight=[m.OLD])),('새 전체 분기 받침 후보',scene({m.NAME}|barrel,highlight=[m.NAME]))],
        '두 조절부 공간을 비운 전체 지지 경로','부품·설치축 이동0 /원래 전기받침·M3·양R5 유지 /새 돌출과 강도·외형 미승인','arm074_support_before_after.png')
    sheet([('이전 특수 모듈48',scene(hardware,highlight=[m.OLD])),('별도 받침1 교체 후보48',scene((hardware-{m.OLD})|{m.NAME},highlight=[m.NAME]))],
        '모터·전기·죠를 유지한 실제 CAD 전후','현재862 교체0 /뒤8·조건부670유지 /기존49·92 조립승계0 /전체구동 미승인','arm074_module_before_after.png',[-.6,.7,.5])
    if queries:
        a=queries[0];joined=scene({a['lead'],a['part']},{a['lead']:a['lead_transform'],a['part']:a['part_transform']},[m.NAME]);red=scene([a['name']]);panels=[]
        for label,s in [('첫 새막힘의 두 부품',joined),('같은 공통재료만 확대',red)]:
            v=r.view_fit([s],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);panels.append((label,render(s,v,1100)))
        target=HERE/'images/arm074_first_new_common.png';assert not target.exists();r.sheet(panels,2,1100,'새 받침에 남은 첫 실제 막힘','공통 '+format(a['volume_mm3'],'.10g')+' mm3 /증거는 설치부품 아님').save(target)
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/continuation_check.json',HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries),new_support_geometry=1,
        output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('arm074_*.png')},opaque=True,all_selected_geometry_inside_frame=True,unique_image_names_no_overwrite=True,view_omissions_not_test_exclusions=True,old_native_groups_repeated=0,old_common_Boolean_repeated=0,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('images',len(list((HERE/'images').glob('arm074_*.png'))),'newcommon',len(queries),flush=True)


if __name__=='__main__':main()

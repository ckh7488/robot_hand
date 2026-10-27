"""Correct only the last display's absent screw;reuse saved three images and CAD."""
import importlib.util,shutil,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm067_recover',Path(__file__).with_name('render_evidence.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,r,ROOT,HERE,engine=a.n,a.np,a.r,a.ROOT,a.HERE,a.engine


def main():
    out=HERE/'reviews/evidence_final.json';assert not out.exists()
    ev=n.p.read(HERE/'reviews/evidence.json');raw=n.p.read(HERE/'reviews/check.json');source=dict(ev['source_sha256'])
    history=HERE/'history';history.mkdir(exist_ok=True)
    shutil.copyfile(HERE/'render_evidence.py',history/'render_initial.py.txt')
    image=HERE/'images/packet_fastener_block.png';shutil.copyfile(image,history/'packet_fastener_block_initial_absent_screw.png')
    original_image_hash=r.sha(image);meta,matrix=a.old.metadata();meta={z['name']:z for z in meta}
    key=raw['first_blocked_sample'];p=next(p for p in raw['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in p['samples'] if s['id']==key)
    moving=set(p['moving_names']);packet=raw['packets'][0];shown=[k for k in packet['names'] if k in set(p['present_names'])|moving]
    omitted=sorted(set(packet['names'])-set(shown));assert omitted==['index_mcp_actuator_rack_screw_2'] and len(shown)==23
    def scene(names,extra=None):
        parts=[]
        for name in names:
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl']
            M=(np.array(s['transform']) if name in moving else np.eye(4))@matrix(name)
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=M,color=[.95,.55,.1] if name in moving else z.get('color',[.3,.5,.75])))
        for z in extra or []:
            path=ROOT/z['mesh'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.eye(4),color=[.9,.1,.08]))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(scene,view):
        try:
            scene['_render_state']=r.make_renderer(scene,{'v':view},1000,engine);scene['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(scene,[0.]*20,'v',{'v':view},1000,engine)[0]
        finally:
            if '_render_state' in scene:scene['_render_state']['renderer'].close();del scene['_render_state']
    q=ev['common_queries'][0];shown_scene=scene(shown);focus=scene([q['lead'],q['part']])
    view=r.view_fit([focus],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,.7]),1.)
    red=scene([],n.p.read(ROOT/ev['common_scene'])['parts']);rv=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,.7]),1.)
    r.sheet([('실제23행,고정나사+Y20에서 걸림',render(shown_scene,view)),('저장한 실제 공통재료,별도 확대',render(red,rv))],2,1000,
        '첫packet 고정나사 진입 실패,아직 없는 나사2는 생략','마지막 그림의 부품목록만 교정 /앞3그림·저장CAD재사용 /native·공통 질의 반복0').save(image)
    for f in [Path(__file__),HERE/'reviews/evidence.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    ev.update(source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},
        first_render_display_error='Last scene included index_mcp_actuator_rack_screw_2 despite its pending status. Original source/image/evidence preserved;current picture uses actual23 packet rows only.',
        original_incorrect_image_sha256=original_image_hash,corrected_packet_shown_names=shown,corrected_packet_omitted_names=omitted,
        first3_images_reused=True,recovery_native_queries=0,recovery_common_queries=0,recovery_new_geometry=0,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items()))
    n.a.save(out,ev);print('last image corrected,actual23,omitted',omitted,'native0/common0',flush=True)


if __name__=='__main__':main()

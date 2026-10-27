"""Render stored actual common solids; no new Boolean operations."""
import importlib.util
from pathlib import Path
spec=importlib.util.spec_from_file_location('_arm059_detail_post',Path(__file__).with_name('postprocess.py'))
p=importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
n,np,r,engine,ROOT,HERE=p.n,p.np,p.r,p.engine,p.ROOT,p.HERE


def main():
    out=HERE/'reviews/mechanical_render.json'
    assert not out.exists()
    raw=n.p.read(HERE/'reviews/mechanical_followup.json')
    rows=n.p.read(ROOT/raw['scene'])['parts']
    chosen=[z for z in rows if 'outside_old' not in z['name']]
    source={}
    images=[]
    labels=['실제명령-90° spool/새dogleg 공통10.759436mm³','실제편위-20° jaw/손목guide 공통11.711018mm³','실제middle_dip너트/새carrier 공통17.954510mm³']
    for label,z in zip(labels,chosen):
        path=ROOT/z['mesh']
        assert r.sha(path)==z['sha256_stl']
        source[z['mesh']]=z['sha256_stl']
        scene=dict(parts=[dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
            neutral_inverse=np.eye(4),color=[.98,.04,.03])],design=n.g.DESIGN)
        view=r.view_fit([scene],[dict(angles_deg=[0.]*20)],np.array([.8,-.6,.5]),1.)
        try:
            scene['_render_state']=r.make_renderer(scene,{'v':view},850,engine)
            scene['_render_state']['model'].geom_rgba[:,3]=1.
            pic=r.render_one(scene,[0.]*20,'v',{'v':view},850,engine)[0]
            images.append((label,pic))
        finally:
            scene['_render_state']['renderer'].close()
    file=HERE/'images/new_conflicts.png'
    r.sheet(images,3,850,'새 후보에 남은 실제 공통재료,각기 다른 배율',
        '첫 공통은 옛carrier밖 새추가재료에전부남음 /중립회피는출력회전·손목운동승인이아님 /현재설치변경0').save(file)
    for path in [Path(__file__),HERE/'reviews/mechanical_followup.json']: source[path.relative_to(ROOT).as_posix()]=r.sha(path)
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={file.relative_to(ROOT).as_posix():r.sha(file)},
        native_queries=0,actual_common_only=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__': main()

"""Render the new far-side approach without repeating the five saved images."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm068_entry_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm068_entry_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    out=HERE/'reviews/evidence_final.json';assert not out.exists();raw=n.p.read(HERE/'reviews/selected_check.json');ext=n.p.read(HERE/'reviews/entry_extension.json');ev=n.p.read(HERE/'reviews/evidence.json')
    assert ext['completed'];source=dict(ev['source_sha256']);source.update(ext['source_sha256']);meta,matrix=old.metadata();meta={z['name']:z for z in meta}
    def scene(p,s):
        names=sorted(set(raw['bench_names'])|({k for k in p['present_names'] if k in raw['packets'][0]['names']}))
        parts=[]
        for name in names+p['moving_names']:
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            moving=name in p['moving_names'];parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=(np.array(s['transform']) if moving else np.eye(4))@matrix(name),color=[.95,.55,.1] if moving else z.get('color',[.3,.5,.75])))
        return dict(parts=parts,design=n.g.DESIGN)
    panels=[]
    for p in [ext['paths'][0],ext['paths'][-1]]:
        for label,s in [('바깥 시작',p['samples'][0]),('새 구간 마지막',p['samples'][-1]),('기존 착좌 구간 끝',p['reused_suffix_samples'][-1])]:
            panels.append((p['moving_names'][0].replace('index_mcp_actuator_','')+' '+label,scene(p,s)))
    v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],np.array([.5,-.7,.7]),1.)
    def render(s):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},850,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},850,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    file=HERE/'images/full_entry_extension.png'
    r.sheet([(label,render(s)) for label,s in panels],3,850,'전체 장애물 경계상자 바깥부터 실제 진입','M3는 X+160/Y+40에서 옆진입 /M2너트는 X+160에서 진입 /기존24구간 재검사0 /다른몸체는 보기 생략').save(file)
    for f in [Path(__file__),HERE/'reviews/entry_extension.json',HERE/'reviews/evidence.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    outputs=dict(ev['output_sha256']);outputs[file.relative_to(ROOT).as_posix()]=r.sha(file)
    n.a.save(out,dict(ev,source_sha256=source,output_sha256=outputs,new_images=1,previous5images_repeated=0,extension_scenes_use_exact_saved_transforms=True,
        extended_access_clear=ext['extended_access_clear'],input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('extension rendered,old5 repeated0/native0/common0/export0',flush=True)


if __name__=='__main__':main()

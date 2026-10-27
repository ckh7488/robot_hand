"""Two new common queries at the saved failed cap pose; no path replay."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm065_assembly_render',Path(__file__).with_name('render_evidence.py'))
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
a,n,np,d,ROOT,HERE=b.a,b.n,b.np,b.d,b.ROOT,b.HERE
r,engine,post=b.r,b.engine,b.post


def main():
    out=HERE/'reviews/evidence_complete.json';assert not out.exists()
    ev=n.p.read(HERE/'reviews/evidence.json');raw=n.p.read(HERE/'reviews/check.json');assert ev['completed'] and not raw['assembly_completed']
    pose=raw['assembly'][-1];records=raw['native'][pose['id']]['overlaps'];assert len(records)==3
    source=dict(ev['source_sha256']);meta,delta,reloc,clock,shift,moved=b.r64.prev.metadata()
    def matrix(name):
        M=np.array(delta.get(name,np.eye(4)))
        if name in reloc:M=reloc[name]@M
        if name in post.a.a.CLOCK_NAMES:M=clock@M
        if name in moved:M=shift@M
        return M
    capmeta=next(z for z in n.p.read(ROOT/raw['scene'])['parts'] if z['name']==pose['name'])
    cap=n.c.load([capmeta],source)[0];T=np.eye(4);T[:3,3]=pose['translation_mm'];cap['shape']=n.g.original.transform(cap['shape'],T)
    partmeta=[next(z for z in meta if z['name']==q['part']) for q in records]
    parts=n.c.load(partmeta,source)
    for z in parts:z['shape']=n.g.original.transform(z['shape'],matrix(z['name']))
    common=[];queries=[]
    for i,z in enumerate(parts[:2]):
        sh=cap['shape'].intersect(z['shape'],tol=1e-6);assert sh.isValid() and sh.Solids()
        queries.append(dict(lead=cap['name'],part=z['name'],lead_transform=T,volume_mm3=sh.Volume(),solids=len(sh.Solids())))
        common.append(dict(name='arm065_actual_common_'+str(i),shape=sh,body='forearm',color=[.86,.13,.12],role='saved failed assembly pose actual common,not installed'))
    common_scene=n.a.a.prior.export(common,HERE/'generated/assembly_common')
    def scene(items):
        out=[]
        for z in items:
            M=T if z['name']==cap['name'] else matrix(z['name']);path=ROOT/z['mesh']
            assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            out.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=M,color=z.get('color',[.3,.5,.75])))
        return dict(parts=out,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def fit(ss):return r.view_fit(ss,[dict(angles_deg=[0.]*20)],np.array([.65,-.75,.65]),1.)
    s=scene([capmeta]+partmeta)
    r.sheet([('첫cap 진입 Z+14mm /주변 실제모터·rotor',render(s,fit([s]),1100))],1,1100,
        '완성 위치는 비어도 첫 조립 위치는 막힘','새cap/작은MCP모터126.40064196430593·작은PIP모터104.94920010014212·rotor79.90828414561531mm3 /이후조립미실행').save(HERE/'images/assembly_block.png')
    panels=[]
    for z in n.p.read(ROOT/common_scene)['parts']:
        s=scene([z]);panels.append((z['name'],render(s,fit([s]),950)))
    r.sheet(panels,2,950,'저장된 첫 조립 실패의 실제 공통재료2,각각 확대','2새공통질의만추가 /원래조립경로재실행0 /실패뒤나사·너트미실행 /손·모터절삭·이동0').save(HERE/'images/assembly_common.png')
    for f in [Path(__file__),HERE/'reviews/evidence.json',HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(ev,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},
        common_scene=common_scene,common_queries=queries,new_common_queries=2,assembly_failure_pose=pose,
        original_first4_images_reused=True,assembly_path_groups_repeated=0,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

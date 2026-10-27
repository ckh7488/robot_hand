"""Reuse four finished images;allow disconnected common evidence without hiding it."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm063_render_saved',Path(__file__).with_name('render_evidence.py'))
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
a,n,np,d,ROOT,HERE,r,engine=t.a,t.n,t.np,t.d,t.ROOT,t.HERE,t.r,t.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    first=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json')
    before,delta,reloc,clock,shift,moved=t.metadata();source={}
    initial=n.p.read(ROOT/first['scene'])['parts'];fresh=n.p.read(ROOT/selected['scene'])['parts']
    entry=n.p.read(ROOT/first['entry_scene'])['parts'];bound=n.p.read(ROOT/first['protection_input_scene'])['parts'][:9]
    for z in before+initial+fresh+entry+bound:
        assert r.sha(ROOT/z['mesh'])==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
    def matrix(name):
        M=np.array(delta.get(name,np.eye(4)))
        if name in reloc:M=reloc[name]@M
        if name in t.post.a.a.CLOCK_NAMES:M=clock@M
        if name in moved:M=shift@M
        return M
    shapes=n.c.load(initial,source);lookup={z['name']:z for z in shapes};carrier=lookup[a.CARRIER]['shape']
    rotor_record=first['native']['new_whole_unit6__retained865_neutral']['overlaps'][0]
    meta=next(z for z in before if z['name']==rotor_record['part']);rotor=n.c.load([meta],source)[0]
    rotor['shape']=n.g.original.transform(rotor['shape'],matrix(rotor['name']))
    query_file=HERE/'reviews/common_recovery.json';assert not query_file.exists();queries=[];common=[]
    for label,right in [('first_rotor',rotor['shape']),('first_cap',lookup[a.CAP]['shape'])]:
        q=carrier.intersect(right,tol=1e-6);assert q.isValid() and len(q.Solids())>0
        queries.append(dict(id=label,volume_mm3=q.Volume(),solids=len(q.Solids())))
        n.a.save(query_file,dict(completed=False,queries=queries,original_unstored_query_count_unknown=True))
        common.append(dict(name='arm063_'+label+'_common',shape=q,body='forearm',color=[.85,.1,.1],role='preserved_initial_failure_common_not_installed'))
    common_scene=n.a.a.prior.export(common,HERE/'generated/first_common_recovered')
    panels=[]
    for z,label in zip(n.p.read(ROOT/common_scene)['parts'],['첫후보 실제rotor공통,현재살아님','첫후보 실제cap공통,현재살아님']):
        path=ROOT/z['mesh'];source[z['mesh']]=z['sha256_stl']
        s=dict(parts=[dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.eye(4),color=z['color'])],design=n.g.DESIGN)
        view=r.view_fit([s],[dict(angles_deg=[0.]*20)],np.array([.65,-.75,.7]),1.)
        try:
            s['_render_state']=r.make_renderer(s,{'v':view},950,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            im=r.render_one(s,[0.]*20,'v',{'v':view},950,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
        panels.append((label,im))
    images=HERE/'images'
    assert all((images/(x+'.png')).exists() for x in ['before_after','whole_unit_path','root_correction','reserved_spaces'])
    r.sheet(panels,2,950,'보존한 첫 후보의 실제 공통재료,각기 다른 확대',
        '분리된공통조각도그대로저장 /첫1solid가정오류보존 /설치물0 /검사제외0').save(images/'first_failure_common.png')
    for f in [Path(__file__),HERE/'render_evidence.py',HERE/'model.py',HERE/'selected_model.py',HERE/'reviews/check.json',HERE/'reviews/selected_check.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(query_file,dict(completed=True,queries=queries,scene=common_scene,original_unstored_query_count_unknown=True,
        original_saved_common_queries=0,recovery_new_queries=2,native_groups_repeated=0))
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        first_common_scene=common_scene,first_common_queries=queries,common_source_queries=2,initial_rotor_record=rotor_record,
        original_render_failed_due_to_unjustified_single_solid_common_assumption=True,first_four_saved_images_reused_without_rerender=True,
        original_saved_common_queries=0,original_unstored_query_count_unknown=True,native_groups_repeated=0,opaque=True,
        whole_hardware_display_counts=[871,871],pixels_matrices_armvisualunits_numerically_replayed=False,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

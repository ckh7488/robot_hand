"""Original electrical parts through a boundary-derived broken entry path."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm080_prior',ROOT/'cad/open_electrical_world_assembly_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
OLD,NAME,MODULE,ROOT_PART=a.OLD,a.NAME,a.MODULE,a.ROOT_PART
translate,moved,bbox,project_bounds,outside_start,inward_steps,bench_vectors,corners,enrich_vectors,replay_samples=a.translate,a.moved,a.bbox,a.project_bounds,a.outside_start,a.inward_steps,a.bench_vectors,a.corners,a.enrich_vectors,a.replay_samples


def context():
    rows,membership,source,packets=a.context()
    for f in [Path(__file__),ROOT/'cad/open_electrical_world_assembly_v1/reviews/check.json',ROOT/'cad/open_electrical_world_assembly_v1/reviews/survey.json',ROOT/'cad/open_electrical_world_assembly_v1/reviews/evidence.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source,packets


def inheritance(lookup):
    exact69=a.inheritance(lookup);q=n.p.read(ROOT/'cad/open_electrical_world_assembly_v1/reviews/check.json');paths=q['paths'][:3]
    assert all(n.m.drive.sha(ROOT/f)==h for f,h in q['source_sha256'].items())
    assert len(paths)==3 and sum(len(p['samples']) for p in paths)==84
    state=set(exact69['carrier11_names']);world=set(q['initial_world718_names']);assert len(world)==718
    for i,p in enumerate(paths):
        present=state if i<2 else world;moving=set(p['moving_names'])
        assert set(p['present_names'])==present and not moving&present and p['completed']
        assert all(not q['native'][s['id']]['overlaps'] and not q['native'][s['id']]['operation_failures'] for s in p['samples'])
        assert np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))<1e-10
        if i==2:assert moving==state==set(q['module_contracts'][0]['body13_names'])
        present.update(moving)
    assert len(world)==731 and sorted(world)==q['packet_final_present_names']
    return dict(source_report='cad/open_electrical_world_assembly_v1/reviews/check.json',paths=paths,sample_scope=84,new_native_groups=0,exact69=exact69,all_inputs_registered_geometry_hashes_unchanged=True,exact_moving_and_present_sets_unchanged=True,final_world731_names=sorted(world),registered_bounds={k:n.c.m.pack._bbox(lookup[k]['shape']) for k in sorted(state)})

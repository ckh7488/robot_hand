"""Original hardware, with both rack nuts placed before either port kit."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm076_prior',ROOT/'cad/branched_module_assembly_v1/model.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
n,np,d=p.n,p.np,p.d
OLD,NAME=p.OLD,p.NAME
translate,moved,bbox,project_bounds,outside_start,inward_steps,bench_vectors=p.translate,p.moved,p.bbox,p.project_bounds,p.outside_start,p.inward_steps,p.bench_vectors


def context():
    rows,membership,source,packets=p.context()
    for f in [Path(__file__),HERE/'reviews/survey.json',ROOT/'cad/branched_module_assembly_v1/reviews/check.json',ROOT/'cad/branched_module_assembly_v1/reviews/selected_check.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source,packets


def inheritance():
    q=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/check.json')
    result=p.inheritance()
    nuts={f'thumb_ip_actuator_rack_nut_{i}' for i in [1,2]}
    for label,paths,count in [('independent_port1_kit42',q['paths'][:6],42),('same_electrical_carrier25_prefix64',q['paths'][6:14],64)]:
        names={k for path in paths for k in path['moving_names']+path['present_names']}
        assert not names&nuts and all(path['completed'] for path in paths)
        assert sum(len(path['samples']) for path in paths)==count
        assert all(not q['native'][s['id']]['overlaps'] and not q['native'][s['id']]['operation_failures'] for path in paths for s in path['samples'])
        assert all(n.m.drive.sha(ROOT/f)==h for f,h in q['source_sha256'].items())
        result.append(dict(id=label,paths=paths,names=sorted(names),sample_scope=count,rack_nuts_absent_from_all_moving_and_present=True,source_chain_hashes_unchanged=True,new_native_groups=0))
    assert [z['sample_scope'] for z in result]==[14,42,42,64]
    return result


def replay_samples(path):
    ii={0,len(path['samples'])-1}
    if not path['completed']:ii.update(range(len(path['samples'])))
    elif len(path['moving_names'])==7:ii.update(range(max(0,len(path['samples'])-3),len(path['samples'])))
    elif path['stage']=='world':ii.update(range(max(0,len(path['samples'])-5),len(path['samples'])))
    if 'base_nuts' in path['id']:
        ii.update(i for i,s in enumerate(path['samples']) if s.get('actual_face_event',False))
    return [path['samples'][i] for i in sorted(ii)]

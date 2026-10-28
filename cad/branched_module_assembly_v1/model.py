"""Assembly study of the explicit ARM074 support replacement and original hardware."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm075_prior',ROOT/'cad/branched_terminal_support_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
translate,moved,bbox,project_bounds,outside_start,inward_steps,bench_vectors=a.p.translate,a.p.moved,a.p.bbox,a.p.project_bounds,a.p.outside_start,a.p.inward_steps,a.p.bench_vectors
OLD,NAME=a.OLD,a.NAME


def mapped(names):return sorted(NAME if k==OLD else k for k in names)


def context():
    rows,membership,source=a.context();original=a.p.packets(rows)
    raw=n.p.read(ROOT/'cad/branched_terminal_support_v1/reviews/check.json')
    fresh=n.c.load(n.p.read(ROOT/raw['scene'])['parts'],source)[0]
    assert fresh['name']==NAME
    rows=[z for z in rows if z['name']!=OLD]+[fresh];assert len(rows)==886
    membership=dict(membership);membership.pop(OLD,None);membership[NAME]='forearm'
    for f in [Path(__file__),HERE/'reviews/survey.json',ROOT/'cad/branched_terminal_support_v1/reviews/check.json',ROOT/'cad/branched_terminal_support_v1/reviews/continuation_check.json',ROOT/'cad/branched_terminal_support_v1/reviews/mounting_datum.json',ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json',ROOT/'cad/special_thumb_module_service_v1/reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source,original


def inheritance():
    motor=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/check.json');kit=n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json')
    selected=[('motor_bench14',motor,motor['paths'][:2]),('independent_original_port0_kit42',kit,kit['paths'][:6])]
    result=[]
    for label,owner,paths in selected:
        names={k for p in paths for k in p['moving_names']+p['present_names']}
        assert not names&{OLD,NAME} and all(p['completed'] for p in paths)
        assert all(not owner['native'][s['id']]['overlaps'] and not owner['native'][s['id']]['operation_failures'] for p in paths for s in p['samples'])
        assert all(n.m.drive.sha(ROOT/f)==h for f,h in owner['source_sha256'].items())
        result.append(dict(id=label,paths=paths,names=sorted(names),sample_scope=sum(len(p['samples']) for p in paths),support_absent_from_all_moving_and_present=True,source_chain_hashes_unchanged=True,new_native_groups=0))
    assert [z['sample_scope'] for z in result]==[14,42]
    return result


def replay_samples(path):
    indices={0,len(path['samples'])-1}
    if not path['completed']:indices.update(range(len(path['samples'])))
    elif path['stage']=='carrier_bench' and len(path['moving_names'])==7:indices.update(range(max(0,len(path['samples'])-3),len(path['samples'])))
    elif path['stage']=='world':indices.update(range(max(0,len(path['samples'])-5),len(path['samples'])))
    return [path['samples'][i] for i in sorted(indices)]

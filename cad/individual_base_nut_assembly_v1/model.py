"""Use the four original compound solids as independent moving fasteners."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm077_prior',ROOT/'cad/preloaded_rack_nut_module_v1/model.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
n,np,d=p.n,p.np,p.d
OLD,NAME=p.OLD,p.NAME;BASE='thumb_ip_actuator_base_nuts'
translate,moved,bbox,project_bounds,outside_start,inward_steps,bench_vectors=p.translate,p.moved,p.bbox,p.project_bounds,p.outside_start,p.inward_steps,p.bench_vectors


def context():
    rows,membership,source,packets=p.context()
    for f in [Path(__file__),HERE/'reviews/survey.json',ROOT/'cad/preloaded_rack_nut_module_v1/reviews/selected_check.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source,packets


def split_nuts(lookup):
    original=lookup[BASE];solids=original['shape'].Solids();assert len(solids)==4
    return [dict(name=BASE+'__solid_'+str(i),shape=sh,body=original.get('body','forearm'),source_compound=BASE,source_solid_index=i,not_additional_hardware=True) for i,sh in enumerate(solids)]


def replay_samples(path):
    ii={0,len(path['samples'])-1}
    if not path['completed']:ii.update(range(len(path['samples'])))
    ii.update(i for i,s in enumerate(path['samples']) if s.get('phase_boundary',False))
    return [path['samples'][i] for i in sorted(ii)]

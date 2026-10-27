"""Preassemble the captured adjuster kit before entering the original R5 base."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm073_prior',ROOT/'cad/special_thumb_module_service_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
translate,moved,bbox,packets,project_bounds,outside_start,inward_steps,bench_vectors=a.translate,a.moved,a.bbox,a.packets,a.project_bounds,a.outside_start,a.inward_steps,a.bench_vectors


def context():
    rows,membership,source=a.context()
    for f in [Path(__file__),HERE/'reviews/survey.json',ROOT/'cad/special_thumb_module_service_v1/reviews/check.json',ROOT/'cad/special_thumb_module_service_v1/reviews/selected_check.json',ROOT/'cad/special_thumb_module_service_v1/reviews/continuation_check.json',ROOT/'cad/special_thumb_module_service_v1/reviews/continuation_common_prepared.json',ROOT/'cad/sheath_retention_v1/retention_model.py',ROOT/'cad/motor_tendon_terminal_v1/terminal_model.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source


def replay_samples(path):
    indexes={0,len(path['samples'])-1}
    if not path['completed']:indexes.update(range(len(path['samples'])))
    elif path['stage']=='carrier_bench':indexes.update(range(max(0,len(path['samples'])-3),len(path['samples'])))
    return [path['samples'][i] for i in sorted(indexes)]

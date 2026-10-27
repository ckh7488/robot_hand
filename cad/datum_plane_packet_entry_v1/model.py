"""Original packet geometry;registered entry along its final mounting datum."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm071_prior',ROOT/'cad/open_guide_module_assembly_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
translate,moved,bbox,packets,project_bounds,outside_start,inward_steps=a.translate,a.moved,a.bbox,a.packets,a.project_bounds,a.outside_start,a.inward_steps
bench_vectors=a.bench_vectors


def context():
    rows,membership,source=a.context()
    for f in [Path(__file__),HERE/'reviews/survey.json',ROOT/'cad/open_guide_module_assembly_v1/reviews/continuation_check.json',ROOT/'cad/open_guide_module_assembly_v1/reviews/assessment.json',ROOT/'cad/shared_roll_root_v1/model.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source


def radial_entry_steps(start):
    return list(np.arange(start,19.,-20.))+[15.,10.,5.,0.]


def replay_samples(path):
    indexes={0,len(path['samples'])-1}
    if path['stage']=='world':
        count=5 if path['id'].endswith('__body19_final_datum_radial') else 4 if '__rack_M3_' in path['id'] else 3
        indexes.update(range(max(0,len(path['samples'])-count),len(path['samples'])))
    return [path['samples'][i] for i in sorted(indexes)]

"""Original special thumbIP48, with its complete service interface retained."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm072_prior',ROOT/'cad/datum_plane_packet_entry_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
translate,moved,bbox,packets,project_bounds,outside_start,inward_steps=a.translate,a.moved,a.bbox,a.packets,a.project_bounds,a.outside_start,a.inward_steps
bench_vectors=a.bench_vectors


def context():
    rows,membership,source=a.context()
    for f in [Path(__file__),HERE/'reviews/survey.json',ROOT/'cad/datum_plane_packet_entry_v1/reviews/check.json',ROOT/'cad/integrated_motor_end_v1/model.py',ROOT/'cad/integrated_motor_end_v1/README.md',ROOT/'cad/motor_spool_coupling_v1/model.py',ROOT/'cad/motor_spool_coupling_v1/selected/reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source


def replay_samples(path):
    indexes={0,len(path['samples'])-1}
    if not path['completed']:indexes.update(range(len(path['samples'])))
    elif path['stage']=='world':indexes.update(range(max(0,len(path['samples'])-5),len(path['samples'])))
    return [path['samples'][i] for i in sorted(indexes)]

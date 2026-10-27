"""Original rear modules with open-groove electrical reference assembly."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm070_prior',ROOT/'cad/rear_packet_restoration_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
translate,moved,bbox,packets,project_bounds,outside_start,inward_steps=a.translate,a.moved,a.bbox,a.packets,a.project_bounds,a.outside_start,a.inward_steps


def context():
    rows,membership,source=a.context()
    for f in [Path(__file__),ROOT/'cad/rear_packet_restoration_v1/reviews/check.json',ROOT/'cad/rear_packet_restoration_v1/reviews/common_prepared.json',ROOT/'cad/wire_guides_v1/model.py',ROOT/'cad/wire_guides_v1/service_check.py',ROOT/'cad/wire_guides_v1/generated/scene.json',ROOT/'cad/remote_motor_lead_v2/generated/scene.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source


def bench_vectors(moving,fixed,axis):
    lo,_=project_bounds(moving,axis);_,hi=project_bounds(fixed,axis)
    start=float(max(80.,np.ceil((hi-lo)/20.)*20.+20.))
    return [x*np.array(axis) for x in list(np.arange(start,0.,-20.))+[10.,5.,0.]]


def replay_samples(path):
    indexes={0,len(path['samples'])-1}
    if path['stage']=='world' and not path['id'].endswith('four_M2_base_nuts_last'):
        count=4 if '__rack_M3_' in path['id'] else 3
        indexes.update(range(max(0,len(path['samples'])-count),len(path['samples'])))
    return [path['samples'][i] for i in sorted(indexes)]

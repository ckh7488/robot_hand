"""Original rear packets;temporary assembly translations in each actual basis."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm069_prior',ROOT/'cad/captive_module_assembly_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
translate,moved,bbox,packets=a.translate,a.moved,a.bbox,a.packets


def context():
    rows,membership,source=a.context()
    for f in [Path(__file__),ROOT/'cad/captive_module_assembly_v1/reviews/selected_check.json',ROOT/'cad/captive_module_assembly_v1/reviews/entry_extension.json',ROOT/'cad/eight_bank_layout_v1/model.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source


def project_bounds(rows,axis):
    lo,hi=bbox(rows)
    corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
    values=corners@np.array(axis);return float(values.min()),float(values.max())


def outside_start(moving,fixed,axis,offset):
    # A rounded outward translation places the complete AABB beyond all stage
    # obstacles. The spacing is a test start,not a required design clearance.
    lo,_=project_bounds(moved(moving,translate(offset)),axis);_,hi=project_bounds(fixed,axis)
    return float(max(160.,np.ceil((hi-lo)/20.)*20.+20.))


def inward_steps(start):return list(np.arange(start,0.,-20.))+[0.]

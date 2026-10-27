"""A declared tilted bolt approach through the gap between paired rack spokes."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm067_prior',ROOT/'cad/ordered_roll_root_assembly_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
ROOT_NAME=a.a.NAME


def context():
    rows,membership,source=a.context()
    for f in [Path(__file__),ROOT/'cad/ordered_roll_root_assembly_v1/reviews/selected_check.json',ROOT/'cad/ordered_roll_root_assembly_v1/reviews/evidence.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source


def translate(v):return a.translation(v)


def front_pose(x,theta_deg,tip_z,approach_distance=0.):
    # Source shaft tip is Z8 at the unchanged front clamp axis. A positive Y
    # rotation sends the negative-X bolt's tail outward into the paired-spoke gap.
    theta=np.deg2rad(theta_deg);R=np.eye(4)
    R[:3,:3]=[[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]]
    tip=np.array([x,-162.5,tip_z])-approach_distance*R[:3,2]
    return translate(tip)@R@translate([-x,162.5,-8.])


def front_route(x):
    sign=1. if x<0 else -1.;states=[]
    for distance in [120.,100.,80.,60.,40.,20.,0.]:
        states.append(dict(phase='diagonal_entry',theta_deg=45.*sign,tip_z_mm=-26.,approach_distance_mm=distance,
            transform=front_pose(x,45.*sign,-26.,distance)))
    controls=[(45.,-26.),(35.,-25.),(25.,-23.),(15.,-21.),(5.,-19.),(0.,-18.)]
    for left,right in zip(controls,controls[1:]):
        for t in [.25,.5,.75,1.]:
            angle=(1-t)*left[0]+t*right[0];z=(1-t)*left[1]+t*right[1]
            states.append(dict(phase='align_while_inserting',theta_deg=angle*sign,tip_z_mm=z,approach_distance_mm=0.,
                transform=front_pose(x,angle*sign,z)))
    for z in [-10.,0.,8.]:
        states.append(dict(phase='axial_seating',theta_deg=0.,tip_z_mm=z,approach_distance_mm=0.,transform=front_pose(x,0.,z)))
    assert len(states)==30 and np.max(abs(states[-1]['transform']-np.eye(4)))<1e-12
    return states


def moved(rows,M):return [dict(z,shape=n.g.original.transform(z['shape'],M)) for z in rows]


def bbox(rows):return a.bbox(rows)

"""Two provisional three-wire main runs, not a completed16motor bus."""
import importlib.util
import math
from pathlib import Path
import sys

import cadquery as cq
from cadquery.occ_impl.shapes import setThreads
import numpy as np

setThreads(1)
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
c=module('_bus_current_pack',ROOT/'cad/palm_motor_layout_v1/check.py')
b,m,p=c.b,c.m,c.p
MODULES=['index_spread_actuator','middle_spread_actuator']


def extend(row,end_y=-350.):
    lane=int(row['name'][-1])-1;radius=[10.,12.,14.][lane]
    start=np.asarray(row['world_end_mm'],float)
    old_direction=np.asarray(row['local']['end_mm'])-row['local']['arc_end_mm']
    tangent=np.asarray(row['transform_world_mm'])[:3,:3]@old_direction
    tangent/=np.linalg.norm(tangent)
    assert np.allclose(tangent,[-1,0,0],rtol=0,atol=1e-8)
    middle=start+[-radius/math.sqrt(2),-radius*(1-1/math.sqrt(2)),0]
    bendend=start+[-radius,-radius,0]
    end=np.array([bendend[0],end_y,start[2]])
    assert end_y<bendend[1]
    path=cq.Wire.assembleEdges([cq.Edge.makeThreePointArc(cq.Vector(*start),cq.Vector(*middle),cq.Vector(*bendend)),
        cq.Edge.makeLine(cq.Vector(*bendend),cq.Vector(*end))])
    plane=cq.Plane(origin=tuple(start),xDir=(0,0,1),normal=(-1,0,0))
    shape=cq.Workplane(plane).circle(.95).sweep(cq.Workplane().newObject([path]),isFrenet=True).val()
    length=math.pi*radius/2+(bendend[1]-end_y)
    assert shape.isValid() and len(shape.Solids())==1 and abs(shape.Volume()-math.pi*.95**2*length)<1e-4
    return dict(name=row['name']+'_main_run',source_lead=row['name'],module=row['module'],body='forearm',
        role='provisional_continuous_wire_extension',shape=shape,color=[[.95,.35,.2],[.95,.76,.12],[.1,.7,.7]][lane],
        start_mm=start.tolist(),arc_middle_mm=middle.tolist(),arc_end_mm=bendend.tolist(),end_mm=end.tolist(),
        start_tangent=tangent.tolist(),end_tangent=[0,-1,0],centreline_radius_mm=radius,
        centreline_length_mm=float(length),outer_diameter_mm=1.9)


def build(rows,end_y=-350.):
    selected=[z for z in rows if z.get('module') in MODULES and z.get('role')=='nominal_electrical_wire_space']
    assert len(selected)==6
    return [extend(z,end_y) for z in selected]


def terminations(wires):
    result=[]
    for name in MODULES:
        group=[z for z in wires if z['module']==name]
        pts=np.asarray([z['end_mm'] for z in group]);centre=pts.mean(axis=0)+[0,-10,0]
        # A reserved volume outside the wire end faces, not an installed board.
        shape=cq.Workplane('XY').box(14,20,6).translate(centre.tolist()).val()
        result.append(dict(name=name+'_temporary_termination_space',module=name,body='forearm',shape=shape,
            role='provisional_board_connector_access_reservation',color=[.65,.4,.85],size_mm=[14,20,6],centre_mm=centre.tolist(),
            supplier_part=None,installed=False))
    return result


def pairs(rows):
    checks=[c.material([z],rows[i+1:]) for i,z in enumerate(rows) if i+1<len(rows)]
    return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in checks),exact_pairs=sum(z['exact_pairs'] for z in checks),
        overlaps=[r for z in checks for r in z['overlaps']],operation_failures=[r for z in checks for r in z['operation_failures']])


def clear(value):return not value['overlaps'] and not value['operation_failures']

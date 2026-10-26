"""Fixed palm lead space, preserving the registered port and all rigid parts."""
import importlib.util
import math
from pathlib import Path
import sys

import cadquery as cq
from cadquery.occ_impl.shapes import setThreads
import numpy as np

setThreads(1)
sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location('_palm_lead_mount', ROOT/'cad/palm_motor_layout_v1/check.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
b, m, p = c.b, c.m, c.p


def lead(y, radius=6., stub=1., straight=8., angle=180.):
    start = np.array([-25.6, y, -8.])
    a = start + [-stub, 0, 0]
    mid = a + [-radius/math.sqrt(2), 0, -radius*(1-1/math.sqrt(2))]
    arcend = a + [-radius, 0, -radius]
    end = arcend + [0, 0, -straight]
    path = cq.Wire.assembleEdges([cq.Edge.makeLine(cq.Vector(*start), cq.Vector(*a)),
        cq.Edge.makeThreePointArc(cq.Vector(*a), cq.Vector(*mid), cq.Vector(*arcend)),
        cq.Edge.makeLine(cq.Vector(*arcend), cq.Vector(*end))])
    plane = cq.Plane(origin=tuple(start), xDir=(0,1,0), normal=(-1,0,0))
    shape = cq.Workplane(plane).circle(.95).sweep(cq.Workplane().newObject([path]), isFrenet=True).val()
    length = stub + radius*math.pi/2 + straight
    if not shape.isValid() or len(shape.Solids()) != 1 or abs(shape.Volume()-math.pi*.95**2*length)>1e-4:
        raise ValueError('Invalid lead sweep')
    shape = shape.rotate((0,y,-8), (1,y,-8), -angle)
    t = math.radians(-angle)
    R = np.array([[1,0,0],[0,math.cos(t),-math.sin(t)],[0,math.sin(t),math.cos(t)]])
    pivot = np.array([0,y,-8.])
    point = lambda v: (pivot + R@(v-pivot)).tolist()
    return shape, dict(start_mm=point(start), arc_start_mm=point(a), arc_middle_mm=point(mid),
        arc_end_mm=point(arcend), end_mm=point(end), centreline_radius_mm=radius,
        straight_stub_mm=stub, terminal_straight_mm=straight, bend_plane_angle_deg=angle,
        centreline_length_mm=length, outer_diameter_mm=1.9, supplied_cable_verified=False)


def wires(motor, **parameters):
    rows=[]
    for i,y in enumerate([-11.5,-9.,-6.5],1):
        shape, local = lead(y, **parameters)
        T=np.asarray(motor['transform_world_mm'])
        rows.append(dict(name=motor['name']+'_palm_lead'+str(i), module=motor['name'],
            body='palm', role='provisional_electrical_lead_space', local=local,
            transform_world_mm=T.tolist(), shape=m.pack._transform(shape,T),
            world_start_mm=(T@np.r_[local['start_mm'],1])[:3].tolist(),
            world_end_mm=(T@np.r_[local['end_mm'],1])[:3].tolist(), color=[[.95,.35,.2],[.95,.76,.12],[.1,.7,.7]][i-1]))
    return rows


def plugs(motors):
    ref=p.read(ROOT/'cad/wiring_access_v10/connector_reference.json')['connector']
    shape=cq.Solid.makeBox(*ref['registered_outer_size_mm'],cq.Vector(*ref['registered_outer_origin_mm'])).translate((-14.5,0,0))
    return [dict(name=z['name']+'_palm_plug_outer',module=z['name'],body='palm',
        role='solid_outer_box_not_mating_geometry',color=[.7,.7,.7],
        shape=m.pack._transform(shape,z['transform_world_mm'])) for z in motors]


def pairs(rows):
    checks=[c.material([r],rows[i+1:]) for i,r in enumerate(rows) if i+1<len(rows)]
    return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in checks),exact_pairs=sum(z['exact_pairs'] for z in checks),
        overlaps=[r for z in checks for r in z['overlaps']],operation_failures=[r for z in checks for r in z['operation_failures']])


def clear(check):
    return not check['overlaps'] and not check['operation_failures']

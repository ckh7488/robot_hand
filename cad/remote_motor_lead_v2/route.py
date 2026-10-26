"""Route only the four blocked remote modules around unchanged rack posts."""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    value=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


old=module('_post_old',ROOT/'cad/remote_motor_lead_v1/batch.py')
c,b,m,p=old.c,old.b,old.m,old.p
CHANGED={'middle_pip_actuator','middle_dip_actuator','ring_pip_actuator','ring_dip_actuator'}


def lead_shape(local,angle_deg):
    shape,_=c.lead_shape(local['straight_stub_mm'],local['start_mm'][1],local['centreline_radius_mm'])
    y,z=local['start_mm'][1:]
    # Rotate the bend plane while preserving the connector exit and its tangent.
    shape=shape.rotate((0,y,z),(1,y,z),-angle_deg)
    theta=np.radians(-angle_deg)
    rotation=np.array([[1.,0.,0.],[0.,np.cos(theta),-np.sin(theta)],[0.,np.sin(theta),np.cos(theta)]])
    centre=np.array([0.,y,z])
    desc=dict(local,bend_plane_angle_deg=angle_deg,engineering_geometry_only=True)
    for key in ['start_mm','arc_start_mm','arc_middle_mm','arc_end_mm','end_mm']:
        desc[key]=(rotation@(np.asarray(local[key])-centre)+centre).tolist()
    return shape,desc


def saved_rows():
    rows=p.read(ROOT/'cad/remote_motor_lead_v1/generated/batch_scene.json')['parts']
    for row in rows:
        for key,hkey in [('step','sha256_step'),('mesh','sha256_stl')]:
            if b.sha(ROOT/row[key])!=row[hkey]:
                raise ValueError('Changed inherited lead/connector')
    return rows


def build_leads(angle_deg,rows):
    leads=[]
    for row in rows:
        if row['role']!='nominal_electrical_wire_space':
            continue
        if row['module'] in CHANGED:
            shape,local=lead_shape(row['local'],angle_deg)
            shape=m.pack._transform(shape,row['transform_world_mm'])
        else:
            shape=cq.importers.importStep(str(ROOT/row['step'])).val()
            local=row['local']
        leads.append(dict(name=row['name'],module=row['module'],shape=shape,local=local,source=row))
    return leads

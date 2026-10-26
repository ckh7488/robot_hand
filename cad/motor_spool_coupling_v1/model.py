"""One current thumb-IP motor output, with an explicit stock-horn interface.

No hand or motor mounting changes. The stock rotor reference is split without
altering its solids; an existing R10 two-tail spool is adapted for four nominal
M2x6 TAP envelopes. This is a coupling candidate, not a complete tendon drive.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import cadquery as cq
import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    obj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(obj)
    return obj


c = module('_drive_coupling_helpers', ROOT/'cad/palm_motor_layout_v1/check.py')
b, m, p = c.b, c.m, c.p
a = module('_drive_stock_reference', ROOT/'cad/thumb_ip_drive_v1/motor_adapter.py')
NAME = 'thumb_ip_actuator'
PARAMS = dict(motor=NAME, spool_radius_mm=10., cord_nominal_diameter_mm=.6,
    groove_x_mm=[2.,10.], mounting_PCD_mm=12., web_thickness_mm=3.5,
    counterbore_radius_mm=2.3, shaft_clearance_radius_mm=1.1,
    mounting_screw_nominal='ROBOTIS PHS M2x6 TAP family; smooth nominal envelope',
    mounting_screw_length_mm=6., nominal_engagement_mm=2.5,
    nominal_head_diameter_mm=4.2, nominal_head_height_mm=1.6,
    screw_head_dimensions_verified=False, screw_driver_verified=False,
    horn_front_x_mm=0., tool_access_radius_mm=1.05,
    factory_output_retention='Original vendor-reference central screw preserved; not replaced with a guessed metal-horn screw',
    whole_tendon_route_complete=False, coupling_strength_qualified=False)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pose(shape, angle=0., shift=0.):
    layout=p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    T=next(z['transform_world_mm'] for z in layout['modules'] if z['name']==NAME)
    shape=shape.rotate((0,0,0),(1,0,0),angle)
    if shift: shape=shape.translate((shift,0,0))
    return m.pack._transform(shape,T)


def build():
    vendor=a.identify_vendor_parts(cq,ROOT)
    spool,anchors,clamps=a._spool(cq)
    holes=[(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)]
    for y,z in holes:
        spool=spool.cut(cq.Solid.makeCylinder(2.3,14.,cq.Vector(3.5,y,z),cq.Vector(1,0,0)))
    spool=spool.clean()
    rows=[]
    def add(name,shape,role,moving=True,color=(.9,.56,.15)):
        assert shape.isValid() and len(shape.Solids())==1 and shape.Volume()>0
        rows.append(dict(name=name,local_shape=shape,shape=pose(shape),role=role,
                         module=NAME,body='forearm',moving=moving,color=list(color)))
    add('thumb_ip_stock_output_horn',vendor['shapes']['removed_front_plastic_horn'],
        'unmodified_stock_reference_horn',color=(.18,.35,.65))
    add('thumb_ip_stock_central_retainer',vendor['shapes']['removed_old_central_screw'],
        'unmodified_stock_reference_retainer',color=(.38,.4,.43))
    add('thumb_ip_stock_rear_idler',vendor['shapes']['preserved_rear_idler'],
        'unmodified_stock_reference_idler',False,(.4,.43,.47))
    add('thumb_ip_two_tail_spool',spool,'custom_counterbored_dual_groove_spool')
    for i,(y,z) in enumerate(holes,1):
        shaft=cq.Solid.makeCylinder(1.,6.,cq.Vector(-2.5,y,z),cq.Vector(1,0,0))
        head=cq.Solid.makeCylinder(2.1,1.6,cq.Vector(3.5,y,z),cq.Vector(1,0,0))
        # Head/drive remain assumptions. A simple cross indicates a tool opening;
        # it is not an identified Phillips size or a supplier machining model.
        cross=cq.Solid.makeBox(1.,.5,2.8,cq.Vector(4.5,y-.25,z-1.4)).fuse(
              cq.Solid.makeBox(1.,2.8,.5,cq.Vector(4.5,y-1.4,z-.25)))
        add('thumb_ip_spool_tap6_'+str(i),shaft.fuse(head).cut(cross).clean(),
            'nominal_tapping_screw_not_vendor_cad',color=(.4,.44,.49))
    for i,s in enumerate(clamps,1):
        add('thumb_ip_tail_clamp_'+str(i),s,'retained_nominal_M2x3_flat_tip_clamp',color=(.35,.4,.45))
    return rows,dict(parameters=PARAMS,anchors=anchors,vendor_identification=vendor['identification'])


def custom(rows):
    return [z for z in rows if not z['role'].startswith('unmodified_stock')]


def moved(rows,angle=0.,shift=0.):
    return [dict(z,shape=pose(z['local_shape'],angle if z['moving'] else 0.,shift)) for z in rows]


def pairs(rows):
    results=[c.material([z],rows[i+1:]) for i,z in enumerate(rows)]
    return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in results),
        exact_pairs=sum(z['exact_pairs'] for z in results),
        overlaps=[v for z in results for v in z['overlaps']],
        operation_failures=[v for z in results for v in z['operation_failures']])


def thread_contacts(rows):
    """Confine nominal shaft/tap-minor overlap to measured mounting holes only."""
    horn=next(z for z in rows if z['name']=='thumb_ip_stock_output_horn')['shape']
    output=[]
    for i,(y,z) in enumerate([(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)],1):
        screw=next(q for q in rows if q['name']=='thumb_ip_spool_tap6_'+str(i))['shape']
        common=screw.intersect(horn,tol=1e-6)
        region=pose(cq.Solid.makeCylinder(1.00001,2.50002,cq.Vector(-2.50001,y,z),cq.Vector(1,0,0)))
        outside=float(common.cut(region,tol=1e-6).Volume())
        output.append(dict(screw=i,common_volume_mm3=common.Volume(),outside_declared_region_mm3=outside,
            acceptable_nominal_thread_envelope=outside<=.001,
            reason='Smooth M2 major envelope vs unmodified vendor R0.8 tap-minor; only X[-2.5,0],PCD12 bores.Helix and tapping strength unverified.'))
    return output


def fixed_inputs(source=None):
    mount=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json')
    meta=mount['new_parts']+mount['retained_parts']
    replaced=[z for z in meta if z['name'] in [NAME+'_rotor',NAME+'_spool']]
    retained=[z for z in meta if z not in replaced]
    assert len(replaced)==2 and len(retained)==358
    fixed=c.load(retained,source)
    wires=c.load(p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'],source)
    arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+z['asset']+'.step'
        if source is not None: source[rel]=sha(ROOT/rel)
        arm.append(dict(name=z['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    return fixed,wires,arm,retained,replaced


def clear(test):
    return not test['overlaps'] and not test['operation_failures']


def unexpected_internal(test):
    expected={('thumb_ip_stock_output_horn','thumb_ip_spool_tap6_'+str(i)) for i in range(1,5)}
    unexpected=[z for z in test['overlaps'] if (z['lead'],z['part']) not in expected]
    return unexpected

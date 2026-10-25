"""One thumb-IP tendon attachment candidate, all geometry in local joint mm.

Import is inert. Supply CadQuery and the saved native child STEP shape in the
IP frame to build_joint(). This is an explicit two-tail friction clamp, not a
claim of measured retention, friction, pretension or dynamic performance.
"""
from __future__ import annotations
import math
import importlib.util
from pathlib import Path


def _geometry():
    spec=importlib.util.spec_from_file_location('_thumb_drive_geometry',Path(__file__).with_name('geometry.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

PARAMETERS = dict(
    tendon_nominal_diameter_mm=.6, tendon_tracks_x_mm=[-4.,4.],
    tendon_centre_radius_mm=7.7, groove_tool_major_radius_mm=7.75, groove_tool_radius_mm=.35,
    anchor_angle_deg=0., tail_length_mm=5.,
    tail_cutter_start_overlap_mm=.5,
    exit_channel_width_mm=.8, exit_channel_radii_mm=[7.4,12.],
    exit_channel_angles_deg=[-200.,180.],
    tail_channel_shape='open_U_flat_floor', tail_channel_width_mm=.8,
    tail_channel_radial_span_mm=[7.4,10.],
    boss_width_mm=15.8, boss_radial_span_mm=[5.,7.4], boss_tangent_span_mm=[-6.3,6.3],
    clamp_width_mm=17.4, clamp_radial_span_mm=[8.,10.], clamp_tangent_span_mm=[-6.3,6.3],
    bolt_positions_xt_mm=[[0.,-3.8],[0.,3.8]],
    bolt_major_diameter_mm=2., bolt_length_including_head_mm=6.,
    bolt_head_diameter_mm=4.7, bolt_head_height_mm=1.35,
    bolt_socket_af_mm=1.3, bolt_socket_depth_mm=.75,
    nut_af_mm=4., nut_height_mm=1.6, nut_radial_span_mm=[4.7,6.3],
    nut_slot_tangent_width_mm=4.2, nut_slot_radial_span_mm=[4.65,6.35],
    clamp_relief_width_mm=17.6, clamp_relief_radial_span_mm=[7.75,10.2],
    clamp_relief_tangent_span_mm=[-6.4,6.4],
    lower_head_relief_radius_mm=2.45, lower_head_relief_y_mm=[10.,15.2],
    assembly_window_centre_xy_mm=[0.,32.], assembly_window_length_mm=14.,
    assembly_window_width_mm=6.4, assembly_window_extrusion_mm=13.,
    clamp_fit_status='nominal geometry; rope compression and retention unmeasured',
)


def _box(cq, size, centre):
    return cq.Workplane('XY').box(*size).translate(centre).val()


def _radial_box(cq, width, radial, tangent):
    return _box(cq,(width,radial[1]-radial[0],tangent[1]-tangent[0]),
                (0,sum(radial)/2,sum(tangent)/2))


def _ycyl(cq, radius, y0, y1, x=0., t=0.):
    return cq.Solid.makeCylinder(radius,y1-y0,cq.Vector(x,y0,t),cq.Vector(0,1,0))


def _exit_cutters(cq):
    """Open running channels with the same R7.4 floor and R7.7 rope centre."""
    p=PARAMETERS;tools=[];width=p['exit_channel_width_mm']
    inner,outer=p['exit_channel_radii_mm']
    for x,degrees in zip(p['tendon_tracks_x_mm'],p['exit_channel_angles_deg']):
        a,b=0.,math.radians(degrees);mid=(a+b)/2
        def point(r,t):return cq.Vector(x-width/2,r*math.cos(t),r*math.sin(t))
        wire=cq.Wire.assembleEdges([
            cq.Edge.makeLine(point(inner,a),point(outer,a)),
            cq.Edge.makeThreePointArc(point(outer,a),point(outer,mid),point(outer,b)),
            cq.Edge.makeLine(point(outer,b),point(inner,b)),
            cq.Edge.makeThreePointArc(point(inner,b),point(inner,mid),point(inner,a))])
        tools.append(cq.Solid.extrudeLinear(wire,[],cq.Vector(width,0,0)))
    return tools


def _hex(cq, af, y0, y1, x=0., t=0.):
    r=af/math.sqrt(3)
    wire=cq.Wire.makePolygon([cq.Vector(x+r*math.cos(i*math.pi/3),y0,t+r*math.sin(i*math.pi/3)) for i in range(6)],close=True)
    return cq.Solid.extrudeLinear(wire,[],cq.Vector(0,y1-y0,0))


def anchor_transform(shape, angle=None):
    if angle is None:angle=PARAMETERS['anchor_angle_deg']
    return shape.rotate((0,0,0),(1,0,0),angle)


def make_anchor_screw(cq,x,t):
    """Accu SSK-M2-6-A2 nominal external dimensions; smooth thread envelope."""
    p=PARAMETERS
    # A 90 degree cone from R1 to R2.35 is 1.35 mm long, matching this SKU.
    s=_ycyl(cq,1.,4.,8.65,x,t).fuse(cq.Solid.makeCone(1.,2.35,1.35,cq.Vector(x,8.65,t),cq.Vector(0,1,0)))
    s=s.cut(_hex(cq,1.3,9.25,10.01,x,t)).clean()
    return anchor_transform(s)


def make_anchor_nut(cq,x,t):
    # Nominal M2 nut dimensions must be bound to the selected supplier facts.
    p=PARAMETERS;lo,hi=p['nut_radial_span_mm']
    return anchor_transform(_hex(cq,p['nut_af_mm'],lo,hi,x,t).cut(_ycyl(cq,1.,lo-.01,hi+.01,x,t)).clean())


def build_joint(cq, native_child_local):
    """Return modified native child and separately fastened clamp hardware.

    The pin, bushings, bearing seats and joint frame are retained. Nut-loading
    slots open at the side of the structural eye; the support/assembly checker
    must establish that loading those slots is possible before installing the
    link in the parent fork. Smooth bores do not model thread contact stresses.
    """
    p=PARAMETERS;geo=_geometry();journal=[]
    geo.require_solid(native_child_local,'native child')
    boss=anchor_transform(_radial_box(cq,p['boss_width_mm'],p['boss_radial_span_mm'],p['boss_tangent_span_mm']))
    child=geo.boolean(cq,'fuse',native_child_local,boss)
    journal.append(dict(stage='boss_union',**geo.require_solid(child,'boss union')))
    # Cut the exits before torus/slot intersections. Late subtraction from
    # the completed native topology produced invalid/empty BReps in probes.
    exits=_exit_cutters(cq)
    for i,tool in enumerate(exits):geo.require_solid(tool,'exit cutter '+str(i))
    child=geo.boolean(cq,'cut',child,*exits)
    journal.append(dict(stage='early_exit_after_boss_before_torus',**geo.require_solid(child,'open exits')))
    clamp=_radial_box(cq,p['clamp_width_mm'],p['clamp_radial_span_mm'],p['clamp_tangent_span_mm'])
    cuts=[]
    for x in p['tendon_tracks_x_mm']:
        # R7.75 - tool R0.35 + rope R0.30 seats the working centre at R7.70.
        cuts.append(cq.Solid.makeTorus(p['groove_tool_major_radius_mm'],p['groove_tool_radius_mm'],cq.Vector(x,0,0),cq.Vector(1,0,0)))
        sign=1 if x<0 else -1
        # Open flat tail channel avoids the closed groove's exit lip.
        # Nominal contact is retained; actual grip/edge finish need tests.
        overlap=p['tail_cutter_start_overlap_mm']
        lo,hi=p['tail_channel_radial_span_mm']
        tail=_box(cq,(p['tail_channel_width_mm'],hi-lo,5.8+overlap),
            (x,(lo+hi)/2,sign*(5.8-overlap)/2))
        cuts.append(anchor_transform(tail))
    for x,t in p['bolt_positions_xt_mm']:
        # Through clearance is distinct from the captive nut's thread bore.
        cuts.append(anchor_transform(_ycyl(cq,1.1,3.85,8.01,x,t)))
        lo,hi=((-8.3,x+2.4) if x<0 else (x-2.4,8.3))
        nlo,nhi=p['nut_slot_radial_span_mm']
        slot=_box(cq,(hi-lo,nhi-nlo,p['nut_slot_tangent_width_mm']),((lo+hi)/2,(nlo+nhi)/2,t))
        cuts.append(anchor_transform(slot))
        clamp=clamp.cut(_ycyl(cq,1.1,7.99,10.01,x,t))
        # Nominal conical bearing surface matches the selected screw envelope.
        countersink=cq.Solid.makeCone(1.1,2.35,1.25,cq.Vector(x,8.75,t),cq.Vector(0,1,0))
        clamp=clamp.cut(countersink)
    for index,cutter in enumerate(cuts):
        child=geo.boolean(cq,'cut',child,cutter)
        journal.append(dict(stage='cut_'+str(index),**geo.require_solid(child,'child cut '+str(index))))
    # Material clearance and entry for real screw heads, not just driver tips.
    assembly_cuts=[
        ('clamp_relief',anchor_transform(_radial_box(cq,p['clamp_relief_width_mm'],p['clamp_relief_radial_span_mm'],p['clamp_relief_tangent_span_mm']))),
        ('lower_screw_head_relief',_ycyl(cq,p['lower_head_relief_radius_mm'],*p['lower_head_relief_y_mm'],0.,-3.8)),
        ('upper_screw_entry_window',cq.Workplane('XY').center(*p['assembly_window_centre_xy_mm']).slot2D(p['assembly_window_length_mm'],p['assembly_window_width_mm'],90.).extrude(p['assembly_window_extrusion_mm']).val()),
    ]
    for name,cutter in assembly_cuts:
        before=geo.describe(child)['volume_mm3'];child=geo.boolean(cq,'cut',child,cutter)
        journal.append(dict(stage=name,removed_mm3=before-geo.describe(child)['volume_mm3'],**geo.require_solid(child,name)))
    # UnifySameDomain/clean corrupted this native torus/slot topology in a
    # reproducible probe. Preserve the valid non-destructive BRep directly.
    clamp=anchor_transform(clamp.clean())
    parts=[dict(name='thumb_ip_tendon_clamp',shape=clamp,body='thumb_ip',role='removable_two_tail_clamp',material='aluminum')]
    for i,(x,t) in enumerate(p['bolt_positions_xt_mm'],1):
        parts.extend([
            dict(name=f'thumb_ip_tendon_clamp_screw_{i}',shape=make_anchor_screw(cq,x,t),body='thumb_ip',role='M2x6_countersunk_major_envelope',material='steel'),
            dict(name=f'thumb_ip_tendon_clamp_nut_{i}',shape=make_anchor_nut(cq,x,t),body='thumb_ip',role='M2_hex_nut_major_envelope',material='steel'),
        ])
    for row in [dict(name='thumb_segment3_tip_core',shape=child),*parts]:
        geo.require_solid(row['shape'],row['name'])
    return dict(child=child,parts=parts,parameters=dict(p),operation_journal=journal,limitations=[
        'Candidate only; raw printed contact surfaces are not approved rope running surfaces.',
        'The clamp is shown at nominal contact with undeformed 0.6 mm rope; clamp force, compression, slip and damage require tests.',
        'All threads are smooth major-diameter envelopes; full thread fit, minimum engagement and tightening torque remain unverified.',
        'Nut loading precedes hinge assembly. Screws enter through the upper link window before the tip cap is installed; assembly audit is separate.',
        'The uncovered assembly window changes a local upper wall. Its strength and exposed-rope protection require physical qualification.',
        'Open U channels retain nominal R7.4 floors; slack-rope retention, edge finishing and tolerance remain unqualified.',
        'Tendon surface quality, real diameter, bending fatigue, stiffness and creep remain unmeasured.',
    ])


def tendon_geometry(q_deg, motor_radius=10., motor_distance=120.):
    """Exact common exterior tangents for a fixed-parent one-joint bench.

    No Bowden sheath, upstream-joint cancellation, servo dynamics or tension
    equilibrium is assumed here. Both tracks remain coplanar with axis X.
    """
    p=PARAMETERS; r=p['tendon_centre_radius_mm']; a=math.radians(p['anchor_angle_deg']+q_deg)
    ny=(motor_radius-r)/motor_distance
    if abs(ny)>=1:raise ValueError('No common exterior tangent')
    nz=math.sqrt(1-ny*ny)
    result=[]
    for x,side in [(-4.,-1),(4.,1)]:
        contact=math.atan2(side*nz,ny)
        wrap=(a-contact) if side<0 else (contact-a)
        if not 0<wrap<2*math.pi:raise ValueError('Tendon lost wrap')
        jc=[x,r*ny,side*r*nz]
        mc=[x,-motor_distance+motor_radius*ny,side*motor_radius*nz]
        tangent=[0.,-math.sin(a),math.cos(a)]
        if side>0:tangent=[-v for v in tangent]
        anchor=[x,r*math.cos(a),r*math.sin(a)]
        result.append(dict(track_x_mm=x,contact_angle_rad=contact,anchor_angle_rad=a,wrap_rad=wrap,
            joint_contact_mm=jc,motor_contact_mm=mc,anchor_mm=anchor,
            tail_end_mm=[anchor[i]+5*tangent[i] for i in range(3)],
            exposed_straight_mm=math.sqrt(motor_distance**2-(motor_radius-r)**2),
            wrapped_joint_mm=r*wrap))
    return result

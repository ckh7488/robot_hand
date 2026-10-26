"""Keep the mounting web, put the rotating tail bosses beyond the adapter."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as original
from model import *


def build():
    rows,design=original.build()
    collar=cq.Solid.makeCylinder(9.,2.,cq.Vector(0,0,0),cq.Vector(1,0,0)).cut(
        cq.Solid.makeCylinder(2.9,2.02,cq.Vector(-.01,0,0),cq.Vector(1,0,0)))
    for y,z in [(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)]:
        collar=collar.cut(cq.Solid.makeCylinder(1.1,2.02,cq.Vector(-.01,y,z),cq.Vector(1,0,0)))
    for row in rows:
        if row['name']=='thumb_ip_two_tail_spool':
            shape=row['local_shape'].translate((2.,0,0)).fuse(collar)
            # Total load-bearing web stays3.5mm; the original M2x6 shaft still
            # enters the stock horn2.5mm. No longer screw enters the motor.
            for y,z in [(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)]:
                shape=shape.cut(cq.Solid.makeCylinder(2.3,15.,cq.Vector(3.5,y,z),cq.Vector(1,0,0)))
            row['local_shape']=shape.clean();row['shape']=pose(row['local_shape'])
        elif row['name'].startswith('thumb_ip_tail_clamp_'):
            row['local_shape']=row['local_shape'].translate((2.,0,0));row['shape']=pose(row['local_shape'])
        assert row['local_shape'].isValid() and len(row['local_shape'].Solids())==1
    design['correction']=dict(axial_neck_length_mm=2.,neck_outer_radius_mm=9.,
        spool_and_tail_boss_shift_x_mm=2.,groove_x_mm=[4.,12.],
        total_mounting_web_mm=3.5,fastener_length_mm=6.,nominal_horn_engagement_mm=2.5,
        reason='First candidate rotating tail boss reaches fixed adapter;shift boss outside its axial span,retain actual collar/bolts.Not a motor installation or hand geometry change.')
    for anchor in design['anchors']:
        anchor['groove_x_mm']+=2.
        for key in ['tail_start_mm','tail_exit_mm','screw_tip_mm','screw_outer_mm','cord_center_at_clamp_mm']:anchor[key][0]+=2.
    return rows,design


def custom_envelopes(rows):
    # Every rear mounting shaft has X[-2.5,0],radius from the motor axis[5,7].
    # The integral neck is exactly an R9 cylinder in X[0,2]. All other new
    # material is in X[2,18.1]. Its AABB gives a conservative radial bound.
    remaining=[z for z in custom(rows) if z['name']=='thumb_ip_two_tail_spool' or z['name'].startswith('thumb_ip_tail_clamp_')]
    radius=0.;xmax=2.
    for z in remaining:
        box=np.asarray(m.pack._bbox(z['local_shape']));radius=max(radius,float(np.linalg.norm(np.max(abs(box[:,1:]),axis=0))));xmax=max(xmax,box[1,0])
    radial= cq.Solid.makeCylinder(7.00001,2.50002,cq.Vector(-2.50001,0,0),cq.Vector(1,0,0)).cut(
        cq.Solid.makeCylinder(4.99999,2.50004,cq.Vector(-2.50002,0,0),cq.Vector(1,0,0)))
    neck=cq.Solid.makeCylinder(9.00001,2.00002,cq.Vector(-.00001,0,0),cq.Vector(1,0,0))
    front=cq.Solid.makeCylinder(radius+.00001,xmax-2.+.00002,cq.Vector(1.99999,0,0),cq.Vector(1,0,0))
    return [dict(name=name,shape=pose(shape)) for name,shape in [('full_rotation_TAP_shaft_annulus',radial),('full_rotation_neck_bound',neck),('full_rotation_drum_and_clamps_bound',front)]],dict(front_radius_mm=radius,front_x_mm=[2.,float(xmax)],neck_radius_mm=9.,neck_x_mm=[0.,2.],shaft_radii_mm=[5.,7.],shaft_x_mm=[-2.5,0.],
        proof='Canonical X rotation preserves X and sqrt(Y²+Z²).Exact collar/shaft primitives plus conservative local AABB radial maxima bound every angle,not just vertices at sampled poses.Numerical padding1e-5mm.')

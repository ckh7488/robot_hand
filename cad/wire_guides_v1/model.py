"""Removable wire guide candidates carried by the existing rack bolt pattern.

The groove captures wires transversely. Axial slip, clamping force, wear and
strain relief are not qualified. Fasteners are nominal geometric envelopes.
"""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_guide_tail',ROOT/'cad/remote_motor_lead_v2/route.py')
t=importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
b,m,p=t.b,t.m,t.p


def box(size,centre):
    return cq.Workplane('XY').box(*size).translate(centre).val()


def cylinder(radius,length,start):
    return cq.Solid.makeCylinder(radius,length,cq.Vector(*start))


def hexagon(af,height,z,xy=(0.,0.)):
    return cq.Workplane('XY').polygon(6,af*2/np.sqrt(3)).extrude(height).translate((xy[0],xy[1],z)).val()


def build():
    wiremeta=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts']
    modules=[r for r in p.read(ROOT/'cad/wiring_layout_v1/generated/layout.json')['modules'] if r['zone']!='palm']
    parts=[]
    groups=[]
    def add(name,shape,group,kind,color):
        if not shape.isValid() or len(shape.Solids())!=1:
            raise ValueError('New guide component must be one valid solid: '+name)
        parts.append(dict(name=name,module=group,kind=kind,color=color,shape=shape))
    for motor in modules:
        name=motor['name'];cx,cy,cz=motor['module_envelope_center_mm'];bx=cx-16
        wire=next(r for r in wiremeta if r['name']==name+'_lead2')
        transform=np.asarray(wire['transform_world_mm'])
        end=np.asarray(wire['world_end_mm'])
        local=wire['local']
        tangent=transform[:3,:3]@(np.asarray(local['end_mm'])-np.asarray(local['arc_end_mm']))
        tangent/=np.linalg.norm(tangent)
        side=np.cross([0.,0.,1.],tangent)
        centre=end-5*tangent
        frame=np.eye(4);frame[:3,0]=tangent;frame[:3,1]=side;frame[:3,3]=[centre[0],centre[1],0.]
        place=lambda shape:m.pack._transform(shape,frame)
        # Existing adapter flange underside Z-79, groove floor at nominal wire bottom.
        foot=box((4,36,2),(bx-7.5,cy,-80))
        for dy in [-13,13]:
            pad=box((9.1,9,2),(bx-0.95,cy+dy,-80))
            foot=foot.fuse(pad)
        foot=foot.fuse(box((4,4,4),(bx-7.5,centre[1],-79)))
        low=min(centre[0]-2,bx-9.5);high=max(centre[0]+2,bx-5.5)
        foot=foot.fuse(box((high-low,4,2),((low+high)/2,centre[1],-78)))
        foot=foot.fuse(box((4,4,5.25),(centre[0],centre[1],-74.375)))
        base=box((6,20,4.4),(0,0,-69.55))
        base=base.cut(box((8,7.5,5),(0,0,-67.25)))
        lid=box((6,20,1.5),(0,0,-66.6))
        for offset in [-7.2,7.2]:
            base=base.cut(cylinder(1.1,8,(0,offset,-74)))
            base=base.cut(hexagon(4.2,1.6,-71.75,(0,offset)))
            lid=lid.cut(cylinder(1.1,4,(0,offset,-68)))
        foot=foot.fuse(place(base))
        for k,dy in enumerate([-13,13],1):
            foot=foot.cut(cylinder(1.7,5,(bx,cy+dy,-83)))
            # A 1mm spacer preserves nut engagement without raising the nut into the wire.
            spacer=cylinder(3,1,(bx,cy+dy,-82)).cut(cylinder(1.7,2,(bx,cy+dy,-82.5)))
            screw=cylinder(1.5,12,(bx,cy+dy,-82)).fuse(cylinder(2.85,2,(bx,cy+dy,-84)))
            add(name+'_rack_screw_'+str(k),screw.clean(),name,'replacement_M3x12_envelope',[.55,.58,.62])
            add(name+'_guide_spacer_'+str(k),spacer.clean(),name,'custom_1mm_spacer_envelope',[.7,.72,.75])
        add(name+'_wire_guide_base',foot.clean(),name,'guide_base',[.24,.65,.57])
        add(name+'_wire_guide_lid',place(lid),name,'removable_lid',[.5,.78,.71])
        for k,offset in enumerate([-7.2,7.2],1):
            screw=cylinder(1.,6,(0,offset,-71.85)).fuse(cylinder(1.75,1.5,(0,offset,-65.85)))
            nut=hexagon(4.,1.6,-71.75,(0,offset)).cut(cylinder(1.05,3,(0,offset,-72)))
            add(name+'_lid_screw_'+str(k),place(screw.clean()),name,'M2x6_envelope',[.55,.58,.62])
            add(name+'_lid_nut_'+str(k),place(nut.clean()),name,'M2_hex_nut_envelope',[.7,.72,.75])
        groups.append(dict(module=name,motor_centre_mm=[cx,cy,cz],bolt_axis_xy_mm=[[bx,cy-13],[bx,cy+13]],
            frame_world_mm=frame.tolist(),guide_centre_mm=centre.tolist(),groove_floor_z_mm=-69.75,
            spacer_mm=1.,M3_shaft_z_mm=[-82,-70],old_nut_z_mm=[-73,-70.6],nominal_nut_tip_projection_mm=.6,
            note='Existing M3 head is an inherited envelope, not a selected supplier part. Groove supports/captures wires; axial pull restraint remains unqualified.'))
    return parts,groups

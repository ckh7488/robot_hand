"""Connect unchanged palm adapters to two existing posts with split collars."""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
from cadquery.occ_impl.shapes import setThreads
import numpy as np

setThreads(1)
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_palm_mount_prior',ROOT/'cad/palm_motor_layout_v1/check.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
p,b,m=c.p,c.b,c.m


def box(size,centre):return cq.Workplane('XY').box(*size).translate(centre).val()
def cyl(r,h,start,axis=(0,0,1)):return cq.Solid.makeCylinder(r,h,cq.Vector(*start),cq.Vector(*axis))


def hex_y(af,h,point):
    return cq.Workplane('XZ').polygon(6,af*2/np.sqrt(3)).extrude(h).translate(point).val()


def load_scene(source=None):
    saved=p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/scene.json')
    rows=c.load(saved['changed_parts']+saved['retained_packaging_guide'],source)
    replaced=[r for r in rows if r.get('body')=='palm' and r['name'].endswith('_adapter')]
    retained=[r for r in rows if r not in replaced]
    assert len(replaced)==4 and len(retained)==349
    return replaced,retained


def build(adapters,params=None):
    a=params or p.read(HERE/'parameters.json')
    z=a['floor_z_mm'];height=a['collar_height_mm'];cz=z+height/2
    carrier=adapters[0]['shape']
    for row in adapters[1:]:carrier=carrier.fuse(row['shape'])
    # Rails remain above the existing floor and below the motor cases.
    rail_z=z+a['rail_height_mm']/2
    for size,centre in [((66,4,a['rail_height_mm']),(20,44,rail_z)),
                        ((4,6,a['rail_height_mm']),(-13,47,rail_z)),
                        ((22,4,a['rail_height_mm']),(-24,50,rail_z)),
                        ((4,7,a['rail_height_mm']),(-19,53.5,rail_z)),
                        ((4,56,a['rail_height_mm']),(8,72,rail_z)),
                        ((4,9,a['rail_height_mm']),(25,48.5,rail_z))]:
        carrier=carrier.fuse(box(size,centre))
    rows=[];groups=[];holes=[];nut_seats=[]
    def add(name,shape,kind,color):
        shape=shape.clean()
        if not shape.isValid() or len(shape.Solids())!=1:raise ValueError('Invalid/split new mount '+name)
        rows.append(dict(name=name,shape=shape,body='palm',module='palm_carrier',kind=kind,color=color))
    for i,(x,y) in enumerate(a['posts_xy_mm'],1):
        ring=cyl(a['collar_outer_radius_mm'],height,(x,y,z)).cut(cyl(a['post_radius_mm'],height+2,(x,y,z-1)))
        gap=a['split_half_gap_mm']
        main=ring.intersect(box((40,20,10),(x,y-10-gap,cz)))
        cap=ring.intersect(box((40,20,10),(x,y+10+gap,cz)))
        for side in [-1,1]:
            xx=x+side*a['ear_offset_x_mm']
            main=main.fuse(box((5,5,height),(xx,y-2.5-gap,cz)))
            cap=cap.fuse(box((5,5,height),(xx,y+2.5+gap,cz)))
            hole=cyl(a['screw_hole_radius_mm'],20,(xx,y-10,cz),(0,1,0));holes.append(hole)
            cap=cap.cut(hole)
            end_y=y+5+gap
            cap=cap.cut(cyl(a['nominal_head_radius_mm']+a['head_spotface_extra_radius_mm'],a['head_spotface_length_mm'],(xx,end_y,cz),(0,1,0)))
            nut_seats.append(hex_y(a['nominal_nut_AF_mm']+a['nut_seat_extra_AF_mm'],a['nut_seat_length_mm'],(xx,y-5-gap,cz)))
            screw=cyl(a['nominal_screw_radius_mm'],a['nominal_screw_length_mm'],(xx,end_y,cz),(0,-1,0))
            screw=screw.fuse(cyl(a['nominal_head_radius_mm'],a['nominal_head_length_mm'],(xx,end_y,cz),(0,1,0)))
            screw=screw.cut(hex_y(a['nominal_socket_AF_mm'],1,(xx,end_y+a['nominal_head_length_mm'],cz)))
            nut=hex_y(a['nominal_nut_AF_mm'],a['nominal_nut_length_mm'],(xx,y-5-gap,cz))
            nut=nut.cut(cyl(a['nominal_nut_bore_radius_mm'],3,(xx,y-4.8,cz),(0,-1,0)))
            tag=str(i)+'_'+('left' if side<0 else 'right')
            add('Palm_collar_'+tag+'_screw',screw,'nominal_M2x12_socket_envelope',[.5,.54,.58])
            add('Palm_collar_'+tag+'_nut',nut,'nominal_M2_nut_envelope',[.63,.66,.7])
            groups.append(dict(id=tag,post=i,bolt_x_mm=xx,post_y_mm=y,axis_z_mm=cz,
                screw='Palm_collar_'+tag+'_screw',nut='Palm_collar_'+tag+'_nut',cap='Palm_collar_'+str(i)+'_cap',
                head_outer_y_mm=end_y+a['nominal_head_length_mm']))
        add('Palm_collar_'+str(i)+'_cap',cap,'removable_collar_cap',[.24,.65,.5])
        carrier=carrier.fuse(main)
    # Recut after all unions; the rails cannot fill a post bore or screw passage.
    for x,y in a['posts_xy_mm']:carrier=carrier.cut(cyl(a['post_radius_mm'],height+2,(x,y,z-1)))
    for hole in holes:carrier=carrier.cut(hole)
    for pocket in nut_seats:carrier=carrier.cut(pocket)
    add('Palm_four_motor_carrier',carrier,'four_adapter_carrier',[.94,.53,.14])
    assert len(rows)==11
    return rows,groups


def tool(group,params):
    x,y,z=group['bolt_x_mm'],group['head_outer_y_mm'],group['axis_z_mm']
    shape=cyl(params['tool_stem_radius_mm'],params['tool_stem_length_mm'],(x,y,z),(0,1,0))
    return shape.fuse(hex_y(params['tool_tip_AF_mm'],1,(x,y,z))).clean()

"""Open radial seats and keyed collars on the NEW guide; hand parts unchanged."""
from probe import *

def build():
    prior,design=g.build();old={z['name']:z for z in prior};parts=[]
    def add(name,s,role):
        s=g.original.require(s.clean());parts.append(dict(name=name,shape=g.original.transform(s,g.original.WORLD),local_shape=s,body='thumb_mcp',role=role,color=old[name]['color']))
    carrier=None
    for seat in design['seats']:
        x,y,z=seat['x_mm'],seat['y_mm'],seat['bottom_z_mm']
        foot=g.original.cyl(3.2,5.,(x,y,z),(0,0,1));carrier=foot if carrier is None else carrier.fuse(foot)
        add(seat['bolt'],old[seat['bolt']]['local_shape'],'unchanged_nominal_extended_cover_bolt')
    carrier=carrier.fuse(g.original.box([-3.2,design['seats'][0]['y_mm'],15.8],[3.2,design['seats'][1]['y_mm'],17.2]))
    carrier=carrier.fuse(g.original.box([0,-31,15.8],[20,-25,17.2])).fuse(g.original.box([16,-33,-22],[20,-23,22]))
    def zbox(xlo,xhi,ulo,uhi,zlo,zhi,side):
        za,zb=sorted([zlo*side,zhi*side]);return g.original.box([xlo,ulo,za],[xhi,uhi,zb])
    def zcyl(r,lo,hi,side):return g.original.cyl(r,hi-lo,(6,0,lo*side),(0,0,side))
    def tools():
        return [g.original.cylinder_y(.65,-4.01,4.01),g.original.cylinder_y(1.6,0,2),g.original.box([1.1,0,-.6],[2.7,2,.6])]
    for row in g.original.layout():
        i=row['cord'];side=row['side'];Q=row['socket_transform']
        base=zbox(-5,20-Q[0,3],-4,4,-4,0,side);carrier=carrier.fuse(g.original.transform(base,Q))
        key=g.original.transform(g.original.box([1.2,0,-.5],[2.6,2,.5]),Q)
        add('flanged_liner_'+str(i),old['flanged_liner_'+str(i)]['local_shape'].fuse(key),'same_OD_ID_R5_liner_with_positive_index_tab')
        cap=zbox(-5,9,-4,4,0,4,side)
        for tool in tools()+[zcyl(1.1,-4.01,5.21,side)]:cap=cap.cut(tool)
        add('liner_flange_keeper_'+str(i),g.original.transform(cap,Q),'radially_removable_half_socket_cover')
        bolt=zcyl(1.,-6,4,side).fuse(zcyl(1.9,4,5.2,side))
        add('liner_keeper_M2x10_'+str(i),g.original.transform(bolt,Q),'nominal_radial_M2x10_smooth_envelope_no_thread_drive_approval')
        pts=[cq.Vector(6+2.31*math.cos(k*math.pi/3),2.31*math.sin(k*math.pi/3),-5.6*side) for k in range(6)]
        nut=cq.Solid.extrudeLinear(cq.Wire.makePolygon(pts,close=True),[],cq.Vector(0,0,1.6*side)).cut(zcyl(1.,-5.61,-3.99,side))
        add('liner_keeper_M2_nut_'+str(i),g.original.transform(nut,Q),'nominal_radial_M2_nut_no_threads')
    for row in g.original.layout():
        side=row['side'];Q=row['socket_transform']
        for tool in tools()+[zbox(-5.01,9.01,-4.01,4.01,0,4.01,side),zcyl(1.1,-6.01,6.01,side),zcyl(2.8,-12,-4,side)]:
            carrier=carrier.cut(g.original.transform(tool,Q))
    for seat in design['seats']:
        carrier=carrier.cut(g.original.cyl(1.1,8,(seat['x_mm'],seat['y_mm'],seat['bottom_z_mm']-.1),(0,0,1)))
    add('paired_liner_carrier',carrier,'same_mount_feet_bridge_with_two_open_radial_half_sockets')
    design=dict(design,source_sha256=dict(design['source_sha256']),radial_socket_split=True,key_tab_mm=dict(x=[1.2,2.6],u=[0,2],z=[-.5,.5]),key_seat_clearance_mm=.1,hand_body_geometry_changed=False,motor_positions_changed=False,
        guide_axes_fixed_in_body=False,guide_roll_force_approved=False,liner_forming_strength_fatigue_verified=False,
        correction='Replace only new guide interfaces with radial split sockets and indexed collars.Unchanged liner centerlines,OD1.2/ID0.8/R5,accepted hand364,motor positions and electrical geometry.No operating angle restriction.')
    for name in ['probe.py','radial_model.py']:
        design['source_sha256']['cad/thumb_guide_assembly_v2/'+name]=g.g.drive.sha(HERE/name)
    return parts,design

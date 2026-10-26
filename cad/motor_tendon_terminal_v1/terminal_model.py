"""One rigid, rack-mounted twin fairlead and two independent sheath adjusters.

This is a current-motor terminal, not an already-connected thumb Bowden route.
All dimensions are engineering candidates; threads and strength unqualified.
"""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'cad/thumb_guide_assembly_v2'))
import probe as prior
from material_cache import install
g,c,p,np,cq=prior.g,prior.c,prior.p,prior.np,prior.cq
install(c)
route=g.g;drive=route.drive;M=route.M
NAME='thumb_ip_actuator'
SPEC=dict(motor=NAME,cord_OD_mm=.6,guide_bore_mm=.8,guide_outer_mm=3.2,guide_bend_radius_mm=5.,free_tangent_mm=15.,
 sheath_OD_mm=3.,sheath_ID_mm=1.3,barrel_thread_nominal='M5x0.8,not helical CAD or selected supply part',barrel_stroke_candidate_mm=[-2.,2.],
 barrel_shaft_z_mm=[-157.,-137.],sheath_seat_z_mm=-158.,sheath_mouth_z_mm=-163.,upper_beam_z_mm=[-133.,-130.],
 lower_beam_z_mm=[-144.,-141.],guide_end_z_mm=-134.,retained_M3_stack_z_mm=[-82.,-81.],
 source_hand_motor_wire_positions_changed=False,whole_motor_to_thumb_route_complete=False,tension_or_strength_approved=False)
def box(lo,hi):return cq.Solid.makeBox(*(np.asarray(hi)-lo),cq.Vector(*lo))
def cyl(r,lo,hi,xy):return cq.Solid.makeCylinder(r,hi-lo,cq.Vector(*xy,lo))
def hexnut(xy,lo,hi):
    return cq.Workplane('XY').polygon(6,8*2/math.sqrt(3)).extrude(hi-lo).translate((*xy,lo)).val().cut(cyl(2.5,lo-.01,hi+.01,xy))
def shift(s,z):return s.translate((0,0,z))
def sweep_piece(seg,r):return g.original.piece_shape(seg,r)
def parts_valid(s):
    assert s.isValid() and len(s.Solids())==1 and s.Volume()>1e-6
    return s
def channels():
    result=[]
    for i,(x,side,mx) in enumerate(zip([-4.,4.],[-1.,1.],[4.,12.])):
        B=side*math.pi/2
        E=route.circle(M[:3,3],M[:3,:3],mx,10.,B)
        A=E+[0,15,0];Z=A+[0,5,-5];mid=A+[0,5/math.sqrt(2),-5+5/math.sqrt(2)];end=np.array([Z[0],Z[1],SPEC['guide_end_z_mm']])
        result.append(dict(cord=i,exit=E,entry=A,end=end,xy=Z[:2],pieces=[route.arc(A,mid,Z,5*math.pi/2,'fixed_fairlead_bend'),route.line(Z,end,'fixed_fairlead_straight')]))
    return result
def build():
    parts=[];ports=channels();bx=-11.;ys=[-287.,-261.]
    def add(name,s,role,color):parts.append(dict(name=name,shape=parts_valid(s.clean()),role=role,body='forearm',color=color))
    # Same two 1mm spacer seats and unchanged M3 bolts; positive face support.
    frame=cyl(3.,-82,-81,(bx,ys[0])).fuse(cyl(3.,-82,-81,(bx,ys[1])))
    frame=frame.fuse(box([-11,-290,-82],[-5,-258,-81]))
    frame=frame.fuse(box([-8,-289,-144],[-5,-285,-82]))
    for lo,hi in [(-144,-141),(-133,-130)]:
        frame=frame.fuse(box([-8,-289,lo],[23.5,-285,hi]))
        for port in ports:
            x,y=port['xy'];frame=frame.fuse(box([x-2,-287,lo],[x+2,y+2,hi]))
    x0,x1=ports[0]['xy'][0],ports[1]['xy'][0];yf=ports[0]['xy'][1]
    frame=frame.fuse(box([x0-4,yf-4,-144],[x1+4,yf+4,-141]))
    for port in ports:
        for seg in port['pieces']:frame=frame.fuse(sweep_piece(seg,1.6))
    # Cut after the unions so support ties cannot fill the running channels.
    for port in ports:
        for seg in port['pieces']:frame=frame.cut(sweep_piece(seg,.4))
        frame=frame.cut(cyl(2.55,-144.01,-140.99,port['xy']))
    for y in ys:frame=frame.cut(cyl(1.7,-82.01,-80.99,(bx,y)))
    add('thumb_ip_twin_sheath_terminal_frame',frame,'rack_spacer_replacement_and_integral_fixed_fairleads',[.2,.64,.63])
    for port in ports:
        i=port['cord'];xy=port['xy'];barrel=cyl(2.5,-157,-137,xy).fuse(cyl(3.5,-163,-157,xy))
        barrel=barrel.cut(cyl(.65,-163.01,-136.99,xy)).cut(cyl(1.55,-163.01,-158,xy))
        add('thumb_ip_barrel_adjuster_'+str(i),barrel,'nominal_M5x0p8_sheath_seat_adjuster',[.85,.6,.21])
        for side,lo,hi in [('top',-141,-138.6),('bottom',-146.4,-144)]:
            add('thumb_ip_barrel_'+side+'_nut_'+str(i),hexnut(xy,lo,hi),'nominal_M5_jam_nut_no_threads',[.45,.48,.52])
    design=dict(parameters=SPEC,replaced_packaging=[NAME+'_guide_spacer_1',NAME+'_guide_spacer_2'],
        mounting_axes_xy_mm=[[bx,y] for y in ys],spacer_top_z_mm=-81.,spacer_bottom_z_mm=-82.,
        port_centres_world_mm=[z['end'].tolist() for z in ports],sheath_seat_centres_world_mm=[[*z['xy'],-158.] for z in ports],
        support_body='forearm;fixed rack fasteners,not rotating drum',adjustment_is_static_setting_not_dynamic_compensator=True,
        guide_shapes_are_integral_to_frame=True,friction_force_equilibrium_verified=False,hand_connection_complete=False,
        source_sha256={'cad/motor_tendon_terminal_v1/terminal_model.py':drive.sha(Path(__file__))})
    return parts,design
def cords(angle=0.):
    rows=[];theta=math.radians(angle);anchors=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/reviews/check.json')['design']['anchors']
    for port,mx,side,ma,mb in zip(channels(),[4.,12.],[-1.,1.],[-210.,210.],[-90.,90.]):
        i=port['cord'];a=math.radians(ma)+theta;b=math.radians(mb);wrap=side*(a-b);assert 0<wrap<2*math.pi
        A=route.circle(M[:3,3],M[:3,:3],mx,10.,a);B=route.circle(M[:3,3],M[:3,:3],mx,10.,b)
        tail=route.point(route.v.rotate('x',angle)@np.asarray(anchors[i]['tail_exit_mm']))
        pieces=[route.line(tail,A,'motor_anchor_tail'),route.arc(A,route.circle(M[:3,3],M[:3,:3],mx,10.,(a+b)/2),B,10*wrap,'motor_wrap'),route.line(B,port['entry'],'free_tangent'),*port['pieces'],route.line(port['end'],[*port['xy'],-163.],'adjuster_passage')]
        for j,seg in enumerate(pieces):rows.append(dict(name='terminal_cord_'+str(i)+'_'+str(j)+'_'+seg['label'],cord=i,shape=sweep_piece(seg,.3),body='forearm',role=seg['label'],length_mm=seg['length_mm'],color=[.95,.28,.1] if i==0 else [.1,.6,.95]))
    return rows
def witnesses():
    return [dict(name='future_sheath_section_'+str(z['cord']),shape=cyl(1.5,-175,-158,z['xy']).cut(cyl(.65,-175.01,-157.99,z['xy'])),role='provisional_short_housing_section_not_complete_thumb_route',body='forearm',color=[.6,.29,.7]) for z in channels()]

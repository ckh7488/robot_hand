"""Paired retained liner terminals on the existing thumb-MCP cover fasteners.

Only the distal guide/cord segment is defined. The motor-to-terminal flexible
conduit/root crossing is NOT supplied by this model.
"""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'cad/thumb_tendon_route_v1'))
import route_model as g
c,b,m,p,cq,np=g.c,g.b,g.m,g.p,g.cq,g.np
DESIGN=g.DESIGN
NEUTRAL=g.v.frames(DESIGN,np.zeros(20));O,R=NEUTRAL['thumb_ip']
WORLD=np.eye(4);WORLD[:3,:3]=R;WORLD[:3,3]=O
INV=np.linalg.inv(WORLD)

def transform(s,T):return m.pack._transform(s,T)
def box(lo,hi):return cq.Solid.makeBox(*(np.asarray(hi)-lo),cq.Vector(*lo))
def cyl(r,length,point,axis):return cq.Solid.makeCylinder(r,length,cq.Vector(*point),cq.Vector(*axis))
def cylinder_y(r,lo,hi,x=0,z=0):return cyl(r,hi-lo,(x,lo,z),(0,1,0))
def piece_shape(seg,radius):
    if seg['kind']=='line':return cyl(radius,seg['length_mm'],seg['a'],seg['tangent'])
    plane=cq.Plane(origin=tuple(seg['a']),normal=tuple(seg['tangent']))
    path=cq.Wire.assembleEdges([seg['edge']])
    return cq.Workplane(plane).circle(radius).sweep(cq.Workplane().newObject([path]),isFrenet=True).val()
def require(s):
    assert s.isValid() and len(s.Solids())==1 and s.Volume()>1e-6
    return s
def local_hand():
    cat=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');source={}
    rows=[z for z in cat['hand'] if z['body']=='thumb_mcp'];shapes=c.load(rows,source)
    return {z['name']:transform(z['shape'],INV) for z in shapes},source

def layout():
    rows=[]
    for i,(x,side) in enumerate(zip([-4.,4.],[-1.,1.])):
        old=g.old.route(0.)[i];T=np.asarray(old['joint_contact_mm']);t=T-np.asarray(old['motor_contact_mm']);t/=np.linalg.norm(t)
        E=T+t*((-8.-T[1])/t[1]);n=np.cross([1.,0.,0.],t)*(-side)
        S=E-10*t-10*n;A=S+5*t+5*n
        first=g.arc(S,S+5*((1-math.cos(math.pi/4))*n+math.sin(math.pi/4)*t),A,5*math.pi/2,'liner_bend_1')
        center=S+10*t+5*n
        second=g.arc(A,center+5*(-math.cos(math.pi/4)*t+math.sin(math.pi/4)*n),E,5*math.pi/2,'liner_bend_2')
        I=S-22*t;origin=S-10*t
        Q=np.eye(4);Q[:3,:3]=np.column_stack(([1.,0.,0.],t,np.cross([1.,0.,0.],t)));Q[:3,3]=origin
        rows.append(dict(cord=i,side=side,track_x=x,t=t,n=n,exit=E,entry=I,target=T,socket_transform=Q,
            pieces=[g.line(I,S,'liner_straight'),first,second],guide_centers=[(S+5*n).tolist(),center.tolist()]))
    return rows

def build():
    hand,source=local_hand();parts=[];replaced=[];rows=layout()
    def add(name,s,role,color):
        require(s);parts.append(dict(name=name,local_shape=s,shape=transform(s,WORLD),body='thumb_mcp',role=role,color=list(color)))
    feet=[];seats=[]
    for i in [1,2]:
        prefix='thumb_segment2_dark_faceplate_clamp'+str(i);washer=hand[prefix+'_washer'];bolt=hand[prefix+'_bolt']
        lo,hi=np.asarray(m.pack._bbox(washer));x,y=(lo[:2]+hi[:2])/2;z=hi[2]
        foot=cyl(3.2,5.,(x,y,z),(0,0,1));feet.append(foot)
        moved=bolt.translate((0,0,5.));blo,bhi=np.asarray(m.pack._bbox(bolt));moved=moved.fuse(cyl(1.,5.,(x,y,blo[2]),(0,0,1))).clean()
        add('guide_cover_M2x12_'+str(i),moved,'nominal_extended_original_M2_envelope',(0.3,.63,.35))
        replaced.append(prefix+'_bolt');seats.append(dict(washer=prefix+'_washer',bolt='guide_cover_M2x12_'+str(i),x_mm=float(x),y_mm=float(y),bottom_z_mm=float(z),head_plane_z_mm=float(z+5.),retained_tip_z_mm=float(blo[2])))
    carrier=feet[0].fuse(feet[1]).fuse(box([-3.2,seats[0]['y_mm'],15.8],[3.2,seats[1]['y_mm'],17.2]))
    for seat in seats:carrier=carrier.cut(cyl(1.1,8,(seat['x_mm'],seat['y_mm'],seat['bottom_z_mm']-.1),(0,0,1)))
    carrier=carrier.fuse(box([0.,-31.,15.8],[20.,-25.,17.2])).fuse(box([16.,-33.,-22.],[20.,-23.,22.]))
    for row in rows:
        i=row['cord'];Q=row['socket_transform'];pipe=None
        for seg in row['pieces']:
            tube=piece_shape(seg,.6).cut(piece_shape(seg,.4));pipe=tube if pipe is None else pipe.fuse(tube)
        flange=transform(cylinder_y(1.5,0.,2.).cut(cylinder_y(.4,-.01,2.01)),Q)
        pipe=pipe.fuse(flange).clean();add('flanged_liner_'+str(i),pipe,'nominal_OD1p2_ID0p8_R5_formed_liner_with_retained_flange',(.95,.55,.15))
        block=box([-5.,-4.,-4.],[20.-Q[0,3],2.,4.]);block=block.cut(cylinder_y(.65,-4.01,2.01)).cut(cylinder_y(1.6,0.,2.01)).cut(cylinder_y(1.1,-4.01,2.01,x=6.))
        carrier=carrier.fuse(transform(block,Q))
        cap=box([-5.,2.,-4.],[9.,4.,4.]);cap=cap.cut(box([-5.01,1.99,-.65],[0.,4.01,.65]).fuse(cylinder_y(.65,1.99,4.01))).cut(cylinder_y(1.1,1.99,4.01,x=6.))
        add('liner_flange_keeper_'+str(i),transform(cap,Q),'side_entry_open_slot_keeper',(.15,.6,.66))
        bolt=cylinder_y(1.,-6.,4.,x=6.).fuse(cylinder_y(1.9,4.,5.2,x=6.))
        # Illustrative socket envelope; no supplied driver/thread qualification.
        bolt=bolt.cut(transform(cq.Workplane('XZ').polygon(6,1.5).extrude(.7).val(),np.eye(4)).translate((6.,5.2,0.)))
        add('liner_keeper_M2x10_'+str(i),transform(bolt,Q),'nominal_M2x10_smooth_shaft',(.3,.63,.35))
        points=[cq.Vector(6.+2.31*math.cos(k*math.pi/3),-5.6,2.31*math.sin(k*math.pi/3)) for k in range(6)]
        nut=cq.Solid.extrudeLinear(cq.Wire.makePolygon(points,close=True),[],cq.Vector(0,1.6,0)).cut(cylinder_y(1.,-5.61,-3.99,x=6.))
        add('liner_keeper_M2_nut_'+str(i),transform(nut,Q),'nominal_M2_nut_no_threads',(.35,.4,.43))
    # Cut the finished union so the cross-brace cannot refill socket bores.
    for row in rows:
        Q=row['socket_transform']
        for tool in [cylinder_y(.65,-4.01,2.01),cylinder_y(1.6,0.,2.01),cylinder_y(1.1,-4.01,2.01,x=6.)]:
            carrier=carrier.cut(transform(tool,Q))
    add('paired_liner_carrier',carrier.clean(),'existing_two_cover_fasteners_and_side_bridge',(.15,.6,.66))
    return parts,dict(source_sha256=source,replaced_hand_parts=replaced,body='thumb_mcp',seats=seats,
        guide_liner_OD_mm=1.2,guide_liner_ID_mm=.8,cord_OD_mm=.6,guide_centerline_radius_mm=5.,flange_OD_mm=3.,flange_length_mm=2.,
        geometry_assumptions_not_supplier_qualified=True,liner_forming_strength_fatigue_verified=False,
        guide_axes_fixed_in_body=True,force_equilibrium_verified=False,upstream_motor_root_conduit_complete=False,
        local_frame='neutral thumb_ip mm;body attachment thumb_mcp',sockets=[dict(cord=z['cord'],transform=z['socket_transform'].tolist(),entry_mm=z['entry'].tolist(),exit_mm=z['exit'].tolist(),guide_centers_mm=z['guide_centers']) for z in rows])

def cord_pieces(q):
    out=[];q=np.asarray(q,float);parent=q.copy();parent[19]=0.;frame=g.v.frames(DESIGN,parent)['thumb_ip'];a=np.deg2rad(q[19])
    for row in layout():
        i=row['cord'];x=row['track_x'];side=row['side'];old=g.old.route(0.)[i];contact=old['contact_angle_rad'];end=np.array([x,7.7*math.cos(a),7.7*math.sin(a)])
        angle=(-side)*(a-contact);assert 0<angle<2*math.pi
        pieces=[*row['pieces'],g.line(row['exit'],row['target'],'joint_approach'),
            g.arc(row['target'],np.array([x,7.7*math.cos((a+contact)/2),7.7*math.sin((a+contact)/2)]),end,7.7*angle,'joint_wrap'),
            g.line(end,end+5*np.array([0.,side*math.sin(a),-side*math.cos(a)]),'joint_anchor_tail')]
        for index,z in enumerate(pieces):
            s=piece_shape(z,.3);T=np.eye(4);T[:3,:3]=frame[1];T[:3,3]=frame[0]
            out.append(dict(name='distal_cord_'+str(i)+'_'+str(index)+'_'+z['label'],cord=i,role=z['label'],shape=transform(s,T),local_shape=s,color=[.95,.29,.12] if i==0 else [.12,.65,.9],body='palm',length_mm=z['length_mm']))
    return out

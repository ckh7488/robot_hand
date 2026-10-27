"""An actual cantilevered sleeve pair outside the rear wrist references.

The six-part clamp remains on the existing saddle edge. A connected bridge
supports the rearward sleeves; stiffness and clamp preload remain unverified.
"""
from feed_survey import m,g,c,p,np,cq,ROOT,HERE

def box(a,b):return cq.Solid.makeBox(*[y-x for x,y in zip(a,b)],cq.Vector(*a))
def cyl(x,z,a,b,r):return cq.Solid.makeCylinder(r,b-a,cq.Vector(x,a,z),cq.Vector(0,1,0))

def build():
    scene=p.read(ROOT/'cad/wrist_outlet_route_v1/inboard/generated/assembly_scene.json');kept=[z for z in scene['retained_parts']];retained=c.load(kept)
    sh=box([45,-9,-6],[62,-6,6]).fuse(box([54,-12,-6],[102,-9,4]))
    sh=sh.fuse(box([54,-9,-58],[62,-6,6])).fuse(box([45,-9,-58],[62,-6,-46])).fuse(box([54,-6,-58],[56,0,-46]))
    for z in [-3.,2.]:sh=sh.fuse(cyl(100,z,-100,-4,2.5))
    for z in [-3.,2.]:sh=sh.cut(cyl(100,z,-112,10,1.6))
    for z in [-55.,-49.]:sh=sh.cut(cyl(59,z,-14,6,1.1))
    sh=sh.clean();assert sh.isValid() and len(sh.Solids())==1
    new=[dict(name='wrist_external_feed_clip',shape=sh,body='palm',attachment_body='Palm_wrist_saddle',installed_part_candidate=True,color=[.1,.68,.65])]
    for i,z in enumerate([-3.,2.]):
        for kind,shape in [('housing',cyl(100,z,-110,8,1.5).cut(cyl(100,z,-110.01,8.01,.65))),('cord',cyl(100,z,-110,8,.3))]:
            new.append(dict(name='external_feed_'+kind+'_'+str(i),shape=shape,kind=kind,physical_line=i,body='palm',installed_part_candidate=False,color=[.66,.27,.83] if kind=='housing' else [.95,.43,.12]))
    return new,retained,dict(replaces='wrist_inboard_guide_clip',new_hardware_rows=1,retained_hardware_rows=5,local_reference_rows=4,attachment_body='Palm_wrist_saddle',saddle_contact_y_mm=[-6,0],saddle_edge_x_mm=54,fastener_axes_xz_mm=[[59,-55],[59,-49]],guide_axis=[0,1,0],guide_axis_x_mm=100,guide_centers_z_mm=[-3,2],guide_y_mm=[-100,-4],guide_length_mm=96,bore_OD_mm=[3.2,5],local_housing_cord_y_mm=[-110,8],local_reference_length_mm=118,housing_OD_ID_mm=[3,1.3],cord_OD_mm=.6,connected_bridge_x_mm=[54,102],projection_past_hand_palm_nominal_max_x_mm=12.5,guide_is_rigid_custom_candidate=True,hand_body_motor_rack_electrical_changed=False,entire_housing_installed=False,clamp_preload_stiffness_strength_manufacturing_exterior_approved=False)

def around(origin,axis,degrees):
    t=np.deg2rad(degrees);co,si=np.cos(t),np.sin(t)
    R=np.array([[1,0,0],[0,co,-si],[0,si,co]]) if axis=='X' else np.array([[co,-si,0],[si,co,0],[0,0,1]])
    T=np.eye(4);T[:3,:3]=R;O=np.asarray(origin);T[:3,3]=O-R@O;return T

def wrist_frames(flex,yaw):
    design=p.read(ROOT/'cad/modular_roots_v9/design.json')['arm'];y=-design['wrist_to_palm']-design['wrist_axis_offset']
    P=around([0,y,0],'X',flex);Y=around([design['wrist_lateral_offset'],-design['wrist_to_palm'],0],'Z',yaw)
    return P,P@Y

def transform(row,T):return dict(row,shape=c.m.pack._transform(row['shape'],T))

def posed_arm(arm,P,H):
    return [transform(z,H if z['name']=='Palm_wrist_saddle' else P) if z['name'] in ['Palm_wrist_saddle','Wrist_box_carrier','AK_wrist_deviation'] else z for z in arm]

"""A removable saddle-edge clip and two actual oblique guide bores.

No drilling of the reference wrist, hand or motor relocation is implied.
The clip needs unverified bolt preload/friction to resist sliding on the edge.
"""
from wrist_survey import m,g,c,p,np,cq,ROOT,HERE

def box(lo,hi):return cq.Solid.makeBox(*[b-a for a,b in zip(lo,hi)],cq.Vector(*lo))
def cylinder(a,b,r):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float);v=b-a
    return cq.Solid.makeCylinder(r,float(np.linalg.norm(v)),cq.Vector(*a),cq.Vector(*v))
def point(y,z):return np.array([37.+y,y,z])

def build():
    back=box([45,-9,-6],[62,-6,6]).fuse(box([54,-6,-6],[56,0,6]))
    back=back.fuse(box([25,-12,-6],[54,-9,-2.5]))
    for z in [-3.,2.]:
        back=back.fuse(cylinder(point(-14,z),point(-4,z),2.5))
    for z in [-3.,2.]:back=back.cut(cylinder(point(-18,z),point(4,z),1.6))
    cap=box([45,0,-6],[62,3,6]);rows=[]
    for z in [-3.,3.]:
        bore=cylinder([59,-14,z],[59,6,z],1.1);back=back.cut(bore);cap=cap.cut(bore)
    for name,sh in [('wrist_oblique_guide_clip',back.clean()),('wrist_edge_clip_front_cap',cap.clean())]:
        rows.append(dict(name=name,shape=sh,body='palm',attachment_body='Palm_wrist_saddle',role='nominal_saddle_edge_clip_bolt_clamped_friction_unqualified',color=[.1,.68,.65],installed_part_candidate=True))
    for j,z in enumerate([-3.,3.]):
        bolt=cylinder([59,-13,z],[59,3,z],1.).fuse(cylinder([59,3,z],[59,4.2,z],1.9))
        pts=[cq.Vector(59+2.31*np.cos(k*np.pi/3),-10.6,z+2.31*np.sin(k*np.pi/3)) for k in range(6)]
        nut=cq.Solid.extrudeLinear(cq.Wire.makePolygon(pts,close=True),[],cq.Vector(0,1.6,0)).cut(cylinder([59,-11,z],[59,-8,z],1.))
        for kind,sh in [('bolt',bolt),('nut',nut)]:
            rows.append(dict(name='wrist_edge_clip_'+kind+'_'+str(j),shape=sh,body='palm',attachment_body='Palm_wrist_saddle',role='nominal_M2x16_or_M2_nut_smooth_envelope_supply_thread_torque_unverified',color=[.5,.53,.58],installed_part_candidate=True))
    for i,z in enumerate([-3.,2.]):
        a,b=point(-18,z),point(18,z)
        rows.append(dict(name='wrist_local_housing_'+str(i),shape=cylinder(a,b,1.5).cut(cylinder(a-.01*(b-a),b+.01*(b-a),.65)),body='palm',attachment_body='free_local_housing_through_real_bore',role='OD3_ID1p3_assumed_local_only_not_full_housing',color=[.66,.27,.83],installed_part_candidate=False))
        rows.append(dict(name='wrist_local_cord_'+str(i),shape=cylinder(a,b,.3),body='palm',attachment_body='nominal_centerline_only',role='cord_OD0p6_assumed_not_taut_wall_contact',color=[.95,.45,.12],installed_part_candidate=False))
    for z in rows:assert z['shape'].isValid() and len(z['shape'].Solids())==1,(z['name'],len(z['shape'].Solids()))
    return rows,dict(new_hardware_rows=6,physical_local_housings=2,physical_local_cords=2,new_geometry_rows=10,attachment_body='Palm_wrist_saddle',attachment='Existing six-mm-thick right outer edge x54,faces y=-6 and0;rear clip integral side stop and separate front cap.Two nominal M2x16 fasteners outside original wall at x59,z+-3.No new holes.',guide_bore_diameter_assumed_mm=3.2,housing_OD_ID_assumed_mm=[3.,1.3],cord_OD_assumed_mm=.6,guide_sleeve_OD_mm=5.,centers_z_mm=[-3.,2.],axis_unit=np.array([1.,1.,0.]).__truediv__(np.sqrt(2.)).tolist(),axis_relation='x=37+y;oblique45deg inXY;no change to hand/motor installation axes',sleeve_centerline_y_mm=[-14.,-4.],local_housing_centerline_y_mm=[-18.,18.],local_housing_length_mm=float(36*np.sqrt(2.)),physical_bracket_load_path='Housing can touch actual guide bore;integral sleeve/arm -> rear clip/front cap/nominal fasteners -> original saddle edge.Friction/slip/preload/load not validated.',new_guide_world_pose_only_neutral_wrist=True,whole_motor_to_thumb_connected=False,wrist_motion_tested=False,hand_body_geometry_changed=False,clip_preload_and_6DOF_stiffness_verified=False,manufacturing_or_exterior_approved=False)

"""Move only the guide bores into the existing opening's inboard lane."""
from outlet_survey import m,g,c,p,np,cq,ROOT,HERE

def box(a,b):return cq.Solid.makeBox(*[y-x for x,y in zip(a,b)],cq.Vector(*a))
def cyl(x,z,a,b,r):return cq.Solid.makeCylinder(r,b-a,cq.Vector(x,a,z),cq.Vector(0,1,0))

def build():
    meta=p.read(ROOT/'cad/wrist_housing_guide_v1/selected2/generated/parts/scene.json')['parts'];kept=[z for z in meta if z['installed_part_candidate'] and z['name']!='wrist_oblique_guide_clip'];rows=c.load(kept)
    sh=box([45,-9,-6],[62,-6,6]).fuse(box([24,-12,-6],[54,-9,4.]))
    sh=sh.fuse(box([54,-9,-58],[62,-6,6])).fuse(box([45,-9,-58],[62,-6,-46])).fuse(box([54,-6,-58],[56,0,-46]))
    for z in [-3.,2.]:sh=sh.fuse(cyl(26,z,-14,-4,2.5))
    for z in [-3.,2.]:sh=sh.cut(cyl(26,z,-18,4,1.6))
    for z in [-55.,-49.]:sh=sh.cut(cyl(59,z,-14,6,1.1))
    sh=sh.clean();assert sh.isValid() and len(sh.Solids())==1
    new=[dict(name='wrist_inboard_guide_clip',shape=sh,body='palm',attachment_body='Palm_wrist_saddle',installed_part_candidate=True,color=[.1,.68,.65])]
    for i,z in enumerate([-3.,2.]):
        for kind,shape in [('housing',cyl(26,z,-18,18,1.5).cut(cyl(26,z,-18.01,18.01,.65))),('cord',cyl(26,z,-18,18,.3))]:
            new.append(dict(name='inboard_local_'+kind+'_'+str(i),shape=shape,kind=kind,physical_line=i,body='palm',installed_part_candidate=False,color=[.66,.27,.83] if kind=='housing' else [.95,.43,.12]))
    return new,rows,dict(replaces='wrist_oblique_guide_clip',new_hardware_parts=1,retained_hardware_parts=5,local_housing_references=2,local_cord_references=2,guide_axis_world=[0,1,0],guide_x_mm=26.,centers_z_mm=[-3.,2.],guide_centerline_y_mm=[-14.,-4.],local_reference_y_mm=[-18.,18.],local_length_mm=36.,bore_OD_mm=[3.2,5.],housing_OD_ID_mm=[3.,1.3],cord_OD_mm=.6,clamp_contact_fastener_stack_unchanged=True,new_clamp_or_original_wrist_cut=False,hand_motor_rack_electrical_changed=False,full_motor_thumb_connected=False,whole_hand_drive_approved=False)

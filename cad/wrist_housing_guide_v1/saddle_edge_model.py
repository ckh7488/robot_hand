"""Keep the oblique guide, grip the saddle below the actual thumb root."""
import wrist_guide_model as prior
m,g,c,p,np,cq,ROOT,HERE=prior.m,prior.g,prior.c,prior.p,prior.np,prior.cq,prior.ROOT,prior.HERE
box,cylinder,point=prior.box,prior.cylinder,prior.point

def build():
    rows,design=prior.build();selected=[]
    back=box([45,-9,-6],[62,-6,6]).fuse(box([25,-12,-6],[54,-9,4.]))
    back=back.fuse(box([54,-9,-54],[62,-6,6])).fuse(box([45,-9,-54],[62,-6,-42])).fuse(box([54,-6,-54],[56,0,-42]))
    for z in [-3.,2.]:back=back.fuse(cylinder(point(-14,z),point(-4,z),2.5))
    for z in [-3.,2.]:back=back.cut(cylinder(point(-18,z),point(4,z),1.6))
    cap=box([45,0,-54],[62,3,-42])
    for z in [-51.,-45.]:
        bore=cylinder([59,-14,z],[59,6,z],1.1);back=back.cut(bore);cap=cap.cut(bore)
    for z in rows:
        if z['name']=='wrist_oblique_guide_clip':z=dict(z,shape=back.clean())
        elif z['name']=='wrist_edge_clip_front_cap':z=dict(z,shape=cap.clean())
        elif z['name'].startswith('wrist_edge_clip_bolt_') or z['name'].startswith('wrist_edge_clip_nut_'):z=dict(z,shape=z['shape'].translate((0,0,-48)))
        assert z['shape'].isValid() and len(z['shape'].Solids())==1,z['name']
        selected.append(z)
    design.update(attachment='Same original saddle wall six-mm thickness and outer edge x54;clamp contacts are z=-54..-42 below original thumb fixed housing minimumz=-40.5.Integral rear spine connects existing oblique sleeves to this clamp.No hand cut or installation change.',clamp_center_shift_z_mm=-48.,bolt_axes_world_mm=[[59,-51],[59,-45]],backbone_x_y_z_mm=[[54,-9,-54],[62,-6,6]],original_front_cap_collision_mm3=192.59487070610356,whole_motor_to_thumb_connected=False)
    return selected,design

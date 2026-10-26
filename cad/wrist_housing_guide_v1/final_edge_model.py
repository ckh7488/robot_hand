"""Put the front contact entirely below the current palm floor."""
import saddle_edge_model as prior
m,g,c,p,np,cq,ROOT,HERE=prior.m,prior.g,prior.c,prior.p,prior.np,prior.cq,prior.ROOT,prior.HERE
box,cylinder,point=prior.box,prior.cylinder,prior.point

def build():
    rows,design=prior.build();selected=[]
    back=box([45,-9,-6],[62,-6,6]).fuse(box([25,-12,-6],[54,-9,4.]))
    back=back.fuse(box([54,-9,-58],[62,-6,6])).fuse(box([45,-9,-58],[62,-6,-46])).fuse(box([54,-6,-58],[56,0,-46]))
    for z in [-3.,2.]:back=back.fuse(cylinder(point(-14,z),point(-4,z),2.5))
    for z in [-3.,2.]:back=back.cut(cylinder(point(-18,z),point(4,z),1.6))
    cap=box([45,0,-58],[62,3,-46])
    for z in [-55.,-49.]:
        bore=cylinder([59,-14,z],[59,6,z],1.1);back=back.cut(bore);cap=cap.cut(bore)
    for z in rows:
        if z['name']=='wrist_oblique_guide_clip':z=dict(z,shape=back.clean())
        elif z['name']=='wrist_edge_clip_front_cap':z=dict(z,shape=cap.clean())
        elif z['name'].startswith('wrist_edge_clip_bolt_') or z['name'].startswith('wrist_edge_clip_nut_'):z=dict(z,shape=z['shape'].translate((0,0,-4)))
        assert z['shape'].isValid() and len(z['shape'].Solids())==1,z['name']
        selected.append(z)
    design.update(attachment='Same original saddle six-mm wall and outer edge x54.Frontcap z=-58..-46 lies below currentPalm_tapered_chassis minz=-44.0003364523.Rigid rear spine connects guide to clamp.No hand/wrist material removed.',clamp_center_shift_z_mm=-52.,bolt_axes_world_mm=[[59,-55],[59,-49]],backbone_x_y_z_mm=[[54,-9,-58],[62,-6,6]],intermediate_lower_cap_palm_overlap_mm3=.6159933373684516)
    return selected,design

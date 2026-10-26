"""Replace the interfering broad plate by a rib following the real channel."""
from offset_model import *
HERE=Path(__file__).resolve().parent/'selected'
def build():
    parts,guide,design=__import__('offset_model').build();by={z['name']:z for z in parts};Q=g.original.layout()[1]['socket_transform']
    cap=old.zbox([-5,-4,0],[9,4,4],1)
    cap=cap.fuse(old.bx([-2,-13,2],[2,-3,4])).fuse(pathshape(1.5).translate((0,0,3))).fuse(old.bx([7.5,-36,0],[20.5,-26,4]))
    cap=cap.fuse(pathshape(.8)).fuse(old.cy(.9,-12,-10))
    for x,u in [(0,-12),(7,-19),(14,-26)]:cap=cap.fuse(old.zcyl(.65,x,u,0,3,1))
    tools=[old.cy(.65,-12,4.01),old.cy(1.6,0,2),old.bx([1.1,0,-.6],[2.7,2,.6]),old.zcyl(1.1,6,0,-6.01,5.21,1),pathshape(.4),old.cy(2.05,-36.01,-26).translate((14,0,0)),old.cy(2.7,-33,-31).translate((14,0,0))]
    # The existing liner is laid radially into this open half-socket.
    tools.append(old.bx([-1.,-12.,-1.],[1.,-10.,0.]))
    for x in [9.5,18.5]:tools.append(old.zcyl(1.1,x,-29.5,-6.01,5.21,1))
    for t in tools:cap=cap.cut(t)
    cap=g.original.require(cap.clean());local=g.original.transform(cap,Q);by['distal_housing_keeper_1']=dict(by['distal_housing_keeper_1'],shape=g.original.transform(local,g.original.WORLD),local_shape=local,role='integral_offset_channel_on_three_posts_and_curved_rib_with_open_radial_liner_seat')
    design=dict(design,support_plate_socket_bounds_mm=None,curved_support_rib_profile_radius_mm=1.5,curved_support_rib_z_mm=3.,liner_coupling_radial_open=True,downstream_keeper_bore_corrected_to_u_plus4p01=True,prototype_broad_plate_replaced=True)
    design['source_sha256']=dict(design['source_sha256']);design['source_sha256']['cad/thumb_sheath_terminal_v2/selected_model.py']=drive.sha(Path(__file__))
    return [by[z['name']] for z in parts],guide,design

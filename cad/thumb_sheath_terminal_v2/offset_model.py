"""Supported upstream offset, preserving the existing distal liner centerline."""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/thumb_sheath_terminal_v1'))
import distal_model as old
g,c,p,np,cq,drive=old.g,old.c,old.p,old.np,old.cq,old.drive
SHIFT=np.array([14.,-17.,0.]);R=7.
def paths():
    a=np.array([0.,-12.,0.]);b=np.array([7.,-19.,0.]);z=np.array([14.,-26.,0.]);d=R/math.sqrt(2)
    return [g.g.arc(a,[7-d,-12-d,0],b,R*math.pi/2,'upper_supported_offset_1'),g.g.arc(b,[7+d,-26+d,0],z,R*math.pi/2,'upper_supported_offset_2')]
def pathshape(radius):
    ss=[g.original.piece_shape(seg,radius) for seg in paths()]
    return ss[0].fuse(ss[1])
def build():
    allparts,guide,design=old.build();selected={z['name']:z for z in allparts};Q=g.original.layout()[1]['socket_transform']
    def add(name,s,role):
        s=g.original.require(s.clean());local=g.original.transform(s,Q);selected[name]=dict(name=name,shape=g.original.transform(local,g.original.WORLD),local_shape=local,body='thumb_mcp',role=role,color=selected[name]['color'])
    cap=old.zbox([-5,-4,0],[9,4,4],1)
    cap=cap.fuse(old.bx([-2,-36,2],[20.5,-3,4])).fuse(old.bx([7.5,-36,0],[20.5,-26,4]))
    cap=cap.fuse(pathshape(.8)).fuse(old.cy(.9,-12,-10))
    for x,u in [(0,-12),(7,-19),(14,-26)]:cap=cap.fuse(old.zcyl(.65,x,u,0,3,1))
    tools=[old.cy(.65,-12,-3.99),old.cy(1.6,0,2),old.bx([1.1,0,-.6],[2.7,2,.6]),old.zcyl(1.1,6,0,-6.01,5.21,1),pathshape(.4)]
    # Terminal port/ferrule keep their nominal diameters and flange capture.
    tools += [old.cy(2.05,-36.01,-26,).translate(tuple([14,0,0])),old.cy(2.7,-33,-31).translate((14,0,0))]
    for x in [9.5,18.5]:tools.append(old.zcyl(1.1,x,-29.5,-6.01,5.21,1))
    for tool in tools:cap=cap.cut(tool)
    add('distal_housing_keeper_1',cap,'integral_upper_keeper_offset_channel_support_plate_three_posts_and_ferrule_capture')
    shoe=old.bx([7.5,-36,-4],[20.5,-26,0])
    for tool in tools:shoe=shoe.cut(tool)
    add('distal_housing_shoe_1',shoe,'radially_removable_lower_half_of_offset_ferrule_seat')
    for name in ['distal_flanged_ferrule_1','distal_shoe_M2x10_1_0','distal_shoe_M2x10_1_1','distal_shoe_M2_nut_1_0','distal_shoe_M2_nut_1_1']:
        local=g.original.transform(selected[name]['local_shape'],np.linalg.inv(Q)).translate(tuple(SHIFT));add(name,local,'same_nominal_ferrule_or_fastener_shifted_with_upper_port')
    design=dict(design,upper_port_shift_socket_mm=SHIFT.tolist(),upper_offset_radius_candidate_mm=R,upper_integral_channel_OD_ID_mm=[1.6,.8],upper_liner_spigot_socket_OD_ID_length_mm=[1.8,1.3,2.],support_plate_socket_bounds_mm=[[-2,-36,2],[20.5,-3,4]],physical_support_posts_socket_mm=[[0,-12],[7,-19],[14,-26]],guide_centerline_downstream_unchanged=True,upper_channel_and_plate_integral=True,compression_reaction_at_captured_ferrule_flange=True,whole_transmission_complete=False)
    design['source_sha256']=dict(design['source_sha256']);design['source_sha256']['cad/thumb_sheath_terminal_v2/offset_model.py']=drive.sha(Path(__file__))
    return [selected[z['name']] for z in allparts],guide,design
def witnesses():
    rows=old.witnesses();Q=g.original.layout()[1]['socket_transform'];T=np.eye(4);T[:3,3]=SHIFT
    for z in rows:
        if z['name'].endswith('_1'):
            z['local_shape']=g.original.transform(z['local_shape'],Q@T@np.linalg.inv(Q));z['shape']=g.original.transform(z['local_shape'],g.original.WORLD)
    return rows
def extension_cords():
    Q=g.original.layout()[1]['socket_transform'];rows=[]
    for j,s in enumerate(paths()):
        sh=g.original.transform(g.original.piece_shape(s,.3),g.original.WORLD@Q)
        rows.append(dict(name='upper_offset_cord_'+str(j),shape=sh,body='thumb_mcp',cord=1,role='two_CAD_arcs_of_same_existing_upper_cord',color=[.97,.57,.13]))
    return rows

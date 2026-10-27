"""Actual upstream bend support attached to the existing upper clamp jaw."""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/paired_rear_guide_v1'))
import check_pair as prior
a,m,g,c,p,np,cq=prior.a,prior.m,prior.g,prior.c,prior.p,prior.np,prior.cq
RADIUS=20.;LEAD=12.
REPLACED='distal_housing_clamp_1_jaw_1'

def frame():
    design=p.read(ROOT/'cad/sheath_retention_v1/selected/reviews/check.json')['design'];return np.asarray(next(z for z in design['ports'] if z['tag']=='distal_housing_clamp_1')['Q'])

def edges(include_terminal=0.):
    A=[RADIUS+LEAD,-11-RADIUS,0];B=[RADIUS,-11-RADIUS,0];D=[0,-11,0];M=[RADIUS-RADIUS/math.sqrt(2),-11-RADIUS/math.sqrt(2),0]
    result=[cq.Edge.makeLine(cq.Vector(*A),cq.Vector(*B)),cq.Edge.makeThreePointArc(cq.Vector(*B),cq.Vector(*M),cq.Vector(*D))]
    if include_terminal:result.append(cq.Edge.makeLine(cq.Vector(*D),cq.Vector(0,-11+include_terminal,0)))
    return result

def placed(sh):return g.original.transform(sh,frame())

def build():
    scene=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json');source={};base=c.load([z for z in scene['parts'] if z['name']==REPLACED],source)[0];guide=a.sweep(edges(),2.5).cut(a.sweep(edges(),1.6)).clean();body=base['shape'].fuse(placed(guide)).clean();assert body.isValid() and len(body.Solids())==1
    removed=base['shape'].cut(body,tol=1e-6);lost=removed.Volume() if removed.Solids() else 0.;assert lost<1e-8
    new=[dict(name='upper_clamp_jaw_with_entry_elbow',shape=body,body='thumb_mcp',physical_line=1,installed_part_candidate=True,color=[.1,.68,.65])]
    for kind,terminal in [('housing',25.),('cord',26.)]:
        es=edges(terminal);sh=a.sweep(es,1.5).cut(a.sweep(es,.65)).clean() if kind=='housing' else a.sweep(es,.3);sh=placed(sh);assert sh.isValid() and len(sh.Solids())==1
        new.append(dict(name='upper_entry_'+kind+'_reference',shape=sh,body='thumb_mcp',kind=kind,physical_line=1,installed_part_candidate=False,length_mm=sum(z.Length() for z in es),color=[.63,.25,.8] if kind=='housing' else [.97,.45,.12]))
    Q=frame();point=lambda xyz:(Q@np.r_[xyz,1.])[:3].tolist();design=dict(replaced_hand_part=REPLACED,hand_rows_with_replacement=397,new_hardware_rows=1,reference_rows=2,old_jaw_material_removed_mm3=lost,valid_connected_one_solid=True,bend_radius_mm=RADIUS,straight_entry_mm=LEAD,guide_OD_bore_mm=[5.,3.2],housing_OD_ID_mm=[3.,1.3],cord_OD_mm=.6,entry_local_mm=[RADIUS+LEAD,-11-RADIUS,0.],entry_world_neutral_mm=point([RADIUS+LEAD,-11-RADIUS,0.]),entry_axis_world_neutral=(-Q[:3,0]).tolist(),bend_end_local_mm=[0.,-11.,0.],bend_end_world_neutral_mm=point([0.,-11.,0.]),terminal_axis_world_neutral=Q[:3,1].tolist(),terminal_seat_world_neutral_mm=point([0,14,0]),support_body='thumb_mcp via captured upper clamp jaw/ferrule/bolts and original upper keeper',same_jaw_grip_collar_fastener_geometry_preserved=True,full_flexible_link_connected=False,preload_friction_strength_manufacturing_exterior_verified=False,source_sha256=source)
    return new,design

def feed_window(tip,length=100.):
    """A prescribed constant-length leading housing fed into the actual elbow."""
    es=edges(25.);ls=[z.Length() for z in es];lo,hi=tip-length,tip;pieces=[];A=np.array([RADIUS+LEAD,-11-RADIUS,0.]);axis=np.array([-1.,0.,0.]);start=0.
    if lo<0:pieces.append(cq.Edge.makeLine(cq.Vector(*(A+axis*lo)),cq.Vector(*(A+axis*min(hi,0.)))))
    for ed,L in zip(es,ls):
        left,right=max(0.,lo-start),min(L,hi-start)
        if right-left>1e-8:pieces.append(ed.trim(ed.paramAt(left/L),ed.paramAt(right/L)))
        start+=L
    assert hi<=sum(ls)+1e-7 and pieces;sh=a.sweep(pieces,1.5).cut(a.sweep(pieces,.65)).clean();assert sh.isValid() and len(sh.Solids())==1 and abs(sum(z.Length() for z in pieces)-length)<1e-5
    return dict(name='upper_housing_100mm_leading_reference',shape=placed(sh),body='thumb_mcp',installed_part_candidate=False,length_mm=length,tip_material_coordinate_mm=tip)

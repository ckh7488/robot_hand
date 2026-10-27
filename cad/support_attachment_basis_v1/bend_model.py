"""Two actual motor-jaw bend guides; long flexible links remain unconnected."""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/material_length_motion_v1'))
import check_motion as k
a,m,g,c,p,np,cq=k.a,k.m,k.g,k.c,k.p,k.np,k.cq
R=40.;LEAD=12.

def frame(i):
    ports=p.read(ROOT/'cad/sheath_retention_v1/selected/reviews/check.json')['design']['ports']
    return np.asarray(next(z for z in ports if z['tag']=='motor_housing_clamp_'+str(i))['Q'])

def outward_edges():
    A=cq.Vector(0,-12,0);B=cq.Vector(0,-12-LEAD,0);D=cq.Vector(0,-12-LEAD-R,-R)
    M=cq.Vector(0,-12-LEAD-R/math.sqrt(2),-R+R/math.sqrt(2))
    return [cq.Edge.makeLine(A,B),cq.Edge.makeThreePointArc(B,M,D)]

def inward_edges():
    A=cq.Vector(0,-12,0);B=cq.Vector(0,-12-LEAD,0);D=cq.Vector(0,-12-LEAD-R,-R)
    M=cq.Vector(0,-12-LEAD-R/math.sqrt(2),-R+R/math.sqrt(2))
    return [cq.Edge.makeThreePointArc(D,M,B),cq.Edge.makeLine(B,A),cq.Edge.makeLine(A,cq.Vector(0,5,0))]

def sweep(es,radius):return a.a.prior.a.sweep(es,radius)

def build(pack):
    rows=[];design=[];source={}
    for i in [0,1]:
        old=next(z for z in pack if z['name']=='motor_housing_clamp_'+str(i)+'_jaw_-1');Q=frame(i)
        tube=sweep(outward_edges(),2.5).cut(sweep(outward_edges(),1.6)).clean();body=old['shape'].fuse(g.original.transform(tube,Q)).clean()
        assert body.isValid() and len(body.Solids())==1,(i,body.isValid(),len(body.Solids()))
        removed=old['shape'].cut(body,tol=1e-6);lost=removed.Volume() if removed.Solids() else 0.;assert lost<1e-8
        rows.append(dict(name='motor_'+str(i)+'_jaw_with_fixed_bend',shape=body,body='forearm',physical_line=i,installed_part_candidate=True,replaced_part=old['name'],color=[.1,.68,.65]))
        es=inward_edges()
        for kind in ['housing','cord']:
            sh=sweep(es,1.5).cut(sweep(es,.65)).clean() if kind=='housing' else sweep(es,.3);sh=g.original.transform(sh,Q);assert sh.isValid() and len(sh.Solids())==1
            rows.append(dict(name='motor_'+str(i)+'_guided_'+kind+'_reference',shape=sh,body='forearm',physical_line=i,kind=kind,installed_part_candidate=False,length_mm=sum(ed.Length() for ed in es),color=[.63,.25,.8] if kind=='housing' else [.97,.45,.12]))
        design.append(dict(line=i,replaced=old['name'],removed_old_material_mm3=lost,valid_connected_one_solid=True,Q=Q,free_port_world_mm=(Q@np.array([0,-12-LEAD-R,-R,1]))[:3],outward_tangent_world=Q[:3,:3]@np.array([0,0,-1]),fixed_bend_radius_mm=R,guide_straight_mm=LEAD,guide_centerline_length_mm=LEAD+math.pi*R/2,seat_to_free_port_length_mm=sum(ed.Length() for ed in es),nominal_reaction_path='housing -> bore -> integral existing lower jaw -> existing two M2/opposite jaw -> captured barrel-adjuster head -> existing terminal support/M3 -> original forearm motor mount',same_old_jaw_and_fastener_seats_preserved=True,actual_load_friction_stiffness_or_tools_verified=False))
    return rows,design

def feed(i,tip,length=100.):
    es=inward_edges();lo=tip-length;hi=tip;parts=[];E=np.array([0.,-12-LEAD-R,-R]);axis=np.array([0.,0.,1.]);offset=0.
    if lo<0:parts.append(cq.Edge.makeLine(cq.Vector(*(E+lo*axis)),cq.Vector(*(E+min(hi,0)*axis))))
    for ed in es:
        ll=ed.Length();left,right=max(0.,lo-offset),min(ll,hi-offset)
        if right-left>1e-8:parts.append(ed.trim(ed.paramAt(left/ll),ed.paramAt(right/ll)))
        offset+=ll
    assert hi<=offset+1e-6 and abs(sum(z.Length() for z in parts)-length)<1e-5
    sh=sweep(parts,1.5).cut(sweep(parts,.65)).clean();sh=g.original.transform(sh,frame(i));assert sh.isValid() and len(sh.Solids())==1
    return dict(name='motor_'+str(i)+'_100mm_feed_tip_'+str(tip),shape=sh,body='forearm',physical_line=i,installed_part_candidate=False,length_mm=length,tip_mm=tip)

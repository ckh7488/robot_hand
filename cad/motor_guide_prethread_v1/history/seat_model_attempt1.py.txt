"""Carry actual loose closed guides while bending unchanged-length housing tails.

The stored neutral arcs supply one assembly target, not a new biarc pose solve.
"""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/whole_line_threading_v1'))
import thread_model as t
l,a,m,g,c,p,np,cq,b=t.l,t.a,t.m,t.g,t.c,t.p,t.np,t.cq,t.l.b


def rotation(axis,angle):
    axis=np.asarray(axis,float);axis/=np.linalg.norm(axis);x,y,z=axis;K=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)


def reverse_edge(ed):
    return cq.Edge.makeLine(ed.endPoint(),ed.startPoint()) if ed.geomType()=='LINE' else cq.Edge.makeThreePointArc(ed.endPoint(),ed.positionAt(.5),ed.startPoint())


def definitions(case,i):
    downstream=l.middle_edges(case,i,'housing')[5:];rear=np.array(downstream[0].startPoint().toTuple());axis=-np.array(downstream[0].tangentAt(0).toTuple());Q=b.frame(i);fixed=[g.original.transform(ed,Q) for ed in b.inward_edges()];fixed_length=sum(z.Length() for z in fixed);tail=case['lines'][i]['housing_length_mm']-sum(z.Length() for z in downstream)
    return downstream,rear,axis,fixed,fixed_length,tail


def scaled_free(case,i,scale):
    downstream,rear,axis,fixed,fixed_length,tail=definitions(case,i);P=rear.copy();M=np.eye(3);result=[];radii=[];lengths=[]
    for raw in reversed(case['lines'][i]['up']['arcs']):
        A=np.asarray(raw['end']);T=-np.asarray(raw['t1']);N=(np.asarray(raw['center'])-A)/raw['radius_mm'];axis_b=np.cross(T,N);angle=raw['angle_rad'];length=raw['length_mm'];theta=scale*angle;tt=M@T;nn=M@N
        if scale==0:Z=P+length*tt;ed=cq.Edge.makeLine(cq.Vector(*P),cq.Vector(*Z))
        else:
            radius=length/theta;Z=P+radius*(math.sin(theta)*tt+(1-math.cos(theta))*nn);mid=P+radius*(math.sin(theta/2)*tt+(1-math.cos(theta/2))*nn);ed=cq.Edge.makeThreePointArc(cq.Vector(*P),cq.Vector(*mid),cq.Vector(*Z));radii.append(radius)
        assert abs(ed.Length()-length)<1e-5
        result.append(ed);lengths.append(length);P=Z;M=M@rotation(axis_b,(scale-1)*angle)
    if scale==0:
        assert np.linalg.norm(P-(rear+axis*sum(lengths)))<1e-7 and np.linalg.norm(M-np.eye(3))<1e-7
        P=rear+axis*sum(lengths);M=np.eye(3);result=[cq.Edge.makeLine(cq.Vector(*rear),cq.Vector(*P))]
    return result,P,M,radii


def edges(case,i,phase,value):
    downstream,rear,axis,fixed,fixed_length,tail=definitions(case,i);E=np.asarray(fixed[0].startPoint().toTuple());T=np.eye(4)
    if phase=='feed':
        assert 0<=value<=fixed_length+1e-6;entry=rear+axis*(tail-value);T[:3,3]=entry-E;up=[cq.Edge.makeLine(cq.Vector(*rear),cq.Vector(*entry))];offset=0.
        for ed in fixed:
            size=ed.Length();hi=min(size,value-offset)
            if hi>1e-8:up.append(g.original.transform(ed.trim(ed.paramAt(0),ed.paramAt(hi/size)),T))
            offset+=size
        radii=[b.R] if value>0 else []
    else:
        assert phase=='bend' and 0<=value<=1;up,entry,M,radii=scaled_free(case,i,value);T[:3,:3]=M;T[:3,3]=entry-M@E;up += [g.original.transform(ed,T) for ed in fixed];radii.append(b.R)
    ordered=[reverse_edge(ed) for ed in reversed(up)]+downstream;length=sum(z.Length() for z in ordered);assert abs(length-case['lines'][i]['housing_length_mm'])<1e-5
    gaps=[(x.endPoint()-y.startPoint()).Length for x,y in zip(ordered[:-1],ordered[1:])];assert max(gaps+[0.])<1e-5
    info=dict(line=i,phase=phase,value=value,guide_transform=T,material_length_mm=case['lines'][i]['housing_length_mm'],actual_length_mm=length,PAIR_rear_material_s_mm=tail,guide_entry_mm=entry,housing_free_end_mm=ordered[0].startPoint().toTuple(),maximum_join_gap_mm=max(gaps+[0.]),minimum_prescribed_radius_mm=min(radii) if radii else None,material_points=[dict(s_mm=float(s),world_mm=t.point_at(ordered,float(s))) for s in np.linspace(0.,length,9)])
    return ordered,T,info


def build(case,i,phase,value,hardware):
    es,T,info=edges(case,i,phase,value);rows=[]
    for j,ed in enumerate(es):
        sh=m.swept(ed,1.5).cut(m.swept(ed,.65)).clean();assert sh.isValid() and len(sh.Solids())==1
        rows.append(dict(name='seat_housing_'+str(i)+'_'+str(j),shape=sh,physical_line=i,kind='housing',segment=j,length_mm=ed.Length(),body='palm',installed_part_candidate=False,color=[.63,.25,.8]))
    mobile=dict(hardware,shape=g.original.transform(hardware['shape'],T),assembly_transform=T,current_world_pose_only=True)
    return rows,mobile,info

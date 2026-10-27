"""Thread whole cord from the real motor groove through actual seated channels.

Open spool/joint wraps and the uninserted tail are held manually. A declared tail
endpoint is not a knot, clamping force, or proof of elastic equilibrium.
"""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/whole_line_threading_v1'))
import thread_model as t
w,l,a,m,g,c,p,np,cq=t.w,t.l,t.a,t.m,t.g,t.c,t.p,t.np,t.cq


def specification(case,i):
    motor=w.motor_edges(i);anchor=motor[0][1];path=[ed for _,ed in motor[1:]]+l.middle_edges(case,i,'cord')+[ed for _,ed in w.distal_edges(case,i)]
    length=p.read(ROOT/'cad/biarc_supported_link_v1/reviews/check.json')['chains'][i]['length_mm']
    labels=[name for name,_ in motor[1:]]+['middle_'+str(j) for j in range(len(l.middle_edges(case,i,'cord')))]+[name for name,_ in w.distal_edges(case,i)]
    assert abs(sum(z.Length() for z in path)+anchor.Length()-length)<1e-5
    assert max((x.endPoint()-y.startPoint()).Length for x,y in zip(path[:-1],path[1:]))<1e-5
    return path,labels,length,anchor


def pieces(case,i,tip,tail_fraction=None):
    path,labels,length,anchor=specification(case,i);total=sum(z.Length() for z in path)
    if tail_fraction is None:
        edges=t.window_edges(path,tip,length)
    else:
        assert abs(tip-total)<1e-5 and 0<=tail_fraction<=1
        A=np.asarray(path[0].startPoint().toTuple());v0=-np.asarray(path[0].tangentAt(0).toTuple());v1=np.asarray(anchor.startPoint().toTuple())-A;v1/=np.linalg.norm(v1)
        angle=math.acos(float(np.clip(v0@v1,-1,1)));axis=np.cross(v0,v1);axis/=np.linalg.norm(axis)
        v=v0*math.cos(angle*tail_fraction)+np.cross(axis,v0)*math.sin(angle*tail_fraction)+axis*(axis@v0)*(1-math.cos(angle*tail_fraction))
        edges=[cq.Edge.makeLine(cq.Vector(*(A+anchor.Length()*v)),cq.Vector(*A))]+path
    gaps=[(x.endPoint()-y.startPoint()).Length for x,y in zip(edges[:-1],edges[1:])]
    info=dict(line=i,tip_coordinate_mm=tip,tail_fraction=tail_fraction,material_length_mm=length,actual_length_mm=sum(z.Length() for z in edges),
              path_length_mm=total,anchor_tail_length_mm=anchor.Length(),start_mm=edges[0].startPoint().toTuple(),end_mm=edges[-1].endPoint().toTuple(),
              maximum_join_gap_mm=max(gaps+[0.]),motor_tail_is_declared_endpoint_not_actual_knot=tail_fraction is not None,
              material_points=[dict(material_s_mm=float(v),world_mm=t.point_at(edges,float(v))) for v in np.linspace(0.,length,9)])
    assert abs(info['actual_length_mm']-length)<1e-5 and info['maximum_join_gap_mm']<1e-5
    return edges,info


def build(case,i,tip,tail_fraction=None):
    edges,info=pieces(case,i,tip,tail_fraction);rows=[]
    for j,ed in enumerate(edges):
        sh=m.swept(ed,.3);assert sh.isValid() and len(sh.Solids())==1
        rows.append(dict(name='motorfeed_cord_'+str(i)+'_'+str(j),shape=sh,kind='cord',physical_line=i,segment=j,length_mm=ed.Length(),
                         body='palm',installed_part_candidate=False,color=[.97,.45,.12],endpoint_start_mm=ed.startPoint().toTuple(),endpoint_end_mm=ed.endPoint().toTuple()))
    return rows,info


def samples(case,i):
    path,_,_,_=specification(case,i)
    return [0.]+np.cumsum([z.Length() for z in path]).tolist()

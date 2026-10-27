"""Thread the actual motor anchor bore before laying its open spool wrap."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import cord_model as first
t,w,l,a,m,g,c,p,np,cq,ROOT,HERE=first.t,first.w,first.l,first.a,first.m,first.g,first.c,first.p,first.np,first.cq,first.ROOT,first.HERE


def specification(case,i):
    labelled=w.ordered(case,i);path=[ed for _,ed in labelled];length=p.read(ROOT/'cad/biarc_supported_link_v1/reviews/check.json')['chains'][i]['length_mm']
    assert abs(sum(z.Length() for z in path)-length)<1e-5
    assert max((x.endPoint()-y.startPoint()).Length for x,y in zip(path[:-1],path[1:]))<1e-5
    return path,[name for name,_ in labelled],length


def pieces(case,i,tip):
    path,labels,length=specification(case,i);lo,hi=tip-length,tip;result=[];roles=[];A=path[0].startPoint();axis=path[0].tangentAt(0);offset=0.
    if lo<0:
        result.append(cq.Edge.makeLine(A+axis*lo,A+axis*min(hi,0.)));roles.append((-1,'incoming_free_tail'))
    for j,ed in enumerate(path):
        size=ed.Length();left,right=max(0.,lo-offset),min(size,hi-offset)
        if right-left>1e-8:
            result.append(ed.trim(ed.paramAt(left/size),ed.paramAt(right/size)));roles.append((j,labels[j]))
        offset+=size
    assert abs(sum(z.Length() for z in result)-length)<1e-5
    gaps=[(x.endPoint()-y.startPoint()).Length for x,y in zip(result[:-1],result[1:])]
    info=dict(line=i,tip_coordinate_mm=tip,material_length_mm=length,actual_length_mm=sum(z.Length() for z in result),
              path_length_mm=offset,tail_outside_path_mm=max(0.,length-tip),start_mm=result[0].startPoint().toTuple(),end_mm=result[-1].endPoint().toTuple(),
              maximum_join_gap_mm=max(gaps+[0.]),piece_roles=roles,
              material_points=[dict(material_s_mm=float(v),world_mm=t.point_at(path,tip-length+float(v))) for v in np.linspace(0.,length,9)])
    assert info['maximum_join_gap_mm']<1e-5
    return result,roles,info


def build(case,i,tip):
    edges,roles,info=pieces(case,i,tip);rows=[]
    for j,(ed,(path_segment,role)) in enumerate(zip(edges,roles)):
        sh=m.swept(ed,.3);assert sh.isValid() and len(sh.Solids())==1
        rows.append(dict(name='anchorfeed_cord_'+str(i)+'_'+str(j),shape=sh,kind='cord',physical_line=i,segment=j,path_segment=path_segment,role=role,
                         length_mm=ed.Length(),body='palm',installed_part_candidate=False,color=[.97,.45,.12],endpoint_start_mm=ed.startPoint().toTuple(),endpoint_end_mm=ed.endPoint().toTuple()))
    return rows,info


def samples(case,i):
    path,_,_=specification(case,i)
    return [0.]+np.cumsum([z.Length() for z in path]).tolist()

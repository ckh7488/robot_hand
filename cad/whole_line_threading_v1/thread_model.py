"""Full material windows through real channels, with explicitly held straight tails.

This assembly branch ends with free motor tails, not installed motor endpoints.
"""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/biarc_supported_link_v1'))
import details as prior
w,l,a,m,g,c,p,np,cq=prior.w,prior.l,prior.a,prior.m,prior.g,prior.c,prior.p,prior.np,prior.w.cq


def neutral():return p.read(ROOT/'cad/biarc_supported_link_v1/reviews/analysis.json')['cases'][0]


def specification(case,i,kind):
    housing=l.middle_edges(case,i,'housing')[5:]
    length=case['lines'][i]['housing_length_mm'];downstream=sum(ed.Length() for ed in housing)
    rear=housing[0].startPoint();axis=housing[0].tangentAt(0)
    if kind=='housing':return housing,length
    bare_tail=cq.Edge.makeLine(rear+axis*(downstream-length),rear)
    cord=l.middle_edges(case,i,'cord')[5:]
    edges=[bare_tail]+cord+[ed for _,ed in w.distal_edges(case,i)]
    full=p.read(ROOT/'cad/biarc_supported_link_v1/reviews/check.json')['chains'][i]['length_mm']
    assert full>sum(ed.Length() for ed in edges)
    return edges,full


def window_edges(edges,tip,length):
    lo,hi=tip-length,tip;parts=[];A=edges[0].startPoint();axis=edges[0].tangentAt(0);offset=0.
    if lo<0:parts.append(cq.Edge.makeLine(A+axis*lo,A+axis*min(hi,0.)))
    for ed in edges:
        size=ed.Length();left,right=max(0.,lo-offset),min(size,hi-offset)
        if right-left>1e-8:parts.append(ed.trim(ed.paramAt(left/size),ed.paramAt(right/size)))
        offset+=size
    assert 0<=tip<=offset+1e-6 and abs(sum(z.Length() for z in parts)-length)<1e-5
    return parts


def point_at(edges,coordinate):
    if coordinate<0:return (edges[0].startPoint()+edges[0].tangentAt(0)*coordinate).toTuple()
    for ed in edges:
        size=ed.Length()
        if coordinate<=size+1e-8:return ed.positionAt(min(1.,coordinate/size)).toTuple()
        coordinate-=size
    raise AssertionError(coordinate)


def state(case,i,kind,tip):
    edges,length=specification(case,i,kind);pieces=window_edges(edges,tip,length);rows=[]
    for j,ed in enumerate(pieces):
        sh=m.swept(ed,1.5).cut(m.swept(ed,.65)).clean() if kind=='housing' else m.swept(ed,.3)
        assert sh.isValid() and len(sh.Solids())==1,(kind,i,j)
        rows.append(dict(name='thread_'+kind+'_'+str(i)+'_'+str(j),shape=sh,kind=kind,physical_line=i,segment=j,length_mm=ed.Length(),installed_part_candidate=False,body='palm',color=[.63,.25,.8] if kind=='housing' else [.97,.45,.12]))
    gaps=[(z.endPoint()-t.startPoint()).Length for z,t in zip(pieces[:-1],pieces[1:])]
    info=dict(line=i,kind=kind,tip_coordinate_mm=tip,material_length_mm=length,actual_centerline_length_mm=sum(z.Length() for z in pieces),path_length_mm=sum(z.Length() for z in edges),tail_outside_path_mm=max(0.,length-tip),start_mm=pieces[0].startPoint().toTuple(),end_mm=pieces[-1].endPoint().toTuple(),maximum_piece_endpoint_gap_mm=max(gaps+[0.]),material_points=[dict(material_s_mm=float(s),world_mm=point_at(edges,tip-length+s)) for s in np.linspace(0.,length,9)])
    return rows,info


def samples(case,i,kind):
    edges,length=specification(case,i,kind);ends=np.cumsum([z.Length() for z in edges]).tolist()
    # Real channel/edge boundaries and the midpoint of each free edge.
    points=[0.]+ends
    if kind=='housing':
        cursor=0.
        for j,ed in enumerate(edges):
            if j in [4,5]:points.append(cursor+ed.Length()/2)
            cursor+=ed.Length()
    return sorted(set(round(float(x),11) for x in points))

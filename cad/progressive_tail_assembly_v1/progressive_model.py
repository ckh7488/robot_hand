"""Form each loose tail from the real PAIR mouth toward its motor end.

Already formed material uses the saved neutral curvature. The unformed remainder
stays straight and is held by the operator, not by an invented fixture.
"""
import sys, math
from pathlib import Path
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'cad/motor_guide_prethread_v1'))
import seat_model as s
t,a,m,g,c,p,np,cq,b = s.t,s.a,s.m,s.g,s.c,s.p,s.np,s.cq,s.b


def edges(case, i, formed_mm):
    downstream,rear,axis,fixed,fixed_length,tail = s.definitions(case,i)
    raw = [s.reverse_edge(t.l.edge(z)) for z in reversed(case['lines'][i]['up']['arcs'])]
    total = sum(z.Length() for z in raw)
    assert 0 <= formed_mm <= total+1e-7
    up=[]; remaining=formed_mm; P=rear.copy(); M=np.eye(3)
    for ed,z in zip(raw,reversed(case['lines'][i]['up']['arcs'])):
        use=min(ed.Length(),max(0.,remaining)); remaining-=use
        if use>1e-8:
            part=ed if abs(use-ed.Length())<1e-7 else ed.trim(ed.paramAt(0),ed.paramAt(use/ed.Length()))
            up.append(part); P=np.asarray(part.endPoint().toTuple())
            T=-np.asarray(z['t1']); N=(np.asarray(z['center'])-np.asarray(z['end']))/z['radius_mm']
            M=s.rotation(np.cross(T,N),use/z['radius_mm'])@M
    unformed=max(0.,total-formed_mm)
    if unformed>1e-8:
        tangent=M@axis; Z=P+unformed*tangent
        up.append(cq.Edge.makeLine(cq.Vector(*P),cq.Vector(*Z))); P=Z
    E=np.asarray(fixed[0].startPoint().toTuple()); T=np.eye(4); T[:3,:3]=M; T[:3,3]=P-M@E
    up += [g.original.transform(ed,T) for ed in fixed]
    ordered=[s.reverse_edge(ed) for ed in reversed(up)]+downstream
    length=sum(z.Length() for z in ordered)
    gaps=[(x.endPoint()-y.startPoint()).Length for x,y in zip(ordered[:-1],ordered[1:])]
    tangents=[(x.tangentAt(1)-y.tangentAt(0)).Length for x,y in zip(ordered[:-1],ordered[1:])]
    assert abs(length-case['lines'][i]['housing_length_mm'])<1e-5 and max(gaps+[0.])<1e-5 and max(tangents+[0.])<1e-6
    info=dict(line=i,formed_mm=formed_mm,free_material_length_mm=total,unformed_straight_length_mm=unformed,
              guide_transform=T,material_length_mm=case['lines'][i]['housing_length_mm'],actual_length_mm=length,
              PAIR_rear_material_s_mm=tail,guide_entry_mm=P,housing_free_end_mm=ordered[0].startPoint().toTuple(),
              maximum_join_gap_mm=max(gaps+[0.]),maximum_tangent_gap=max(tangents+[0.]),
              prescribed_free_radii_mm=[z['radius_mm'] for z in case['lines'][i]['up']['arcs']],
              material_points=[dict(s_mm=float(v),world_mm=t.point_at(ordered,float(v))) for v in np.linspace(0.,length,9)])
    return ordered,T,info


def build(case,i,formed_mm,hardware):
    es,T,info=edges(case,i,formed_mm);rows=[]
    for j,ed in enumerate(es):
        sh=m.swept(ed,1.5).cut(m.swept(ed,.65)).clean()
        assert sh.isValid() and len(sh.Solids())==1
        rows.append(dict(name='progressive_housing_'+str(i)+'_'+str(j),shape=sh,physical_line=i,kind='housing',segment=j,
                         length_mm=ed.Length(),body='palm',installed_part_candidate=False,color=[.63,.25,.8]))
    mobile=dict(hardware,shape=g.original.transform(hardware['shape'],T))
    return rows,mobile,info

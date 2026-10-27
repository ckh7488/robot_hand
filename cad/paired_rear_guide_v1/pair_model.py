"""A real connected pair of S-bend rear intakes, ordered like the motors."""
from survey_pair import a,m,g,c,p,np,cq,ROOT,HERE
import math
RADIUS=40.;OFFSET=10.;LEAD=40.
ANGLE=math.acos(1-OFFSET/(2*RADIUS));RUN=2*RADIUS*math.sin(ANGLE)
REAR_Y=-100.-RUN-LEAD

def definitions(i):
    z=[-3.,2.][i];sign=1 if i==0 else -1;A=np.array([100.-sign*OFFSET,REAR_Y,z]);B=A+[0,LEAD,0];J=B+[sign*RADIUS*(1-math.cos(ANGLE)),RADIUS*math.sin(ANGLE),0];C=np.array([100.,-100.,z]);D=np.array([100.,-4.,z])
    mid1=B+[sign*RADIUS*(1-math.cos(ANGLE/2)),RADIUS*math.sin(ANGLE/2),0]
    th=ANGLE/2;mid2=J+[sign*RADIUS*(math.cos(ANGLE-th)-math.cos(ANGLE)),RADIUS*(math.sin(ANGLE)-math.sin(ANGLE-th)),0]
    return [dict(kind='line',start=A.tolist(),end=B.tolist(),length=LEAD),dict(kind='arc',start=B.tolist(),mid=mid1.tolist(),end=J.tolist(),length=RADIUS*ANGLE),dict(kind='arc',start=J.tolist(),mid=mid2.tolist(),end=C.tolist(),length=RADIUS*ANGLE),dict(kind='line',start=C.tolist(),end=D.tolist(),length=96.)]

def edge(d):
    return cq.Edge.makeLine(cq.Vector(*d['start']),cq.Vector(*d['end'])) if d['kind']=='line' else cq.Edge.makeThreePointArc(cq.Vector(*d['start']),cq.Vector(*d['mid']),cq.Vector(*d['end']))

def sweep(edges,r):
    plane=cq.Plane(origin=edges[0].startPoint().toTuple(),normal=edges[0].tangentAt(0).toTuple());wire=cq.Wire.assembleEdges(edges)
    return cq.Workplane(plane).circle(r).sweep(cq.Workplane().newObject([wire]),isFrenet=True).val()

def build():
    oldnew,retained,_=a.build();old=next(z for z in oldnew if z['installed_part_candidate']);outer=[];bore=[];new=[]
    for i in [0,1]:
        edges=[edge(d) for d in definitions(i)];outer.append(sweep(edges[:3],2.5));bore.append(sweep(edges,1.6))
    body=old['shape'].fuse(*outer)
    for sh in bore:body=body.cut(sh)
    body=body.clean();assert body.isValid() and len(body.Solids())==1
    new.append(dict(name='wrist_paired_rear_S_guide',shape=body,body='palm',attachment_body='Palm_wrist_saddle',installed_part_candidate=True,color=[.1,.68,.65]))
    for i in [0,1]:
        edges=[edge(d) for d in definitions(i)]
        for kind,sh in [('housing',sweep(edges,1.5).cut(sweep(edges,.65)).clean()),('cord',sweep(edges,.3))]:
            assert sh.isValid() and len(sh.Solids())==1
            new.append(dict(name='paired_rear_'+kind+'_'+str(i),shape=sh,body='palm',physical_line=i,kind=kind,installed_part_candidate=False,color=[.63,.25,.8] if kind=='housing' else [.97,.45,.12]))
    outside=old['shape'].cut(body,tol=1e-6);lost=outside.Volume() if outside.Solids() else 0.
    design=dict(replaces_body='wrist_external_feed_clip',old_body_material_removed_mm3=lost,new_hardware_rows=1,retained_hardware_rows=5,local_housing_cord_rows=4,support_body='Palm_wrist_saddle',guide_radius_mm=RADIUS,lateral_offset_mm=OFFSET,rear_lead_mm=LEAD,rear_y_mm=REAR_Y,rear_ports_mm=[[90.,REAR_Y,-3.],[110.,REAR_Y,2.]],rear_axis=[0,1,0],front_ports_mm=[[100.,-4.,-3.],[100.,-4.,2.]],front_axis=[0,1,0],centerline_length_mm=LEAD+2*RADIUS*ANGLE+96,channel_edges=[definitions(i) for i in [0,1]],guide_OD_bore_mm=[5.,3.2],housing_OD_ID_mm=[3.,1.3],cord_OD_mm=.6,old_clamp_stack_axes_and_wall_seats_preserved=True,one_connected_custom_solid=True,new_guide_stiffness_friction_manufacture_exterior_approved=False,whole_flexible_link_connected=False)
    assert lost<1e-8;return new,retained,design

def window(i,tip,length=220.):
    """Constant-length leading reference fed along the real channel centerline."""
    ds=definitions(i);total=sum(d['length'] for d in ds);lo,hi=tip-length,tip;edges=[];start=0.
    A=np.array(ds[0]['start']);D=np.array(ds[-1]['end']);axis=np.array([0.,1.,0.])
    if lo<0:edges.append(cq.Edge.makeLine(cq.Vector(*(A+axis*lo)),cq.Vector(*(A+axis*min(hi,0.)))))
    for d in ds:
        left,right=max(0.,lo-start),min(d['length'],hi-start)
        if right-left>1e-8:
            ed=edge(d);edges.append(ed.trim(ed.paramAt(left/d['length']),ed.paramAt(right/d['length'])))
        start+=d['length']
    if hi>total:edges.append(cq.Edge.makeLine(cq.Vector(*(D+axis*max(lo-total,0.))),cq.Vector(*(D+axis*(hi-total)))))
    assert edges
    sh=sweep(edges,1.5).cut(sweep(edges,.65)).clean();assert sh.isValid() and len(sh.Solids())==1
    actual=sum(ed.Length() for ed in edges);assert abs(actual-length)<1e-5,(actual,length)
    return dict(name='feed_reference_'+str(i),shape=sh,kind='housing_reference',physical_line=i,length_mm=actual,tip_material_coordinate_mm=tip)

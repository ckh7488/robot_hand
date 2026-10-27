"""Same material sliding in the seated housings during one IP flex/release.

Parent and wrist stay neutral. Tail no-slip is an explicit ideal boundary;
friction, preload and equilibrium are not inferred from the nominal clamps.
"""
import sys,math,copy
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/motor_full_cord_thread_v1'))
import evidence_render as e
t,w,l,a,m,g,c,p,np,cq=e.t,e.w,e.l,e.a,e.m,e.g,e.c,e.p,e.np,e.cq
ANGLES=[0.,15.,30.,45.,30.,15.,0.]


def transform(O,R):
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=O;return T


def frames(q):
    parent=np.zeros(20);parent[19]=q
    O,R=g.g.v.frames(g.DESIGN,parent)['thumb_ip'];A,B=g.g.v.frames(g.DESIGN,np.zeros(20))['thumb_ip']
    child=transform(O,R)@np.linalg.inv(transform(A,B));M=w.motor.M
    theta=math.radians(.77*q);Rx=np.eye(4);Rx[:3,:3]=np.array([[1,0,0],[0,math.cos(theta),-math.sin(theta)],[0,math.sin(theta),math.cos(theta)]])
    return child,M@Rx@np.linalg.inv(M)


def paths(q,i):
    case=copy.deepcopy(t.neutral());case['q'][19]=q
    # Stored neutral middle edges are unchanged; no new biarc pose is solved.
    case['q'][19]=0.;middle=l.middle_edges(case,i,'cord');case['q'][19]=q
    M=w.motor.M;route=w.motor.route;port=w.motor.channels()[i];side=[-1.,1.][i];mx=[4.,12.][i]
    phase=math.radians(.77*q);aa=math.radians([-210.,210.][i])+phase;bb=math.radians([-90.,90.][i])
    A=route.circle(M[:3,3],M[:3,:3],mx,10.,aa);B=route.circle(M[:3,3],M[:3,:3],mx,10.,bb)
    anchor=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/reviews/check.json')['design']['anchors'][i]
    local=np.array(anchor['tail_exit_mm']);Rx=np.array([[1,0,0],[0,math.cos(phase),-math.sin(phase)],[0,math.sin(phase),math.cos(phase)]])
    tail=(M@np.r_[Rx@local,1.])[:3]
    segs=[route.line(tail,A,'motor_anchor_tail'),route.arc(A,route.circle(M[:3,3],M[:3,:3],mx,10.,(aa+bb)/2),B,10.*side*(aa-bb),'motor_wrap'),
        route.line(B,port['entry'],'free_tangent'),*port['pieces'],route.line(port['end'],[*port['xy'],-163.],'adjuster_passage')]
    motor=[w.seg_edge(z) for z in segs];distal=[ed for _,ed in w.distal_edges(case,i)]
    return motor+middle+distal


def profile(q,i):
    edges=paths(q,i);base=paths(0.,i);length=p.read(ROOT/'cad/biarc_supported_link_v1/reviews/check.json')['chains'][i]['length_mm']
    actual=sum(z.Length() for z in edges);gap=max((x.endPoint()-y.startPoint()).Length for x,y in zip(edges[:-1],edges[1:]))
    assert abs(actual-length)<1e-5 and gap<1e-5
    # Middle segment5 is the actual PAIR rear straight,40mm long.
    rear_index=6+5;rear_s=sum(z.Length() for z in edges[:rear_index]);rear0=sum(z.Length() for z in base[:rear_index]);witness_s=rear0+20.
    rear=np.array(edges[rear_index].startPoint().toTuple());axis=np.array(edges[rear_index].tangentAt(0).toTuple())
    point=np.array(t.point_at(edges,witness_s));location=witness_s-rear_s
    assert 0<location<40 and np.linalg.norm(point-(rear+location*axis))<1e-5
    child,motor=frames(q)
    endpoints=[np.array(base[0].startPoint().toTuple()),np.array(base[-1].endPoint().toTuple())]
    errors=[np.linalg.norm(np.array(ed.toTuple())-(T@np.r_[x,1.])[:3]) for ed,T,x in [(edges[0].startPoint(),motor,endpoints[0]),(edges[-1].endPoint(),child,endpoints[1])]]
    return dict(line=i,IP_deg=q,motor_output_deg=.77*q,material_length_mm=length,actual_length_mm=actual,maximum_join_gap_mm=gap,
        piece_lengths_mm=[z.Length() for z in edges],material_points=[dict(material_s_mm=float(s),world_mm=t.point_at(edges,float(s))) for s in np.linspace(0,length,9)],
        PAIR_rear_material_coordinate_mm=rear_s,PAIR_rear_material_coordinate_change_mm=rear_s-rear0,
        axis_sliding_witness=dict(material_s_mm=witness_s,world_mm=point,guide_rear_mm=rear,guide_axis=axis,guide_straight_length_mm=40.,location_from_rear_mm=location),
        tail_no_slip_endpoint_errors_mm=errors,child_transform=child,motor_transform=motor)


def context():
    case,data,groups,housing=e.inputs();source=data[0]
    meta=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json')['parts']
    moving={z['name'] for z in meta if z['moving']};allhw=[z for _,zz in groups for z in zz]
    # The inherited posed-hand loader retains names/shapes only. Recover the
    # accepted child ownership from its actual catalog, never a name prefix.
    child={z['name'] for z in p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'] if z['body']=='thumb_ip'}
    assert len(allhw)==862 and len(child)==13 and len(moving)==9 and moving.isdisjoint(child)
    for rel in ['cad/whole_cord_sliding_v1/slide_model.py','cad/motor_spool_coupling_v1/selected/generated/scene.json',
                'cad/motor_spool_coupling_v1/selected/reviews/check.json','cad/motor_full_cord_thread_v1/reviews/anchor_check.json',
                'cad/motion_clearance_v2/generated/assembly_catalog.json']:
        source[rel]=m.drive.sha(ROOT/rel)
    return source,groups,housing,moving,child


def hardware(groups,q,moving,child):
    C,M=frames(q);posed=[];changed=[]
    for key,rows in groups:
        result=[]
        for z in rows:
            T=M if z['name'] in moving else C if z['name'] in child else None
            v=dict(z,shape=g.original.transform(z['shape'],T)) if T is not None else z
            result.append(v)
            if T is not None:changed.append(v)
        posed.append((key,result))
    return posed,changed


def cords(q):
    rows=[]
    for i in [0,1]:
        for j,ed in enumerate(paths(q,i)):
            sh=m.swept(ed,.3);assert sh.isValid() and len(sh.Solids())==1
            rows.append(dict(name='sliding_cord_'+str(i)+'_'+str(j),shape=sh,kind='cord',physical_line=i,path_segment=j,
                length_mm=ed.Length(),body='palm',color=[.97,.45,.12],current_world_pose_only=True,
                endpoint_start_mm=ed.startPoint().toTuple(),endpoint_end_mm=ed.endPoint().toTuple()))
    assert len(rows)==52
    return rows


def export(rows,label):
    return a.a.prior.export([{k:v for k,v in z.items() if k not in ['step','mesh','sha256_step','sha256_stl']} for z in rows],HERE/'generated'/label)

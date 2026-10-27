"""Actual declared motor-tail to joint-tail centerline, not threaded hardware."""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import link_model as l
a,m,g,c,p,np,cq,ROOT,HERE=l.a,l.m,l.g,l.c,l.p,l.np,l.cq,l.ROOT,l.HERE
motor=m.drive.module('_biarc039_motor_terminal',ROOT/'cad/motor_tendon_terminal_v1/terminal_selected.py')
offset=m.drive.module('_biarc039_offset',ROOT/'cad/thumb_sheath_terminal_v2/offset_model.py')

def seg_edge(seg):return seg['edge'] if seg['kind']=='arc' else cq.Edge.makeLine(cq.Vector(*seg['a']),cq.Vector(*seg['z']))

def motor_edges(i):
    port=motor.channels()[i];mx=[4.,12.][i];side=[-1.,1.][i];aa=math.radians([-210.,210.][i]);bb=math.radians([-90.,90.][i]);route=motor.route;M=motor.M;anchors=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/reviews/check.json')['design']['anchors'];A=route.circle(M[:3,3],M[:3,:3],mx,10,aa);B=route.circle(M[:3,3],M[:3,:3],mx,10,bb);tail=route.point(np.asarray(anchors[i]['tail_exit_mm']));segments=[route.line(tail,A,'motor_anchor_tail'),route.arc(A,route.circle(M[:3,3],M[:3,:3],mx,10,(aa+bb)/2),B,10*side*(aa-bb),'motor_wrap'),route.line(B,port['entry'],'free_tangent'),*port['pieces'],route.line(port['end'],[*port['xy'],-163.],'adjuster_passage')]
    return [(z['label'],seg_edge(z)) for z in segments]

def distal_edges(case,i):
    q=np.asarray(case['q']);parent=q.copy();parent[19]=0.;O,R=g.g.v.frames(g.DESIGN,parent)['thumb_ip'];G=np.eye(4);G[:3,:3]=R;G[:3,3]=O;G=np.asarray(case['H'])@G;original=g.original;row=original.layout()[i];route=original.g;angle=math.radians(q[19]);side=row['side'];old=route.old.route(0.)[i];contact=old['contact_angle_rad'];x=row['track_x'];end=np.array([x,7.7*math.cos(angle),7.7*math.sin(angle)]);segments=[*row['pieces'],route.line(row['exit'],row['target'],'joint_approach'),route.arc(row['target'],[x,7.7*math.cos((angle+contact)/2),7.7*math.sin((angle+contact)/2)],end,7.7*(-side)*(angle-contact),'joint_wrap'),route.line(end,end+5*np.array([0.,side*math.sin(angle),-side*math.cos(angle)]),'joint_anchor_tail')]
    result=[]
    if i:
        Q=original.WORLD@original.layout()[1]['socket_transform'];T=np.asarray(case['H'])@np.asarray(case['F'])@Q
        for j,z in reversed(list(enumerate(offset.paths()))):
            ed=cq.Edge.makeThreePointArc(cq.Vector(*z['z']),cq.Vector(*z['mid']),cq.Vector(*z['a']));result.append(('upper_offset_reverse_'+str(j),g.original.transform(ed,T)))
    result += [(z['label'],g.original.transform(seg_edge(z),G)) for z in segments]
    return result

def ordered(case,i):return [('motor_'+name,ed) for name,ed in motor_edges(i)]+[('middle_'+str(j),ed) for j,ed in enumerate(l.middle_edges(case,i,'cord'))]+[('distal_'+name,ed) for name,ed in distal_edges(case,i)]

def build(case):
    middle,lengths=l.build(case);housing=[z for z in middle if z['kind']=='housing'];rows=list(housing);chains=[]
    for i in [0,1]:
        pieces=ordered(case,i);joins=[]
        for j,((left,ed),(right,nxt)) in enumerate(zip(pieces[:-1],pieces[1:])):
            joins.append(dict(index=j,left=left,right=right,endpoint_gap_mm=(ed.endPoint()-nxt.startPoint()).Length,tangent_vector_error=(ed.tangentAt(1)-nxt.tangentAt(0)).Length))
        assert max(z['endpoint_gap_mm'] for z in joins)<1e-5,joins
        for j,(label,ed) in enumerate(pieces):
            sh=m.swept(ed,.3);assert sh.isValid() and len(sh.Solids())==1
            rows.append(dict(name='whole_cord_'+str(i)+'_'+str(j)+'_'+label,shape=sh,physical_line=i,kind='cord',segment=j,role=label,length_mm=ed.Length(),body='palm',current_world_pose_only=True,color=[.97,.45,.12]))
        chains.append(dict(line=i,segments=len(pieces),length_mm=sum(ed.Length() for _,ed in pieces),start_motor_anchor_tail_mm=pieces[0][1].startPoint().toTuple(),end_joint_anchor_tail_mm=pieces[-1][1].endPoint().toTuple(),joins=joins,maximum_endpoint_gap_mm=max(z['endpoint_gap_mm'] for z in joins),maximum_join_tangent_vector_error=max(z['tangent_vector_error'] for z in joins),actual_threading_or_force_verified=False))
    return rows,lengths,chains

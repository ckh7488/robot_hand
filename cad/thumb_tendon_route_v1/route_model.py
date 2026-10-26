"""Two actual cord envelopes from the current motor to the accepted IP anchors.

Intermediate bends are guide LOCI, not installed/supporting pulley parts.
The common-mode length residual is retained: one antagonistic motor cannot
remove this residual by rotation alone. No rope tension/friction is simulated.
"""
import math,sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'cad/motor_spool_coupling_v1'))
import candidate2 as drive
c,b,m,p,cq=drive.c,drive.b,drive.m,drive.p,drive.cq
v=drive.module('_tendon_fk',ROOT/'cad/articulation_drive_v2/verify.py')
old=drive.module('_accepted_tendon_local',ROOT/'cad/thumb_ip_drive_v1/routing.py')
DESIGN=p.read(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
M=np.asarray(next(z['transform_world_mm'] for z in p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')['modules'] if z['name']==drive.NAME))
RADIUS=.3


def point(local):return (M@np.r_[local,1.])[:3]


def circle(origin,rotation,x,radius,angle):
    return np.asarray(origin)+rotation@np.array([x,radius*math.cos(angle),radius*math.sin(angle)])


def line(a,z,label):
    a,z=np.asarray(a),np.asarray(z);d=z-a;length=float(np.linalg.norm(d));assert length>1e-6
    return dict(label=label,a=a,z=z,length_mm=length,kind='line',tangent=d/length)


def arc(a,mid,z,length,label):
    a,mid,z=map(np.asarray,[a,mid,z]);edge=cq.Edge.makeThreePointArc(cq.Vector(*a),cq.Vector(*mid),cq.Vector(*z))
    assert abs(edge.Length()-length)<1e-5
    return dict(label=label,a=a,mid=mid,z=z,length_mm=float(length),kind='arc',edge=edge,tangent=np.asarray(edge.tangentAt(0).toTuple()))


def fillet_route(points,radius=5.):
    pts=[np.asarray(z,float) for z in points];bends={};out=[]
    for i in range(1,len(pts)-1):
        incoming=pts[i]-pts[i-1];outgoing=pts[i+1]-pts[i];li=np.linalg.norm(incoming);lo=np.linalg.norm(outgoing)
        u,w=incoming/li,outgoing/lo;phi=math.acos(float(np.clip(np.dot(u,w),-1,1)))
        assert 1e-5<phi<math.pi-1e-4
        distance=radius*math.tan(phi/2)
        assert distance<min(li,lo),('Insufficient straight length',i,distance,li,lo)
        a=pts[i]-u*distance;z=pts[i]+w*distance
        n=(w-u*np.dot(u,w))/math.sin(phi);center=a+n*radius
        radial=(a-center)/radius;mid=center+radius*(radial*math.cos(phi/2)+u*math.sin(phi/2))
        bends[i]=dict(a=a,z=z,mid=mid,center=center,radius_mm=radius,angle_rad=phi)
    start=pts[0]
    for i in range(1,len(pts)-1):
        bend=bends[i];segment=line(start,bend['a'],'free_straight_'+str(i));assert np.dot(segment['tangent'],(pts[i]-pts[i-1]))>0,'Overlapping fillets'
        out.append(segment);out.append(arc(bend['a'],bend['mid'],bend['z'],radius*bend['angle_rad'],'guide_arc_'+str(i)));start=bend['z']
    out.append(line(start,pts[-1],'joint_approach'))
    return out,[dict(index=i,center_mm=z['center'].tolist(),radius_mm=radius,guide_installed=False) for i,z in bends.items()]


def geometry(q,entry_y=-12.,motor_angle_deg=0.,turn_y=-60.,rise_y=-30.):
    q=np.asarray(q,float);parentq=q.copy();parentq[19]=0.
    O,R=v.frames(DESIGN,parentq)['thumb_ip'];joint=math.radians(q[19]);theta=math.radians(motor_angle_deg)
    anchors=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/reviews/check.json')['design']['anchors']
    result=[]
    for i,(x,side,mx,ma,mc) in enumerate(zip([-4.,4.],[-1.,1.],[4.,12.],[-210.,210.],[-90.,90.])):
        baseline=old.route(0.)[i];contact=baseline['contact_angle_rad']
        A=math.radians(ma)+theta;B=math.radians(mc)
        wrap=side*(A-B);jwrap=(-side)*(joint-contact)
        assert 0<wrap<2*math.pi and 0<jwrap<2*math.pi,('Wrap branch lost',i,motor_angle_deg,q[19],wrap,jwrap)
        motorstart=circle(M[:3,3],M[:3,:3],mx,10.,A);motorexit=circle(M[:3,3],M[:3,:3],mx,10.,B)
        local_joint=np.array([x,*baseline['joint_contact_mm'][1:]])
        T=O+R@local_joint
        # Use the original exact accepted straight-slot line at the new entry,
        # rather than fitting a cord through an arbitrary parent wall point.
        y0,z0=baseline['motor_contact_mm'][1:];yt,zt=baseline['joint_contact_mm'][1:]
        zp=z0+(entry_y-y0)*(zt-z0)/(yt-y0)
        P=O+R@np.array([x,entry_y,zp]);U=O+R@np.array([x,entry_y,zp+side*35.])
        freepoints=[motorexit,[motorexit[0],turn_y,motorexit[2]],
            [110.+i*10.,rise_y,motorexit[2]],[110.+i*10.,rise_y,70.+i*10.],U,P,T]
        pieces,guides=fillet_route(freepoints)
        parts=[arc(motorstart,circle(M[:3,3],M[:3,:3],mx,10.,(A+B)/2),motorexit,10.*wrap,'motor_wrap'),*pieces,
            arc(T,circle(O,R,x,7.7,(contact+joint)/2),circle(O,R,x,7.7,joint),7.7*jwrap,'joint_wrap')]
        # Joint tail remains tangent to its existing two-tail clamp.
        ja=parts[-1]['z'];taildir=R@np.array([0.,side*math.sin(joint),-side*math.cos(joint)])
        parts.append(line(ja,ja+5.*taildir,'joint_anchor_tail'))
        anchor=anchors[i];localtail=np.asarray(anchor['tail_exit_mm']);localrot=v.rotate('x',motor_angle_deg)
        endpoint=point(localrot@localtail)
        parts.insert(0,line(endpoint,motorstart,'motor_anchor_tail'))
        result.append(dict(cord=i,pieces=parts,guide_loci=guides,motor_wrap_rad=wrap,joint_wrap_rad=jwrap,
            length_mm=sum(z['length_mm'] for z in parts),motor_anchor_kink_deg=math.degrees(math.acos(2/math.sqrt(5))),
            motor_angle_deg=motor_angle_deg,entry_parent_y_mm=entry_y))
    return result


def coordinated(q,entry_y=-12.,turn_y=-60.,rise_y=-30.):
    neutral=geometry(np.zeros(20),entry_y,0.,turn_y,rise_y);base=geometry(q,entry_y,0.,turn_y,rise_y)
    changes=np.array([a['length_mm']-z['length_mm'] for a,z in zip(base,neutral)])
    angle=math.degrees((changes[0]-changes[1])/20.)
    rows=geometry(q,entry_y,angle,turn_y,rise_y);residuals=[a['length_mm']-z['length_mm'] for a,z in zip(rows,neutral)]
    assert abs(residuals[0]-residuals[1])<1e-7
    return rows,dict(motor_angle_deg=angle,neutral_lengths_mm=[z['length_mm'] for z in neutral],
        final_lengths_mm=[z['length_mm'] for z in rows],common_length_residual_mm=float(np.mean(residuals)),
        differential_residual_mm=float(residuals[0]-residuals[1]),motor_rotation_alone_keeps_both_lengths=bool(max(abs(np.array(residuals)))<1e-5),
        tensioner_installed=False,guide_loci_are_installed=False,force_equilibrium_solved=False)


def tubes(rows):
    parts=[]
    for row in rows:
        for index,seg in enumerate(row['pieces']):
            if seg['kind']=='line':shape=cq.Solid.makeCylinder(RADIUS,seg['length_mm'],cq.Vector(*seg['a']),cq.Vector(*seg['tangent']))
            else:
                path=cq.Wire.assembleEdges([seg['edge']]);plane=cq.Plane(origin=tuple(seg['a']),normal=tuple(seg['tangent']))
                shape=cq.Workplane(plane).circle(RADIUS).sweep(cq.Workplane().newObject([path]),isFrenet=True).val()
            assert shape.isValid() and len(shape.Solids())==1
            parts.append(dict(name='cord_'+str(row['cord'])+'_'+str(index)+'_'+seg['label'],cord=row['cord'],role=seg['label'],shape=shape,
                color=[.95,.29,.12] if row['cord']==0 else [.12,.65,.9],body='palm',a_mm=seg['a'].tolist(),z_mm=seg['z'].tolist(),length_mm=seg['length_mm']))
    return parts

"""Incremental, coupled geometry at actual ports, with exact contour budgets.

This transports an installed candidate, not an independent rod equilibrium.
Arc subdivisions are numerical coordinates. Native contact constraints couple
both housings; no support, friction, stiffness or tension is invented.
"""
import sys, math, copy
from pathlib import Path
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'cad/whole_cord_sliding_v1'))
import slide_model as s
t,w,l,a,m,g,c,p,np,cq = s.t,s.w,s.l,s.a,s.m,s.g,s.c,s.p,s.np,s.cq
rod = m.drive.module('_continue046_arc_math', ROOT/'cad/elastic_housing_link_v1/arc_rod.py')
from scipy.optimize._numdiff import approx_derivative
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN
from OCP.gp import gp_Pnt

NUP, NDOWN = 8, 6
N = 2*(NUP+NDOWN)+1
RADIUS = 1.5


def initial():
    base=t.neutral(); x=[]; weights=[]; budgets=[]
    for line in base['lines']:
        chunks=[]; ww=[]
        for side,n in [('up',NUP),('down',NDOWN)]:
            tangent=np.array(line[side]['arcs'][0]['t0']); e1,e2=rod.normal_frame(tangent)
            angles=[]; lengths=[]
            for raw in line[side]['arcs']:
                axis=np.cross(np.array(raw['t0']), (np.array(raw['center'])-raw['start'])/raw['radius_mm'])
                for _ in range(n//2):
                    theta=raw['angle_rad']/(n//2); normal=np.cross(axis,tangent)
                    angles.extend([theta*normal@e1,theta*normal@e2]); lengths.append(raw['length_mm']/(n//2))
                    co,si=math.cos(theta),math.sin(theta)
                    def rot(v):return v*co+np.cross(axis,v)*si+axis*(axis@v)*(1-co)
                    tangent,e1,e2=rot(tangent),rot(e1),rot(e2)
            chunks.extend(angles); ww.append(np.array(lengths)/sum(lengths))
        up=line['up']['length_mm']; down=line['down']['length_mm']
        x.extend(chunks+[up]); weights.append(ww); budgets.append(up+down)
    return base,np.array(x),weights,np.array(budgets)


def chain(A,t0,angles,lengths):
    pos=np.array(A,float); tangent=np.array(t0,float); e1,e2=rod.normal_frame(tangent); arcs=[]
    for (aa,bb),ell in zip(angles,lengths):
        theta=float(np.hypot(aa,bb)); normal=(aa*e1+bb*e2)/theta if theta>1e-12 else e1
        axis=np.cross(tangent,normal)
        def point(f):
            z=theta*f
            return pos+ell*(tangent*(np.sin(z)/theta if theta>1e-8 else f-theta*theta*f**3/6)+normal*((1-np.cos(z))/theta if theta>1e-8 else theta*f*f/2))
        end=point(1.); mid=point(.5); newt=tangent*np.cos(theta)+normal*np.sin(theta)
        arcs.append(dict(start=pos.copy(),mid=mid,end=end.copy(),tangent_start=tangent.copy(),tangent_end=newt.copy(),angle_rad=theta,length_mm=float(ell),radius_mm=float(ell/theta) if theta>1e-12 else None))
        def rot(v):return v*np.cos(theta)+np.cross(axis,v)*np.sin(theta)+axis*(axis@v)*(1-np.cos(theta))
        pos,tangent,e1,e2=end,newt,rot(e1),rot(e2)
    return arcs


def case_at(q,flex=0.,yaw=0.):
    P,H=a.feed.wrist_frames(flex,yaw); F=a.thumb_frame(np.array(q))
    return dict(q=list(q),P=P,H=H,F=F,wrist_flex_deg=flex,wrist_yaw_deg=yaw)


def ports(case,i):
    anchors=a.s.anchors(np.array(case['q']))[i]; H=np.array(case['H']); Q=l.b.frame(i)
    A=(Q@np.array([0.,-64.,-40.,1.]))[:3]
    return dict(up_A=A,up_B=(H@np.r_[anchors['up_B'],1.])[:3],up_tA=Q[:3,:3]@np.array([0.,0.,-1.]),
        up_tB=H[:3,:3]@np.array([0.,1.,0.]),down_A=(H@np.r_[anchors['down_A'],1.])[:3],
        down_B=(H@np.r_[anchors['down_B'],1.])[:3],down_tA=H[:3,:3]@np.array([0.,1.,0.]),down_tB=H[:3,:3]@anchors['down_tB'])


def curves(x,case,weights,budgets):
    result=[]
    for i in [0,1]:
        z=x[i*N:(i+1)*N]; e=ports(case,i)
        up=chain(e['up_A'],e['up_tA'],z[:2*NUP].reshape(-1,2),weights[i][0]*z[-1])
        down=chain(e['down_A'],e['down_tA'],z[2*NUP:-1].reshape(-1,2),weights[i][1]*(budgets[i]-z[-1]))
        result.append(dict(line=i,up=up,down=down,ports=e,up_length_mm=float(z[-1]),down_length_mm=float(budgets[i]-z[-1])))
    return result


def endpoints(x,case,weights,budgets):
    values=[]
    for row in curves(x,case,weights,budgets):
        for side in ['up','down']:
            last=row[side][-1]; target=row['ports'][side+'_tB']; e1,e2=rod.normal_frame(target)
            values.extend((last['end']-row['ports'][side+'_B'])/100.)
            values.extend([last['tangent_end']@e1,last['tangent_end']@e2])
    return np.array(values)


def samples(x,case,weights,budgets):
    rows=curves(x,case,weights,budgets); pts=[]; owners=[]
    for i,row in enumerate(rows):
        for side in ['up','down']:
            for j,z in enumerate(row[side]):
                A,T=np.array(z['start']),np.array(z['tangent_start']); theta=z['angle_rad']; ell=z['length_mm']
                normal=(np.array(z['mid'])-A-ell*T*(math.sin(theta/2)/theta if theta>1e-9 else .5))
                normal=normal/max(np.linalg.norm(normal),1e-20)
                for f in [0.,.25,.5,.75]:
                    v=theta*f
                    P=A+ell*(T*(math.sin(v)/theta if theta>1e-9 else f)+normal*((1-math.cos(v))/theta if theta>1e-9 else theta*f*f/2))
                    pts.append(P);owners.append((i,side,j,f))
            pts.append(row[side][-1]['end']);owners.append((i,side,len(row[side])-1,1.))
    return np.array(pts),owners


class NativeField:
    """AABB-pruned native point/shell contact, no triangle distance field."""
    def __init__(self,rows):
        self.rows=rows; self.box=np.array([c.m.pack._bbox(z['shape']) for z in rows]); self.shells={};self.calls=0
    def evaluate(self,points):
        records=[]
        for j,P in enumerate(points):
            gap=np.maximum(np.maximum(self.box[:,0]-P,P-self.box[:,1]),0.)
            # 0.2 is a native backend activation band, never design clearance.
            for k in np.flatnonzero(np.linalg.norm(gap,axis=1)<RADIUS+.2):
                if k not in self.shells:self.shells[k]=cq.Compound.makeCompound(self.rows[k]['shape'].Shells())
                sh=self.rows[k]['shape']; tool=BRepExtrema_DistShapeShape(cq.Vertex.makeVertex(*P).wrapped,self.shells[k].wrapped)
                tool.SetMultiThread(False);tool.Perform();self.calls+=1
                assert tool.IsDone() and tool.NbSolution()
                d=tool.Value();Q=np.array(tool.PointOnShape2(1).Coord());cl=BRepClass3d_SolidClassifier(sh.wrapped,gp_Pnt(*P),1e-7)
                sign=-1. if cl.State()==TopAbs_IN else 1.
                records.append((j,int(k),sign*d,sign*(P-Q)/max(d,1e-12)))
        return records


def fixed_edges(case,i,kind):
    neutral=t.neutral(); neutral.update(case)
    # Only the real fixed passages are used. Stored free neutral arcs below
    # are replaced by the continued curves, never solved as new biarcs.
    edges=l.middle_edges(neutral,i,kind)
    return edges[:3],edges[5:9],edges[11:]


def middle(x,case,weights,budgets,i,kind):
    row=curves(x,case,weights,budgets)[i]; before,guide,after=fixed_edges(case,i,kind)
    return before+[rod.edge(z) for z in row['up']]+guide+[rod.edge(z) for z in row['down']]+after


def all_samples(x,case,weights,budgets):
    P,owners=samples(x,case,weights,budgets); lines=[]; maps=[]
    for i in [0,1]:
        ids=[j for j,z in enumerate(owners) if z[0]==i]; line=P[ids].tolist(); ix=list(ids)
        # Fixed route samples participate in between-line constraints too.
        for group in fixed_edges(case,i,'housing'):
            for ed in group:
                for u in np.linspace(0.,1.,max(2,int(math.ceil(ed.Length()/5))+1)):
                    line.append(ed.positionAt(float(u)).toTuple());ix.append(-1)
        lines.append(np.array(line));maps.append(np.array(ix))
    return P,owners,lines,maps


def inequalities(x,case,weights,budgets,field,jacobian):
    P,owners,lines,maps=all_samples(x,case,weights,budgets)
    JP=approx_derivative(lambda y:samples(y,case,weights,budgets)[0].ravel(),x).reshape(len(P),3,len(x)) if jacobian else None
    values=[]; rows=[]; labels=[]
    native=field.evaluate(P)
    for j,k,d,grad in native:
        values.append((d-RADIUS)/100.); labels.append(('hardware',j,field.rows[k]['name'],d))
        if jacobian:rows.append(grad@JP[j]/100.)
    # Every free sample is coupled to every other physical line's samples.
    # This is a sampled necessary condition; full swept-material native pairs
    # are the acceptance test, including nonlocal self-contact.
    for j,own in enumerate(owners):
        other=1-own[0]; delta=P[j]-lines[other]; distance=np.linalg.norm(delta,axis=1)
        k=int(np.argmin(distance)); d=distance[k]; idx=maps[other][k]
        values.append((d-2*RADIUS)/100.); labels.append(('other_line',j,int(idx),float(d)))
        if jacobian:rows.append((delta[k]/max(d,1e-12))@(JP[j]-(JP[idx] if idx>=0 else 0))/100.)
    for i,row in enumerate(curves(x,case,weights,budgets)):
        for side in ['up','down']:
            for j,z in enumerate(row[side]):
                # Tube regularity uses its existing outer radius, no supplied
                # bend-radius specification is invented.
                theta=z['angle_rad']; ell=z['length_mm']; value=1-RADIUS*theta/ell
                values.append(value); labels.append(('tube_regular',i,side,j))
                if jacobian:
                    J=np.zeros(len(x)); k=i*N+2*(j if side=='up' else NUP+j)
                    if theta>1e-12:J[k:k+2]=-RADIUS*x[k:k+2]/theta/ell
                    J[i*N+N-1]=RADIUS*theta/ell/(row['up_length_mm'] if side=='up' else -row['down_length_mm'])
                    rows.append(J)
        for sign,value in [(1,x[i*N+N-1]),(-1,budgets[i]-x[i*N+N-1])]:
            J=np.zeros(len(x));J[i*N+N-1]=sign/100.;values.append(value/100.);labels.append(('positive_contour',i,sign))
            if jacobian:rows.append(J)
    return np.array(values),np.array(rows) if jacobian else None,labels


def projection(E,b,C,d,scale):
    """Minimum incremental coordinate change with a joint active contact set."""
    E=E*scale;C=C*scale; active=[]
    for _ in range(160):
        A=np.vstack([E,C[active]]) if active else E; rhs=np.r_[b,d[active]] if active else b
        multipliers=np.linalg.lstsq(A@A.T,rhs,rcond=1e-11)[0]; y=A.T@multipliers
        if active and min(multipliers[len(b):]) < -1e-8:
            active.pop(int(np.argmin(multipliers[len(b):])));continue
        gap=C@y-d; j=int(np.argmin(gap))
        if gap[j]>=-1e-9:return y*scale,active,float(max(abs(A@y-rhs)))
        if j in active:return y*scale,active,float(max(abs(A@y-rhs)))
        active.append(j)
    return y*scale,active,float(max(abs(A@y-rhs)))


def advance(previous,case,weights,budgets,obstacles):
    x=previous.copy(); field=NativeField(obstacles); history=[];scale=np.ones(len(x))
    for i in [0,1]:scale[i*N+N-1]=budgets[i]
    success=False
    for it in range(35):
        eq=endpoints(x,case,weights,budgets);ineq,C,labels=inequalities(x,case,weights,budgets,field,True)
        error=float(max(abs(eq))); minimum=float(min(ineq));rec=dict(iteration=it,endpoint_scaled_residual=error,minimum_inequality=minimum,native_point_shell_calls=field.calls)
        history.append(rec)
        if error<1e-8 and minimum>-1e-8:success=True;break
        E=approx_derivative(lambda z:endpoints(z,case,weights,budgets),x)
        step,active,residual=projection(E,-eq,C,-ineq,scale)
        # Trust-region damping is solely a numerical increment bound.
        # It does not restrict any joint angle or installed part motion.
        ratio=max(max(abs(step/scale))/.12,1.);step/=ratio
        merit=np.linalg.norm(eq)**2+np.linalg.norm(np.minimum(ineq,0.))**2
        accepted=False
        for alpha in [1.,.5,.25,.125,.0625,.03125]:
            trial=x+alpha*step
            if any(not 0<trial[i*N+N-1]<budgets[i] for i in [0,1]):continue
            f=endpoints(trial,case,weights,budgets);h,_,_=inequalities(trial,case,weights,budgets,field,False)
            new=np.linalg.norm(f)**2+np.linalg.norm(np.minimum(h,0.))**2
            if new<merit*(1-1e-5*alpha):x=trial;accepted=True;rec.update(damping=alpha,active_constraints=[labels[j] for j in active],linear_residual=residual);break
        if not accepted:rec['line_search_stalled']=True;break
    rows=curves(x,case,weights,budgets);ineq,_,labels=inequalities(x,case,weights,budgets,field,False)
    return x,dict(solved=success,parameters=x,case=case,history=history,endpoint_scaled_residual=float(max(abs(endpoints(x,case,weights,budgets)))),
        minimum_inequality=float(min(ineq)),limiting_constraint=labels[int(np.argmin(ineq))],native_point_shell_queries=field.calls,
        upstream_lengths_mm=[z['up_length_mm'] for z in rows],downstream_lengths_mm=[z['down_length_mm'] for z in rows],
        minimum_radius_mm=min(z['radius_mm'] for row in rows for side in ['up','down'] for z in row[side] if z['radius_mm'] is not None),
        same_total_material_lengths_by_construction=True,independent_pose_refit=False,physical_equilibrium_solved=False,
        invented_supports_or_tensioners=0,triangle_distance_field_constructed=False)

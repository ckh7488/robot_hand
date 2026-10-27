"""Inextensible circular-arc centerline model at actual supported ports.

Naturally straight isotropic bending only, normalized EI=1. No gravity,
twist, friction, contact forces, grip compliance or calibrated material.
"""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/wrist_external_feed_v1'))
import feed_model as a
m,g,c,p,np,cq=a.m,a.g,a.c,a.p,a.np,a.cq
from scipy.optimize import minimize
NUP,NDOWN=12,8
TOTAL=650.0

def normal_frame(t):
    e=np.array([1.,0,0]);e=e-t*np.dot(t,e)
    if np.linalg.norm(e)<.1:e=np.array([0.,0,1.])-t*t[2]
    e/=np.linalg.norm(e);return e,np.cross(t,e)

def chain(start,tangent,angles,length):
    pos=np.asarray(start,dtype=float).copy();t=np.asarray(tangent,dtype=float).copy();e1,e2=normal_frame(t);ell=length/len(angles);arcs=[]
    for aa,bb in angles:
        th=float(np.hypot(aa,bb));n=(aa*e1+bb*e2)/th if th>1e-12 else e1;axis=np.cross(t,n)
        def point(f):
            v=th*f
            return pos+ell*(t*(np.sin(v)/th if th>1e-9 else f)+n*((1-np.cos(v))/th if th>1e-9 else th*f*f/2))
        end=point(1.);mid=point(.5);newt=t*np.cos(th)+n*np.sin(th)
        def rotate(v):return v*np.cos(th)+np.cross(axis,v)*np.sin(th)+axis*np.dot(axis,v)*(1-np.cos(th))
        arcs.append(dict(start=pos.copy(),mid=mid,end=end.copy(),tangent_start=t.copy(),tangent_end=newt.copy(),angle_rad=th,length_mm=ell,radius_mm=ell/th if th>1e-12 else None))
        e1,e2=rotate(e1),rotate(e2);pos,t=end,newt
    return pos,t,arcs

def ports(q,i,flex=0.,yaw=0.):
    e=m.endpoints(q,i);P,H=a.wrist_frames(flex,yaw)
    for k in ['B','distal_seat','distal_cord']:e[k]=(H@np.r_[e[k],1.])[:3]
    e['tB']=H[:3,:3]@e['tB'];e['distal_Q']=H@e['distal_Q'];z=[-3.,2.][i]
    e['U']=(H@np.array([100.,-100.,z,1.]))[:3];e['V']=(H@np.array([100.,-4.,z,1.]))[:3];e['tG']=H[:3,:3]@np.array([0.,1.,0.]);e['H']=H;e['P']=P
    return e

def seed(A,B,tA,tB,n,handle):
    C=np.array([A,A+tA*handle,B-tB*handle,B]);ts=np.linspace(0,1,1001);points=np.array([(1-t)**3*C[0]+3*(1-t)**2*t*C[1]+3*(1-t)*t*t*C[2]+t**3*C[3] for t in ts]);ss=np.r_[0,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))];tt=np.interp(np.linspace(0,ss[-1],n+1),ss,ts);D=3*np.diff(C,axis=0);t=tA.copy();e1,e2=normal_frame(t);angles=[]
    for u in tt[1:]:
        newt=(1-u)**2*D[0]+2*(1-u)*u*D[1]+u*u*D[2];newt/=np.linalg.norm(newt);co=np.clip(np.dot(t,newt),-1,1);th=np.arccos(co);axis=np.cross(t,newt);axis/=max(np.linalg.norm(axis),1e-12);normal=np.cross(axis,t);angles.append([th*np.dot(normal,e1),th*np.dot(normal,e2)])
        def rot(v):return v*co+np.cross(axis,v)*np.sin(th)+axis*np.dot(axis,v)*(1-co)
        e1,e2=rot(e1),rot(e2);t=newt
    return np.array(angles),float(ss[-1])

def solve(q,i,flex=0.,yaw=0.,previous=None):
    e=ports(q,i,flex,yaw);A,B,U,V,tA,tB,tG=[e[k] for k in ['A','B','U','V','tA','tB','tG']];flexible=TOTAL-17.-25.-96.
    if previous is None:
        up,lu=seed(A,U,tA,tG,NUP,100.);down,ld=seed(V,B,tG,tB,NDOWN,25.);x0=np.r_[up.ravel(),down.ravel(),np.clip(flexible*lu/(lu+ld),300.,460.)]
    else:x0=np.asarray(previous['parameters'],dtype=float)
    G1,G2=normal_frame(tG);B1,B2=normal_frame(tB)
    def unpack(x):return chain(A,tA,x[:NUP*2].reshape(-1,2),x[-1]),chain(V,tG,x[NUP*2:-1].reshape(-1,2),flexible-x[-1])
    def constraints(x):
        up,down=unpack(x);return np.r_[(up[0]-U)/100.,[np.dot(up[1],G1),np.dot(up[1],G2)],(down[0]-B)/100.,[np.dot(down[1],B1),np.dot(down[1],B2)]]
    def objective(x):return 1000.*(np.sum(x[:NUP*2]**2)/(x[-1]/NUP)+np.sum(x[NUP*2:-1]**2)/((flexible-x[-1])/NDOWN))
    opt=minimize(objective,x0,method='SLSQP',bounds=[(-1.4,1.4)]*(2*(NUP+NDOWN))+[(300.,460.)],constraints=[dict(type='eq',fun=constraints)],options=dict(ftol=1e-11,maxiter=350))
    up,down=unpack(opt.x);residuals=[float(np.linalg.norm(up[0]-U)),float(np.linalg.norm(down[0]-B))];dots=[float(np.dot(up[1],tG)),float(np.dot(down[1],tB))]
    result=dict(physical_line=i,parameters=opt.x.tolist(),anchors={k:np.asarray(v).tolist() for k,v in e.items()},upstream_arcs=up[2],downstream_arcs=down[2],solver_success=bool(opt.success),solver_message=str(opt.message),iterations=int(opt.nit),endpoint_position_residual_mm=residuals,endpoint_tangent_dot=dots,normalized_bending_objective=objective(opt.x)/1000.,total_housing_candidate_length_mm=TOTAL,upstream_length_mm=float(opt.x[-1]),downstream_length_mm=float(flexible-opt.x[-1]),guide_material_coordinate_mm=float(17+opt.x[-1]),guide_straight_length_mm=96.,minimum_arc_radius_mm=min(z['radius_mm'] for z in up[2]+down[2] if z['radius_mm'] is not None),inextensible_piecewise_circular_arcs=True,EI_normalized_to_one=True,gravity_twist_contact_friction_or_tension_solved=False,material_or_supplier_bend_radius_verified=False,internal_arc_nodes_are_installed_guides=False)
    for key in ['upstream_arcs','downstream_arcs']:result[key]=[{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in z.items()} for z in result[key]]
    result['admissible_endpoint_solution']=bool(opt.success and max(residuals)<1e-5 and min(dots)>1-1e-8 and result['minimum_arc_radius_mm']>1.5)
    return result

def edge(arc):
    if arc['angle_rad']<1e-8:return cq.Edge.makeLine(cq.Vector(*arc['start']),cq.Vector(*arc['end']))
    return cq.Edge.makeThreePointArc(cq.Vector(*arc['start']),cq.Vector(*arc['mid']),cq.Vector(*arc['end']))

def build(solutions):
    rows=[]
    for s in solutions:
        assert s['admissible_endpoint_solution'];e={k:np.asarray(v) for k,v in s['anchors'].items()};i=s['physical_line']
        for kind in ['housing','cord']:
            edges=[cq.Edge.makeLine(cq.Vector(*e['motor_seat' if kind=='housing' else 'motor_cord']),cq.Vector(*e['A']))]+[edge(z) for z in s['upstream_arcs']]+[cq.Edge.makeLine(cq.Vector(*e['U']),cq.Vector(*e['V']))]+[edge(z) for z in s['downstream_arcs']]+[cq.Edge.makeLine(cq.Vector(*e['B']),cq.Vector(*e['distal_seat' if kind=='housing' else 'distal_cord']))]
            for j,ed in enumerate(edges):
                sh=m.swept(ed,1.5).cut(m.swept(ed,.65)).clean() if kind=='housing' else m.swept(ed,.3)
                assert sh.isValid() and len(sh.Solids())==1
                rows.append(dict(name='arc_'+kind+'_'+str(i)+'_'+str(j),shape=sh,physical_line=i,kind=kind,segment=j,length_mm=ed.Length(),body='palm',color=[.63,.25,.8] if kind=='housing' else [.96,.45,.12]))
    return rows

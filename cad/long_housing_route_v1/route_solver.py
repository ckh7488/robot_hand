"""Fixed-length quintic flexible spans, anchored only at real terminal ends.

Minimum curvature in this curve family is a geometric candidate. It is not
a calibrated elastic equilibrium, a friction solution or a physical support.
"""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/sheath_retention_v1'))
import retention_model as prior
g,c,p,np,cq,drive=prior.g,prior.c,prior.p,prior.np,prior.cq,prior.drive
from scipy.optimize import minimize
from scipy.integrate import quad
T,W=np.polynomial.legendre.leggauss(192);T=(T+1)/2;W=W/2
def bernstein(n,t):return np.asarray([math.comb(n,k)*t**k*(1-t)**(n-k) for k in range(n+1)]).T
B1=bernstein(4,T);B2=bernstein(3,T)
FLEX_LENGTH=500.
def properties(C):
    D=5*np.diff(C,axis=0);DD=4*np.diff(D,axis=0);d=B1@D;dd=B2@DD;speed=np.linalg.norm(d,axis=1);cross=np.linalg.norm(np.cross(d,dd),axis=1)
    return float(W@speed),float(W@(cross**2/np.maximum(speed,1e-9)**5)),float(np.max(cross/np.maximum(speed,1e-9)**3))
def endpoints(q,i):
    r=p.read(ROOT/'cad/sheath_retention_v1/selected/reviews/check.json');ports=r['design']['ports'];M=np.asarray(next(z for z in ports if z['tag']=='motor_housing_clamp_'+str(i))['Q']);D=np.asarray(next(z for z in ports if z['tag']=='distal_housing_clamp_'+str(i))['Q'])
    v=g.g.v;neutral=v.frames(g.DESIGN,np.zeros(20));posed=v.frames(g.DESIGN,q);O,R=posed['thumb_mcp'];O0,R0=neutral['thumb_mcp'];F=np.eye(4);F[:3,:3]=R@R0.T;F[:3,3]=O-F[:3,:3]@O0;D=F@D
    def point(Q,y):return (Q@np.array([0.,y,0.,1.]))[:3]
    return dict(A=point(M,-12),B=point(D,-11),tA=-M[:3,1],tB=D[:3,1],motor_seat=point(M,5),motor_cord=point(M,0),distal_seat=point(D,14),distal_cord=point(D,12 if i==0 else 15),motor_Q=M,distal_Q=D)
def solve(q,i,previous=None):
    e=endpoints(q,i);A,B,tA,tB=[e[k] for k in ['A','B','tA','tB']]
    def controls(x):return np.asarray([A,A+tA*x[0],x[2:5],x[5:8],B-tB*x[1],B])
    x0=np.r_[70.,70.,A+tA*180+np.array([70.,0,0]),B-tB*130+np.array([70.,0,0])]
    if previous is not None:x0=np.r_[np.linalg.norm(previous[1]-previous[0]),np.linalg.norm(previous[5]-previous[4]),previous[2],previous[3]]
    result=minimize(lambda x:properties(controls(x))[1]*FLEX_LENGTH,x0,method='SLSQP',bounds=[(10.,240.),(10.,240.)]+[(min(A[k%3],B[k%3])-400,max(A[k%3],B[k%3])+400) for k in range(6)],constraints=[dict(type='eq',fun=lambda x:properties(controls(x))[0]-FLEX_LENGTH)],options=dict(ftol=1e-10,maxiter=300))
    C=controls(result.x);length,bend,kappa=properties(C);D=5*np.diff(C,axis=0);independent=quad(lambda t:float(np.linalg.norm(bernstein(4,np.asarray([t]))[0]@D)),0.,1.,epsabs=1e-8,epsrel=1e-11,limit=200)[0]
    assert result.success and abs(independent-FLEX_LENGTH)<1e-5,(result.message,independent)
    return dict(cord=i,controls=C.tolist(),solver_success=bool(result.success),solver_message=str(result.message),iterations=int(result.nit),family='quintic_Bezier_endposition_endtangent_fixed_length',flex_length_candidate_mm=FLEX_LENGTH,quadrature_length_mm=length,independent_quad_length_mm=independent,bending_objective_1_per_mm=bend,sampled_minimum_curvature_radius_mm=1/kappa if kappa>1e-12 else None,anchors={k:np.asarray(v).tolist() for k,v in e.items()},physical_elastic_equilibrium_verified=False,midspan_guides_installed=False)
def split(C,t):
    levels=[np.asarray(C)]
    while len(levels[-1])>1:levels.append((1-t)*levels[-1][:-1]+t*levels[-1][1:])
    return np.asarray([z[0] for z in levels]),np.asarray([z[-1] for z in reversed(levels)])
def segment(C,a,b):
    left=split(C,b)[0];return split(left,a/b)[1] if a else left
def swept(edge,radius):
    plane=cq.Plane(origin=edge.startPoint().toTuple(),normal=edge.tangentAt(0).toTuple());return cq.Workplane(plane).circle(radius).sweep(cq.Workplane().newObject([cq.Wire.assembleEdges([edge])]),isFrenet=True).val()
def tubes(solution):
    C=np.asarray(solution['controls']);e={k:np.asarray(v) for k,v in solution['anchors'].items()};i=solution['cord'];rows=[]
    def line(a,b):return cq.Edge.makeLine(cq.Vector(*a),cq.Vector(*b))
    curves=[cq.Edge.makeBezier([cq.Vector(*z) for z in segment(C,a,b)]) for a,b in [(0.,1/3),(1/3,2/3),(2/3,1.)]]
    lengths={}
    for kind,edges in [('housing',[line(e['motor_seat'],e['A']),*curves,line(e['B'],e['distal_seat'])]),('cord',[line(e['motor_cord'],e['A']),*curves,line(e['B'],e['distal_cord'])])]:
        lengths[kind]=sum(z.Length() for z in edges)
        for j,edge in enumerate(edges):
            sh=swept(edge,1.5).cut(swept(edge,.65)).clean() if kind=='housing' else swept(edge,.3)
            assert sh.isValid() and len(sh.Solids())==1 and sh.Volume()>1e-6,(kind,i,j)
            rows.append(dict(name='long_'+kind+'_'+str(i)+'_'+str(j),shape=sh,body='palm',physical_line=i,kind=kind,segment=j,length_mm=edge.Length(),color=[.6,.27,.78] if kind=='housing' else ([.95,.33,.1] if i==0 else [.15,.6,.95])))
    return rows,lengths

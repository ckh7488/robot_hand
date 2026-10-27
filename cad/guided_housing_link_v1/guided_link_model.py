"""One through-guide topology with constant total length and axial sliding.

The real end clamps and wrist sleeve constrain the curve. The polynomial
coefficients describe unsupported flexible spans, not installed guide points.
This kinematic family does not solve elastic equilibrium, contact or tension.
"""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/long_housing_route_v1'))
import route_solver as m
g,c,p,np,cq=m.g,m.c,m.p,m.np,m.cq
from scipy.integrate import quad
from scipy.optimize import brentq

def controls(q,i,slack):
    e=m.endpoints(q,i);A,B=e['A'],e['B'];z=[-3.,2.][i]
    U=np.array([26.,-18.,z]);V=np.array([26.,8.,z])
    # An external upstream bow and a downward distal return are a single
    # bounded topology. No coefficient is treated as a physical support.
    up=np.array([A,A+e['tA']*55,
                 [110+i*8+slack*70,-240.,-250.-slack*40],
                 [90+i*8+slack*70,-80.,-80.-slack*40],
                 U-[0,60,0],U])
    down=np.array([V,V+[0,5,0],[26,18,-65.],
                   [75+i*3,-5,-60.],B-e['tB']*30,B])
    return e,up,down,U,V

def length(C):
    D=5*np.diff(C,axis=0)
    return float(quad(lambda t:np.linalg.norm(m.bernstein(4,np.array([t]))[0]@D),0,1,epsabs=1e-8,epsrel=1e-11,limit=200)[0])

def describe(q,i,slack):
    e,U,D,a,b=controls(q,i,slack);lu,ld=length(U),length(D)
    fixed_motor=float(np.linalg.norm(e['A']-e['motor_seat']))
    fixed_distal=float(np.linalg.norm(e['distal_seat']-e['B']))
    return dict(physical_line=i,slack_parameter=float(slack),upstream_controls=U.tolist(),downstream_controls=D.tolist(),
                guide_entry_mm=a.tolist(),guide_exit_mm=b.tolist(),anchors={k:np.asarray(v).tolist() for k,v in e.items()},
                upstream_flexible_length_mm=lu,downstream_flexible_length_mm=ld,guide_straight_length_mm=float(np.linalg.norm(b-a)),
                motor_end_straight_length_mm=fixed_motor,distal_end_straight_length_mm=fixed_distal,
                total_housing_length_mm=lu+ld+np.linalg.norm(b-a)+fixed_motor+fixed_distal,
                material_coordinate_at_guide_entry_mm=fixed_motor+lu,
                sampled_curvature_radius_mm=[1/m.properties(C)[2] for C in [U,D]],
                coefficients_are_installed_guides=False,physical_equilibrium_verified=False)

def solve(q,i,target):
    f=lambda x:describe(q,i,x)['total_housing_length_mm']-target
    samples=np.linspace(-.5,2.,26);brackets=[(a,b) for a,b in zip(samples[:-1],samples[1:]) if f(a)*f(b)<=0]
    assert brackets,'No constant-length solution in this bounded family'
    roots=[brentq(f,a,b,xtol=1e-11) for a,b in brackets];s=describe(q,i,min(roots,key=abs))
    s['target_total_length_mm']=target;s['total_length_residual_mm']=s['total_housing_length_mm']-target
    assert abs(s['total_length_residual_mm'])<1e-5
    return s

def edges(s,kind):
    e={k:np.asarray(v) for k,v in s['anchors'].items()};U=np.asarray(s['upstream_controls']);D=np.asarray(s['downstream_controls'])
    line=lambda a,b:cq.Edge.makeLine(cq.Vector(*a),cq.Vector(*b))
    curve=lambda C:cq.Edge.makeBezier([cq.Vector(*z) for z in C])
    spans=[curve(m.segment(U,a,b)) for a,b in [(0.,1/3),(1/3,2/3),(2/3,1.)]]
    spans+=[line(np.asarray(s['guide_entry_mm']),np.asarray(s['guide_exit_mm']))]
    spans+=[curve(m.segment(D,a,b)) for a,b in [(0.,1/3),(1/3,2/3),(2/3,1.)]]
    return [line(e['motor_seat' if kind=='housing' else 'motor_cord'],e['A']),*spans,
            line(e['B'],e['distal_seat' if kind=='housing' else 'distal_cord'])]

def build(solutions):
    rows=[]
    for s in solutions:
        for kind in ['housing','cord']:
            for j,edge in enumerate(edges(s,kind)):
                sh=m.swept(edge,1.5).cut(m.swept(edge,.65)).clean() if kind=='housing' else m.swept(edge,.3)
                assert sh.isValid() and len(sh.Solids())==1 and sh.Volume()>1e-6,(kind,j)
                rows.append(dict(name='guided_'+kind+'_'+str(s['physical_line'])+'_'+str(j),shape=sh,body='palm',
                                 kind=kind,physical_line=s['physical_line'],segment=j,length_mm=edge.Length(),
                                 color=[.6,.27,.78] if kind=='housing' else [.96,.4,.1]))
    return rows

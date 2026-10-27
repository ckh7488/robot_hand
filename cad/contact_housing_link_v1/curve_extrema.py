"""Polynomial curvature extrema of each saved cubic span; no route search."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import contact_solver as s
np,p,ROOT,HERE=s.np,s.p,s.ROOT,s.HERE
from numpy.polynomial import Polynomial as Poly
def extrema(C):
    spans=[]
    for left,right in zip([0.,.2,.4,.6,.8],[.2,.4,.6,.8,1.]):
        h=right-left;co=np.array([s.SPL(left)@C,h*(s.SPL.derivative()(left)@C),h*h/2*(s.SPL.derivative(2)(left)@C),h**3/6*(s.SPL.derivative(3)((left+right)/2)@C)]);v=np.array([co[1],2*co[2],3*co[3]]);cross=np.array([np.cross(v[0],v[1]),2*np.cross(v[0],v[2]),np.cross(v[1],v[2])]);N=sum((Poly(cross[:,j])**2 for j in range(3)),Poly([0.]));S=sum((Poly(v[:,j])**2 for j in range(3)),Poly([0.]));F=N.deriv()*S-3*N*S.deriv();roots=F.roots();us=[0.,1.]+[float(z.real) for z in roots if abs(z.imag)<1e-7 and 0<float(z.real)<1];ks=[max(float(N(u)),0)/max(float(S(u)),1e-30)**3 for u in us];k=int(np.argmax(ks));speedu=[0.,1.]+[float(z.real) for z in S.deriv().roots() if abs(z.imag)<1e-7 and 0<float(z.real)<1];spans.append(dict(parameter_interval=[left,right],position_polynomial_u_coefficients=co.tolist(),curvature_stationary_real_u=us[2:],minimum_radius_mm=1/np.sqrt(max(ks)) if max(ks)>0 else None,maximum_curvature_squared=max(ks),critical_global_parameter=left+h*us[k],minimum_parameter_speed_mm=min(np.sqrt(max(float(S(u)),0))/h for u in speedu)))
    ts=np.linspace(0,1,4097);v=s.SPL.derivative()(ts)@C;acc=s.SPL.derivative(2)(ts)@C;speed=np.linalg.norm(v,axis=1);kappa=np.linalg.norm(np.cross(v,acc),axis=1)/np.maximum(speed,1e-20)**3;best=min(spans,key=lambda z:z['minimum_radius_mm'] if z['minimum_radius_mm'] is not None else np.inf)
    return dict(spans=spans,minimum_radius_mm=best['minimum_radius_mm'],critical_parameter=best['critical_global_parameter'],dense4097_minimum_radius_mm=float(1/kappa.max()),minimum_parameter_speed_mm=min(z['minimum_parameter_speed_mm'] for z in spans),necessary_radius_greater_than_housing_outer_radius_1p5=bool(best['minimum_radius_mm']>1.5))
def main():
    raw=p.read(HERE/'reviews/check.json');C=np.asarray(raw['solver']['controls']);checks=[dict(line=i//2,span='up' if i%2==0 else 'down',**extrema(z)) for i,z in enumerate(C)];p.write(HERE/'reviews/extrema.json',dict(completed=True,curves=checks,minimum_radius_mm=min(z['minimum_radius_mm'] for z in checks),all_necessary_local_radius_checks_pass=all(z['necessary_radius_greater_than_housing_outer_radius_1p5'] for z in checks),supplier_minimum_radius_or_life_approved=False,whole_selfcontact_or_physical_equilibrium_approved=False,optimizer_or_native_groups_repeated=False));print([(z['line'],z['span'],z['minimum_radius_mm']) for z in checks],flush=True)
if __name__=='__main__':main()

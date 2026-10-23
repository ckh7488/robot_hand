"""Independent dense curvature and quadrature verification of geometric loop candidates."""
import numpy as np,json,math
from scipy.special import comb
from scipy.integrate import quad
from pathlib import Path
D=Path(__file__).resolve().parent;j=json.loads((D/'reviews'/'service_loop_curves.json').read_text(encoding='utf-8'));assert j['chosen'];L=j['chosen']['length_mm'];t=np.linspace(0,1,10001)
def basis(t,n):return np.array([comb(n,i)*t**i*(1-t)**(n-i) for i in range(n+1)]).T
B=basis(t,5);Bd=basis(t,4);Bdd=basis(t,3);rows=[];allp=[];per={}
for r in j['chosen']['curves']:
 P=np.array(r['controls_mm']);v=Bd@(5*np.diff(P,axis=0));a=Bdd@(20*np.diff(P,n=2,axis=0));speed=np.linalg.norm(v,axis=1);k=np.linalg.norm(np.cross(v,a),axis=1)/speed**3;dp=5*np.diff(P,axis=0)
 length,err=quad(lambda u:float(np.linalg.norm(basis(np.array([u]),4)[0]@dp)),0,1,epsabs=1e-9,limit=100);pts=B@P;allp.append(pts);per[(r['lane'],r['q_deg'])]=pts[::50];rows.append({'q_deg':r['q_deg'],'lane':r['lane'],'independent_length_mm':length,'length_error_mm':length-L,'quad_error_bound_mm':err,'minimum_sampled_radius_mm':float(1/max(k)),'minimum_parametric_speed':float(min(speed))})
a=np.concatenate(allp);qs=sorted({r['q_deg'] for r in rows});maxstep=max(float(np.max(np.linalg.norm(per[(lane,b)]-per[(lane,a)],axis=1))) for a,b in zip(qs,qs[1:]) for lane in range(3));out={'scope':'Independent adaptive quadrature and10001samples per curve. Sampled radius and inter-angle continuity screen, not physical equilibrium or continuous-joint certification.','target_length_mm':L,'minimum_dense_sampled_radius_mm':min(r['minimum_sampled_radius_mm'] for r in rows),'maximum_absolute_length_error_mm':max(abs(r['length_error_mm']) for r in rows),'loop_bounds_little_MCP_frame_mm':[a.min(0).tolist(),a.max(0).tolist()],'largest_matched_curve_point_shift_per_2deg_mm':maxstep,'rows':rows,'approved_for_manufacture':False};(D/'reviews'/'loop_geometry_verified.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print({k:v for k,v in out.items() if k!='rows'})

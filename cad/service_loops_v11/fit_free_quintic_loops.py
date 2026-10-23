"""Coupled constant-length 3-wire spatial routing; geometric candidate, not cable mechanics."""
import numpy as np,json,math,time
from pathlib import Path
from scipy.optimize import minimize
from scipy.special import comb
from scipy.spatial.distance import cdist
D=Path(__file__).resolve().parent;T=np.linspace(0,1,151);U=np.polynomial.legendre.leggauss(64);GT=(U[0]+1)/2;GW=U[1]/2
basis=lambda t,d:np.array([comb(d,i)*t**i*(1-t)**(d-i) for i in range(d+1)]).T
BT=basis(T,5);BD=basis(T,4);BDD=basis(T,3);BG=basis(GT,4)
def controls(q,lane,x):
 y=[-11.5,-9,-6.5][lane];r=[10,12.5,15][lane];c=math.cos(math.radians(q));s=math.sin(math.radians(q));R=np.array([[1,0,0],[0,c,-s],[0,s,c]]);p0=np.array([-44.5,y,-36.]);p5=R@np.array([-44.5,55+y,-36.]);t0=np.array([0.,0.,-1.]);t1=R@np.array([0.,0.,1.]);a,b,h,dy,dz=x;v=np.array([-h,dy,dz]);return np.array([p0,p0+a*t0,p0+2*a*t0+v,p5-2*b*t1+v,p5-b*t1,p5])
def evaluate(q,x):
 P=np.array([controls(q,i,z) for i,z in enumerate(x.reshape(3,5))]);dp=5*np.diff(P,axis=1);ddp=4*np.diff(dp,axis=1);v=BD@dp;acc=BDD@ddp;sp=np.linalg.norm(v,axis=2);k=np.linalg.norm(np.cross(v,acc),axis=2)/np.maximum(sp,1e-12)**3;length=np.linalg.norm(BG@dp,axis=2)@GW;pts=BT@P;gaps=np.array([cdist(pts[i],pts[j]).min() for i,j in [(0,1),(1,2),(0,2)]]);return P,length,k,gaps
old=json.loads((D/'reviews'/'independent_spatial_rejected.json').read_text(encoding='utf-8'));oldrows={r['q_deg']:r for r in old['chosen']['curves'] if r['lane']==1};search=[];chosen=None;start=time.monotonic()
for L in [120,140]:
 rows=[];success=True;previous=None
 for q in [0,*np.arange(-2,-20.01,-2),*np.arange(2,60.01,2)]:
  base=np.array([[L/5,L/5,12+5*i,0,-5] for i in range(3)]).ravel();starts=[previous if previous is not None and q!=2 else base,base,np.array([[15,15,10+5*i,0,-10] for i in range(3)]).ravel()]
  best=None
  for x0 in starts:
   lastx=None;last=None
   def ev(x):
    global lastx,last
    if lastx is None or not np.array_equal(lastx,x):lastx=x.copy();last=evaluate(q,x)
    return last
   def objective(x):
    X=x.reshape(3,5);return float(np.sum((X-X.mean(axis=0))**2)*.01+np.sum((x-(previous if previous is not None else base))**2)*.001+1e-5*np.sum(x*x))
   cons=[{'type':'eq','fun':lambda x:(ev(x)[1]-L)/L},{'type':'ineq','fun':lambda x:(1/10.15-np.max(ev(x)[2],axis=1))*100},{'type':'ineq','fun':lambda x:ev(x)[3]-2.32}]
   res=minimize(objective,x0,method='SLSQP',bounds=[(3,120),(3,120),(0,90),(-60,60),(-80,40)]*3,constraints=cons,options={'maxiter':180,'ftol':1e-9});P,length,k,gaps=evaluate(q,res.x);valid=max(abs(length-L))<1e-4 and max(k.ravel())<=1/10.14 and min(gaps)>=2.3199
   quality=float(max(abs(length-L))+100*max(0,max(k.ravel())-1/10.14)+max(0,2.3199-min(gaps)))
   if best is None or quality<best[0]:best=(quality,res.x,P,length,k,gaps,valid,res.message)
   if valid:break
  _,x,P,length,k,gaps,valid,msg=best
  print('L',L,'q',q,'ok',valid,'R',1/max(k.ravel()),'wire_centers',min(gaps),'lengtherr',max(abs(length-L)),flush=True)
  if not valid:success=False;break
  for lane in range(3):rows.append({'q_deg':float(q),'lane':lane,'feasible':True,'length_mm':float(length[lane]),'minimum_sampled_radius_mm':float(1/max(k[lane])),'controls_mm':P[lane].tolist(),'handle_lengths_mm':x.reshape(3,5)[lane].tolist()})
  previous=x
 search.append({'target_length_mm':L,'all_3_lanes_41_angles_feasible':success,'completed_curves':len(rows),'failed_q_deg':None if success else float(q)})
 if success:chosen={'length_mm':L,'curves':rows};break
out={'scope':'Coupled spatial quintic Bezier candidates. Fixed length of every wire, fixed endpoint tangents, minimum sampled curvature radius10.14mm, minimum sampled wire center gap2.32mm. Exact CAD capsule validation remains required. No contact/friction/gravity/material law; not a physical cable simulation.','radius_threshold_mm':10,'angles_step_deg':2,'curve_samples':151,'length_quadrature_order':64,'searches':search,'chosen':chosen,'elapsed_s':time.monotonic()-start,'full_harness_approved':False};(D/'reviews'/'service_loop_curves.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print('COMPLETE',search,flush=True)

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
old=json.loads((D/'reviews'/'unsmoothed_free_quintic.json').read_text(encoding='utf-8'));assert old['chosen'];L=old['chosen']['length_mm'];rows=[r for r in old['chosen']['curves'] if r['q_deg']>0];previous=np.array([r['handle_lengths_mm'] for r in sorted([r for r in rows if r['q_deg']==2],key=lambda r:r['lane'])]).ravel();previous_q=2;passed=True;begin=time.monotonic()
for q in [0,*np.arange(-2,-20.01,-2)]:
 previous_pts=BT@np.array([controls(previous_q,i,previous.reshape(3,5)[i]) for i in range(3)]);lastx=None;last=None
 def ev(x):
  global lastx,last
  if lastx is None or not np.array_equal(lastx,x):lastx=x.copy();last=evaluate(q,x)
  return last
 def objective(x):return float(np.sum((x-previous)**2))
 cons=[{'type':'eq','fun':lambda x:(ev(x)[1]-L)/L},{'type':'ineq','fun':lambda x:(1/10.15-np.max(ev(x)[2],axis=1))*100},{'type':'ineq','fun':lambda x:ev(x)[3]-2.32},{'type':'ineq','fun':lambda x:6-np.max(np.linalg.norm(BT@ev(x)[0]-previous_pts,axis=2))}]
 res=minimize(objective,previous,method='SLSQP',bounds=[(3,120),(3,120),(0,90),(-60,60),(-80,40)]*3,constraints=cons,options={'maxiter':500,'ftol':1e-9});P,length,k,gaps=evaluate(q,res.x);shift=float(np.max(np.linalg.norm(BT@P-previous_pts,axis=2)));valid=max(abs(length-L))<1e-4 and max(k.ravel())<=1/10.05 and min(gaps)>=2.3199 and shift<=6.001;print(q,'valid',valid,'shift',shift,'R',1/max(k.ravel()),'gap',min(gaps),'lengtherr',max(abs(length-L)),'status',res.message,flush=True)
 if not valid:passed=False;break
 for lane in range(3):rows.append({'q_deg':float(q),'lane':lane,'feasible':True,'length_mm':float(length[lane]),'minimum_sampled_radius_mm':float(1/max(k[lane])),'controls_mm':P[lane].tolist(),'handle_lengths_mm':res.x.reshape(3,5)[lane].tolist()})
 previous=res.x;previous_q=q
if passed:
 old['chosen']['curves']=rows;old['smoothing']='Re-solved0through-20 from existing+2branch with6mm maximum sampled matched-point change per2deg. Length/radius/wire-distance constraints retained. Positive branch unchanged.';(D/'reviews'/'service_loop_curves.json').write_text(json.dumps(old,indent=2),encoding='utf-8')
(D/'reviews'/'loop_smoothing.json').write_text(json.dumps({'passed':passed,'elapsed_s':time.monotonic()-begin,'last_q_deg':float(q),'scope':'Geometric branch continuity only, not flexible-body equilibrium.'},indent=2),encoding='utf-8')

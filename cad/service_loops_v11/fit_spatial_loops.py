"""Constant-length, tangent-constrained geometric cable candidates. Not flexible-body simulation."""
import sys,json,math,time,numpy as np
from pathlib import Path
from scipy.optimize import minimize
from scipy.special import comb
D=Path(__file__).resolve().parent
T=np.linspace(0,1,201);U=np.polynomial.legendre.leggauss(64);GT=(U[0]+1)/2;GW=U[1]/2

def basis(t,degree):return np.array([comb(degree,i)*t**i*(1-t)**(degree-i) for i in range(degree+1)]).T
BT=basis(T,5);BD=basis(T,4);BDD=basis(T,3);BG=basis(GT,4)
def ends(q,lane):
 y=[-11.5,-9,-6.5][lane];r=[10,12.5,15][lane];c=math.cos(math.radians(q));s=math.sin(math.radians(q));R=np.array([[1,0,0],[0,c,-s],[0,s,c]])
 p0=np.array([-44.5,y,-36.]);p3=R@np.array([-34.5-r,19.,-8.]);return p0,p3,np.array([0.,0.,-1.]),R@np.array([0.,1.,0.])
def controls(q,lane,x):
 p0,p3,t0,t1=ends(q,lane);a,b,h=x[:3];v=np.array([-h,0.,0.]);return np.array([p0,p0+a*t0,p0+2*a*t0+v,p3-2*b*t1+v,p3-b*t1,p3])
def metrics(P):
 d=5*np.diff(P,axis=0);dd=4*np.diff(d,axis=0);v=BD@d;acc=BDD@dd;speed=np.linalg.norm(v,axis=1);curv=np.linalg.norm(np.cross(v,acc),axis=1)/np.maximum(speed,1e-12)**3;length=float(GW@np.linalg.norm(BG@d,axis=1));return length,curv,speed
rows=[];chosen=None;begin=time.monotonic()
for L in [100,120,140,160]:
 trial=[];ok=True
 for lane in range(3):
  previous=np.array([L/3,L/3,30.])
  for q in np.arange(-20,60.01,2):
   def objective(x):
    le,k,sp=metrics(controls(q,lane,x));return float(np.max(k)+1e-7*np.sum(x*x))
   cons=[{'type':'eq','fun':lambda x:metrics(controls(q,lane,x))[0]-L}]
   candidates=[]
   for start in [previous,np.array([L/3,L/3,40.]),np.array([L*.4,L*.25,50.])] :
    res=minimize(objective,start,method='SLSQP',bounds=[(3,100),(3,100),(5,80)],constraints=cons,options={'maxiter':120,'ftol':1e-10})
    le,k,sp=metrics(controls(q,lane,res.x))
    if abs(le-L)<1e-5:candidates.append((float(max(k)),res.x,le,float(min(sp))))
   if not candidates:ok=False;trial.append({'q_deg':float(q),'lane':lane,'feasible':False,'reason':'no length-constrained solution'});break
   k,x,le,sp=min(candidates,key=lambda x:x[0]);previous=x;row={'q_deg':float(q),'lane':lane,'feasible':k<=.1 and sp>1e-4,'length_mm':le,'minimum_sampled_radius_mm':1/k,'controls_mm':controls(q,lane,x).tolist(),'handle_lengths_mm':x.tolist()};trial.append(row)
   if not row['feasible']:ok=False;break
  if not ok:break
 rows.append({'target_length_mm':L,'all_3_lanes_41_angles_feasible':ok,'worst_sampled_radius_mm':min((r.get('minimum_sampled_radius_mm',0) for r in trial),default=0),'cases':len(trial)})
 print(rows[-1],flush=True)
 if ok:chosen={'length_mm':L,'curves':trial};break
out={'scope':'Geometric spatial quintic Bezier candidates, fixed individual wire length and endpoint tangents, 3 wires, one little MCP hinge. End attachments derive from V10 fixed exits. No contact/friction/gravity/material constitutive law; not the physical cable shape or flexible-body simulation.','radius_threshold_mm':10,'radius_threshold_provenance':'Design assumption; actual cable bend/fatigue specification must be measured or supplied.','angles_step_deg':2,'curve_samples':201,'length_quadrature_order':64,'searches':rows,'chosen':chosen,'elapsed_s':time.monotonic()-begin,'full_harness_approved':False};(D/'reviews'/'service_loop_curves.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print('DONE',flush=True)

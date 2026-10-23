"""Offline continuous spread-error boxes around fixed other joints. Never a full robot safety certificate."""
import sys,json,copy,math,time,numpy as np,fcl
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward,rigid_body,eligible
from mesh_collision import Checker
check=Checker();ids=check.ids;items=[META[i] for i in ids];pairs=check.pairs;F=['index','middle','ring','little'];definition={f['name']:f for f in C['hand']['fingers']};dq=fcl.DistanceRequest();cq=fcl.CollisionRequest(num_max_contacts=1);target=1.1

def rodrigues(axis,deg):
 v=axis;K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]);a=math.radians(deg);return np.eye(3)+math.sin(a)*K+(1-math.cos(a))*(K@K)
def test(pose,eps):
 M=forward(pose)[ids];dep=[];rad=[];center={};axis={};lo=[];hi=[]
 for f in F:
  d=definition[f];rx,ry,rz=d['base_rpy'];from kinematics_numpy import rot
  center[f]=np.array(d['root']);axis[f]=rot(2,rz)@rot(1,ry)@rot(0,rx)@np.array([0.,0.,1.])
 for it,m in zip(items,M):
  body=rigid_body(it['name']);f=next((f for f in F if body.startswith(f+'_')),None);dep.append(f);v=check.meshes[it['asset']].vertices@m[:3,:3].T+m[:3,3];lo.append(v.min(0));hi.append(v.max(0))
  if f:
   v=v-center[f];v-=np.outer(v@axis[f],axis[f]);rad.append(float(np.linalg.norm(v,axis=1).max())+.05)
  else:rad.append(0.)
 lo=np.array(lo);hi=np.array(hi);rad=np.array(rad);ii,jj=pairs[:,0],pairs[:,1];disp=2*rad*np.sin(np.deg2rad(eps)/2);gaps=np.maximum(0,np.maximum(lo[ii]-hi[jj],lo[jj]-hi[ii]));mask=np.linalg.norm(gaps,axis=1)<=target+disp[ii]+disp[jj];near=pairs[mask];calls=0;leaves=0;lower=99.;worst=None;unresolved=[]
 for i,j in near:
  ds=[] if dep[i]==dep[j] else list(dict.fromkeys(f for f in [dep[i],dep[j]] if f));cache={}
  def distance(vals):
   nonlocal calls
   key=tuple(round(x,10) for x in vals)
   if key in cache:return cache[key]
   objs=[]
   for index in [i,j]:
    m=M[index].copy();f=dep[index]
    if f in ds:
     R=rodrigues(axis[f],-vals[ds.index(f)]);m[:3,:3]=R@m[:3,:3];m[:3,3]=center[f]+R@(m[:3,3]-center[f])
    objs.append(fcl.CollisionObject(check.models[items[index]['asset']],fcl.Transform(m[:3,:3],m[:3,3])))
   hit=bool(fcl.collide(objs[0],objs[1],cq,fcl.CollisionResult()));d=0. if hit else max(0.,float(fcl.distance(objs[0],objs[1],dq,fcl.DistanceResult())));cache[key]=d;calls+=1;return d
  if items[i]['name']=='middle_spread_carrier' and items[j]['name']=='ring_spread_carrier' and eps==3:
   print('TRACE',dep[i],dep[j],rad[i],rad[j],ds,'witness',distance([3,-3]) if len(ds)==2 else None,flush=True)
  if not ds:
   d=distance([]);lower=min(lower,d)
   if d<target-1e-6:return {'certified':False,'reason':'nominal boundary distance below mesh target','pair':[items[i]['name'],items[j]['name']],'sample_gap_mm':d,'calls':calls}
   continue
  stack=[(np.zeros(len(ds)),np.full(len(ds),eps),0)];nodes=0
  while stack:
   c,h,depth=stack.pop();nodes+=1;d=distance(c)
   if d<target-1e-6:return {'certified':False,'reason':'explicit sample below mesh target','pair':[items[i]['name'],items[j]['name']],'spread_error_sample_deg':dict(zip(ds,c.tolist())),'sample_gap_mm':d,'calls':calls}
   motion=0.;impact=np.zeros(len(ds))
   for index in [i,j]:
    if dep[index] in ds:
     k=ds.index(dep[index]);b=2*rad[index]*math.sin(math.radians(h[k])/2);motion+=b;impact[k]+=b
   lb=d-motion
   if lb>=target-1e-6:lower=min(lower,lb);leaves+=1;continue
   if depth>=18 or nodes>12000:
    unresolved.append({'pair':[items[i]['name'],items[j]['name']],'reason':'subdivision budget','lower_mm':lb});break
   k=int(np.argmax(impact));hh=h.copy();hh[k]/=2
   for sign in [-1,1]:cc=c.copy();cc[k]+=sign*hh[k];stack.append((cc,hh.copy(),depth+1))
  if unresolved:break
 return {'certified':not unresolved,'reason':'all continuously bounded pair cells passed' if not unresolved else 'unresolved cells','mesh_boundary_lower_bound_mm':target if not unresolved else None,'smallest_evaluated_cell_lower_mm':lower,'distance_queries':calls,'certified_leaf_cells':leaves,'broad_phase_pair_candidates':len(near),'unresolved':unresolved}
rows=[];start=time.monotonic()
for name,eps in [('Splay',3)]:
 row={'pose':name,'independent_four_spread_errors_deg':[-eps,eps],'other_joints_held_exactly_at_pose':True}|test(C['poses'][name],eps);rows.append(row);print(json.dumps(row),flush=True);(D/'reviews'/'spread_debug.json').write_text(json.dumps({'scope':'Offline geometry of four simultaneous spread errors, all other finger/arm joints exact. Pairwise branch subdivision plus rigid-motion displacement bounds2Rsin(eps/2), with common-root transforms canceled. Not a certified full joint-limit set, actuator error estimate, actual CAD/FDM guarantee or runtime speed claim.','mesh_target_mm':target,'CAD_interpretation':'1.0mm only conditional on0.05mm per-surface tessellation bound; this does not include print tolerance or structural/servo errors.','topology':'Known nonintersecting nominal moving pairs. Continuous positive boundary separation prevents new containment across each connected uncertainty box.','rows':rows,'elapsed_s':time.monotonic()-start,'actuation_approved':False},indent=2),encoding='utf-8')

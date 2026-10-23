"""Non-convex CAD triangle BVHs via FCL; near-limit cases are checked in BRep."""
import numpy as np,json,time,fcl,trimesh
from pathlib import Path
from kinematics_numpy import C,D,META,forward,eligible
class Checker:
 def __init__(self,scope='hand',use_cache=True,overrides=None):
  self.cache={};self.use_cache=use_cache
  data=np.load(D/'collision_meshes.npz');self.models={};self.meshes={};self.points={};self.corners={}
  for name in sorted({it['asset'] for it in META}):
   v=data[name+'_vertices'];f=data[name+'_faces']
   if overrides and name in overrides:
    ov=np.load(overrides[name]);v=ov['vertices'];f=ov['faces']
   b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();self.models[name]=b
   m=trimesh.Trimesh(v,f,process=True);self.meshes[name]=m
   self.points[name]=np.array([x.vertices[0] for x in m.split(only_watertight=False) if len(x.faces)>4])
   lo=v.min(0);hi=v.max(0);self.corners[name]=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
  self.ids=[i for i,it in enumerate(META) if scope=='whole' or not it['role'].startswith('arm_')]
  self.items=[META[i] for i in self.ids];self.objects=[fcl.CollisionObject(self.models[it['asset']]) for it in self.items]
  self.pairs=np.array([(i,j) for i in range(len(self.items)) for j in range(i+1,len(self.items)) if eligible(self.items[i],self.items[j])],dtype=int)
  self.req=fcl.DistanceRequest(enable_nearest_points=True);self.creq=fcl.CollisionRequest(num_max_contacts=1)
 def evaluate(self,pose,arm_q=(0,0,0,0),cap=3.5,threshold=1.1):
  mat=forward(pose,arm_q)[self.ids];lo=[];hi=[]
  for m,it,o in zip(mat,self.items,self.objects):
   o.setTransform(fcl.Transform(m[:3,:3],m[:3,3]))
   p=self.corners[it['asset']]@m[:3,:3].T+m[:3,3];lo.append(p.min(0));hi.append(p.max(0))
  lo=np.array(lo);hi=np.array(hi);ii=self.pairs[:,0];jj=self.pairs[:,1]
  gaps=np.maximum(0,np.maximum(lo[ii]-hi[jj],lo[jj]-hi[ii]));near=self.pairs[np.einsum('ij,ij->i',gaps,gaps)<cap*cap]
  best={'distance_mm':cap,'pair':None,'lower_bound_only':True};hits=[];calls=0;containedcalls=0
  for i,j in near:
   # Rigidly unchanged relative CAD placements have identical distance/containment.
   relrot=mat[i,:3,:3].T@mat[j,:3,:3];relt=mat[i,:3,:3].T@(mat[j,:3,3]-mat[i,:3,3])
   arr=np.round(np.r_[relrot.ravel(),relt],9);arr[arr==0]=0.;key=arr.tobytes();cached=self.cache.get((i,j)) if self.use_cache else None
   if cached is not None and cached[0]==key:
    row=dict(cached[1]);row.pop('nearest_points_mm',None);row['cached_rigid_relative_transform']=True;d=cached[2]
    if d<best['distance_mm']:best=row|{'lower_bound_only':False}
    if d<threshold:hits.append(row)
    continue
   res=fcl.DistanceResult();d=float(fcl.distance(self.objects[i],self.objects[j],self.req,res));calls+=1;colliding=d<0
   if d<=1e-7:
    colliding=bool(fcl.collide(self.objects[i],self.objects[j],self.creq,fcl.CollisionResult()))
    d=0.
   # BVH surface distance alone misses one closed component wholly inside another.
   elif (np.all(lo[i]>=lo[j]-1e-7) and np.all(hi[i]<=hi[j]+1e-7)) or (np.all(lo[j]>=lo[i]-1e-7) and np.all(hi[j]<=hi[i]+1e-7)):
    small,large=(i,j) if np.linalg.norm(hi[i]-lo[i])<np.linalg.norm(hi[j]-lo[j]) else (j,i)
    pts=self.points[self.items[small]['asset']]
    if len(pts):
     pts=pts@mat[small,:3,:3].T+mat[small,:3,3];pts=(pts-mat[large,:3,3])@mat[large,:3,:3]
     containedcalls+=1
     if np.any(self.meshes[self.items[large]['asset']].contains(pts)):colliding=True;d=0.
   row={'pair':[self.items[i]['name'],self.items[j]['name']],'distance_mm':round(d,6),'surface_collision_or_containment':colliding}
   if res.nearest_points is not None:row['nearest_points_mm']=[np.asarray(p).tolist() for p in res.nearest_points]
   if self.use_cache:self.cache[(i,j)]=(key,dict(row),d)
   if d<best['distance_mm']:best=row|{'lower_bound_only':False}
   if d<threshold:hits.append(row)
  return {'minimum':best,'below_threshold':sorted(hits,key=lambda r:(not r['surface_collision_or_containment'],r['distance_mm'])),'distance_calls':calls,'containment_calls':containedcalls}

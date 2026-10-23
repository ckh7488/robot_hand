import numpy as np,json,fcl,trimesh
from kinematics_numpy import D,T,rot
P=np.load(D/'collision_meshes.npz')
def model(v,faces):
 b=fcl.BVHModel();b.beginModel(len(v),len(faces));b.addSubModel(v,faces);b.endModel();return b
cart=model(P['Root_cartridge_vertices'],P['Root_cartridge_faces']);ob=fcl.CollisionObject(cart);req=fcl.CollisionRequest(num_max_contacts=1);out={'claim':'Straight nut insertion into printed cartridge only; final M2 bearing contact is checked separately by exact CAD volume','tests':[]}
for asset in ['M2_base_nuts','M3_root_mount_nuts']:
 mesh=trimesh.Trimesh(P[asset+'_vertices'],P[asset+'_faces'],process=True)
 for k,nu in enumerate(mesh.split(only_watertight=False)):
  b=model(nu.vertices,nu.faces);bad=[]
  for q in np.arange(12,.01,-.25):
   tr=T(r=rot(1,-90))@T((0,0,-q)) if asset=='M2_base_nuts' else T((q,0,0))
   moving=fcl.CollisionObject(b,fcl.Transform(tr[:3,:3],tr[:3,3]))
   if fcl.collide(moving,ob,req,fcl.CollisionResult()):bad.append(float(q))
  row={'asset':asset,'component':k,'start_center_mm':nu.center_mass.tolist(),'colliding_offsets_mm':bad};out['tests'].append(row);print(row,flush=True)
out['passed']=not any(r['colliding_offsets_mm'] for r in out['tests']);(D/'reviews/cartridge_nut_insertion.json').write_text(json.dumps(out,indent=2))

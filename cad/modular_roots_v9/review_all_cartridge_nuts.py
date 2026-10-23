import numpy as np,json,fcl,trimesh,hashlib
from kinematics_numpy import D,C,T,rot
P=np.load(D/'collision_meshes.npz');req=fcl.CollisionRequest(num_max_contacts=1)
def model(v,f):
 b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();return b
out={'scope':'Individual captive nut insertion into unloaded printed cartridges; not complete module insertion. M2 final bearing surface omitted at0 and independently checked in BRep.','tests':[]}
tf=C['hand']['fingers'][-1];B=rot(2,tf['base_rpy'][2]);tp=T(tf['root'],B@rot(1,-90))
for kind,cart,assets in [('finger','Root_cartridge',['M2_base_nuts','M3_root_mount_nuts']),('thumb','Thumb_internal_cradle',['M2_base_nuts','M3_thumb_mount_nuts'])]:
 ob=fcl.CollisionObject(model(P[cart+'_vertices'],P[cart+'_faces']))
 for asset in assets:
  mesh=trimesh.Trimesh(P[asset+'_vertices'],P[asset+'_faces'],process=True)
  for k,nu in enumerate(mesh.split(only_watertight=False)):
   b=model(nu.vertices,nu.faces);bad=[]
   for q in np.arange(12,.01,-.25):
    if asset=='M2_base_nuts':tr=(T(r=rot(1,-90)) if kind=='finger' else tp)@T((0,0,-q))
    else:tr=T((q,0,0)) if kind=='finger' else T(B@np.array([q,0,0]))
    mo=fcl.CollisionObject(b,fcl.Transform(tr[:3,:3],tr[:3,3]))
    if fcl.collide(mo,ob,req,fcl.CollisionResult()):bad.append(float(q))
   row={'module_kind':kind,'asset':asset,'component':k,'colliding_offsets_mm':bad};out['tests'].append(row);print(row,flush=True)
out['passed']=not any(x['colliding_offsets_mm'] for x in out['tests']);out['input_hashes']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'design.json',D/'collision_meshes.npz']};(D/'reviews/all_cartridge_nut_insertion.json').write_text(json.dumps(out,indent=2))

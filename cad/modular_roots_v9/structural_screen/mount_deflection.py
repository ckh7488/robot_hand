import json,numpy as np
from pathlib import Path
from scipy.spatial import cKDTree
F=Path(__file__).resolve().parent;D=F.parent;I=np.load(F/'mesh_index.npz');U=np.load(F/'fields.npz');C=json.loads((D/'design.json').read_text(encoding='utf-8'));xyz=I['xyz'];results=[]
for k,case in enumerate(['palm_normal','finger_direction','sideways']):
 u=U['u_'+str(k)];rows=[]
 for f in C['hand']['fingers']:
  n=f['name']
  if n=='thumb':
   a=np.deg2rad(f['base_rpy'][2]);R=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1.]])
   centers=np.array([np.array([*f['root'][:2],0.])+R@np.array([24,y,-60.5]) for y in [-14,8]])
  else:centers=np.array([[f['root'][0]+25,f['root'][1]+y,-60.5] for y in [-24,4]])
  mask=np.zeros(len(xyz),bool)
  for c in centers:mask|=np.linalg.norm(xyz-c,axis=1)<6
  ref=centers.mean(0);pts=xyz[mask]-ref;A=np.zeros((len(pts)*3,6))
  for i,(x,y,z) in enumerate(pts):A[3*i:3*i+3,:3]=np.eye(3);A[3*i:3*i+3,3:]=[[0,z,-y],[-z,0,x],[y,-x,0]]
  sol=np.linalg.lstsq(A,u[mask].ravel(),rcond=None)[0];res=(A@sol-u[mask].ravel()).reshape(-1,3)
  rows.append({'root':n,'reference_mm':ref.tolist(),'translation_mm':sol[:3].tolist(),'translation_norm_mm':float(np.linalg.norm(sol[:3])),'small_rotation_rad':sol[3:].tolist(),'rotation_norm_deg':float(np.linalg.norm(sol[3:])*180/np.pi),'patch_nonrigid_rms_mm':float(np.sqrt(np.mean(np.sum(res*res,axis=1))))})
 results.append({'case':case,'mounts':rows})
out={'scope':'Least-squares rigid movement of bearing-zone mesh nodes. Palm-only isotropic model; cartridge flex, bolt slip and actuator compliance omitted.','cases':results};(F/'mount_deflection.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))

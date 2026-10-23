"""Whole preassembled hand modules; sampled straight vertical insertion, not motor dynamics."""
import json,numpy as np,fcl,hashlib
from kinematics_numpy import D,C,META,forward
P=np.load(D/'collision_meshes.npz');models={};corners={}
for a in set(it['asset'] for it in META):
 v=P[a+'_vertices'];f=P[a+'_faces'];b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();models[a]=b
 lo=v.min(0);hi=v.max(0);corners[a]=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
def owner(n):
 for f in C['hand']['fingers']:
  k=f['name']
  if n.startswith(k+'_') or n.startswith(k.capitalize()+'_root_'):return k
 if n in ['Thumb_internal_cradle','Palm_thumb_mount_screws','Palm_thumb_mount_nuts']:return 'thumb'
 return None
def ismountscrew(n):return n.endswith('_root_mount_screws') or n=='Palm_thumb_mount_screws'
rows=[]
for it,m in zip(META,forward(C['poses']['Open'])):
 if it['role'].startswith('arm_') or it['name'] in ['Palm_front_cover','Palm_cover_screws','Palm_cover_nuts']:continue
 v=corners[it['asset']]@m[:3,:3].T+m[:3,3]
 rows.append((it,m,fcl.CollisionObject(models[it['asset']],fcl.Transform(m[:3,:3],m[:3,3])),v.min(0),v.max(0)))
order=['little','ring','middle','index','thumb'];installed=[];results=[];req=fcl.CollisionRequest(num_max_contacts=1)
for who in order:
 moving=[r for r in rows if owner(r[0]['name'])==who and not ismountscrew(r[0]['name'])]
 fixed=[r for r in rows if owner(r[0]['name']) is None or owner(r[0]['name']) in installed]
 lo=np.array([r[3] for r in fixed]);hi=np.array([r[4] for r in fixed]);bad=[];calls=0
 height_start=int(np.ceil(max(hi[:,2])-min(r[3][2] for r in moving)+10))
 for height in range(height_start,-1,-1):
  off=np.array([0,0,height])
  for it,m,o,ml,mh in moving:
   o.setTransform(fcl.Transform(m[:3,:3],m[:3,3]+off));ids=np.where(np.all(ml+off<=hi+1e-8,axis=1)&np.all(mh+off>=lo-1e-8,axis=1))[0]
   for j in ids:
    ot,_,oo,_,_=fixed[j]
    if height==0 and ot['name']=='Palm_back_chassis' and (it['name'].endswith('_root_cartridge') or it['name']=='Thumb_internal_cradle'):continue
    calls+=1
    if fcl.collide(o,oo,req,fcl.CollisionResult()):bad.append({'height_mm':height,'moving':it['name'],'obstacle':ot['name']})
 result={'module':who,'previously_installed':installed.copy(),'moving_placements':len(moving),'start_above_seat_mm':height_start,'step_mm':1,'calls':calls,'collisions':bad,'passed':not bad};results.append(result);print(who,'PASS',not bad,'calls',calls,'hits',len(bad),flush=True)
 installed.append(who)
 # Reset FCL objects after every probe; finished modules are seated for the next test.
 for it,m,o,_,_ in moving:o.setTransform(fcl.Transform(m[:3,:3],m[:3,3]))
out={'assembly_order':order,'opening_pose':'Open','cover_fitted_last':True,'mount_screws_fitted_from_below_after_each_module':True,'scope':'Straight vertical insertion of full preassembled modules at1mm samples; no actuator dynamics, cable or hand-tool volume included','final_bearing_surface_contact':'Excluded at height0 only for cartridge vs palm; exact BRep volume must pass independently','input_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'design.json',D/'scene.json',D/'collision_meshes.npz']},'modules':results,'passed':all(x['passed'] for x in results)}
(D/'reviews/module_installation.json').write_text(json.dumps(out,indent=2))

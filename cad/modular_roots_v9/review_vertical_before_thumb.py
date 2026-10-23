import json,numpy as np,fcl
from kinematics_numpy import D,C,META,forward,T
P=np.load(D/'collision_meshes.npz');models={};corners={}
for a in set(it['asset'] for it in META):
 v=P[a+'_vertices'];f=P[a+'_faces'];b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();models[a]=b;lo=v.min(0);hi=v.max(0);corners[a]=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
mm=forward(C['poses']['Open']);moving=[];fixed=[]
for it,m in zip(META,mm):
 n=it['name']
 if n.startswith('thumb_'):continue
 if it['role'].startswith('arm_') or n in ['Palm_front_cover','Palm_cover_screws','Palm_cover_nuts','Index_root_mount_screws']:continue
 o=fcl.CollisionObject(models[it['asset']],fcl.Transform(m[:3,:3],m[:3,3]));v=corners[it['asset']]@m[:3,:3].T+m[:3,3];row=(it,m,o,v.min(0),v.max(0))
 if n.startswith('index_') or n in ['Index_root_cartridge','Index_root_mount_nuts']:moving.append(row)
 else:fixed.append(row)
lo=np.array([r[3] for r in fixed]);hi=np.array([r[4] for r in fixed]);req=fcl.CollisionRequest(num_max_contacts=1);bad=[];calls=0
for height in range(100,-1,-1):
 off=np.array([0,0,height])
 for it,m,o,ml,mh in moving:
  pos=m[:3,3]+off;o.setTransform(fcl.Transform(m[:3,:3],pos));ids=np.where(np.all(ml+off<=hi,axis=1)&np.all(mh+off>=lo,axis=1))[0]
  for j in ids:
   ot,_,oo,_,_=fixed[j]
   if height==0 and it['name']=='Index_root_cartridge' and ot['name']=='Palm_back_chassis':continue
   calls+=1
   if fcl.collide(o,oo,req,fcl.CollisionResult()):bad.append({'height_mm':height,'moving':it['name'],'obstacle':ot['name']})
 print('height',height,'hits',len(bad),flush=True) if height%20==0 else None
out={'motion':'Index module lowers100mm before the thumb actuated module is installed; other three fingers remain present','step_mm':1,'actuator_dynamics':False,'final_cartridge_palm_bearing_contact':'Excluded from surface collision at height0 only; requires exact CAD zero overlap check','distance_calls':calls,'collisions':bad,'passed':not bad};(D/'reviews/index_vertical_insertion_before_thumb.json').write_text(json.dumps(out,indent=2));print('PASS',not bad,'calls',calls,flush=True)

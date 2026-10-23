import json,sys,time,math,itertools,copy
from pathlib import Path
from kinematics_numpy import C,D,verify
from mesh_collision import Checker
sys.stdout.reconfigure(encoding='utf-8')
mode=sys.argv[1] if len(sys.argv)>1 else 'named';scope='whole' if mode=='arm' else 'hand'
out={'mode':mode,'target_physical_clearance_mm':1.0,'mesh_screen_threshold_mm':1.1,'mesh_deflection_mm':.05,'distance_method':'FCL non-convex triangle BVH + containment rays; near limits require exact CAD confirmation','actuator_simulation':False,'continuous_swept_volume_certified':False,'kinematics_check':verify(),'items':[]}
if '--resume' in sys.argv:
 path=D/'reviews'/('mesh_'+mode+'.json')
 if path.exists():out['items']=json.loads(path.read_text()).get('items',[])
 out['resumed_completed_rows']=len(out['items'])
t0=time.monotonic();ck=Checker(scope);print('LOADED',len(ck.items),'parts',len(ck.pairs),'pairs',flush=True)
if mode=='named':
 for n,p in C['poses'].items():
  r=ck.evaluate(p);out['items'].append({'name':n,**r});print(n,r['minimum'],'below',len(r['below_threshold']),flush=True)
elif mode=='transitions':
 for a,b in itertools.combinations(C['poses'],2):
  if any(x.get('from')==a and x.get('to')==b for x in out['items']):continue
  pa,pb=C['poses'][a],C['poses'][b];delta=max(abs(x-y) for n in pa for x,y in zip(pa[n],pb[n]));steps=max(2,math.ceil(delta/.5))
  best=None;bad=[];calls=0
  for i in range(steps+1):
   r=ck.evaluate({n:[x+(y-x)*i/steps for x,y in zip(pa[n],pb[n])] for n in pa},cap=1.3);calls+=r['distance_calls']
   if best is None or r['minimum']['distance_mm']<best['distance_mm']:best=r['minimum']|{'t':i/steps}
   if r['below_threshold']:bad.append({'t':i/steps,'hits':r['below_threshold'][:8]})
  out['items'].append({'from':a,'to':b,'max_joint_step_deg':delta/steps,'samples':steps+1,'minimum':best,'bad_samples':bad,'distance_calls':calls})
  print(a,'->',b,'min',best,'bad',len(bad),flush=True)
  (D/'reviews'/('mesh_'+mode+'.json')).write_text(json.dumps(out,indent=2))
elif mode=='single_axes':
 ranges={'index':[(-4,8),(-20,60),(0,60),(0,45)],'middle':[(-3,3),(-20,60),(0,60),(0,45)],'ring':[(-4,3),(-20,60),(0,60),(0,45)],'little':[(-8,3),(-20,60),(0,60),(0,45)],'thumb':[(-20,20),(0,35),(0,40),(0,30)]}
 for f,rs in ranges.items():
  for j,(lo,hi) in enumerate(rs):
   if any(x.get('finger')==f and x.get('joint_index')==j for x in out['items']):continue
   bad=[];best=None
   for q in range(lo,hi+1):
    pose=copy.deepcopy(C['poses']['Open']);pose[f][j]=q;r=ck.evaluate(pose,cap=1.3)
    if best is None or r['minimum']['distance_mm']<best['distance_mm']:best=r['minimum']|{'angle_deg':q}
    if r['below_threshold']:bad.append({'angle_deg':q,'hits':r['below_threshold'][:8]})
   row={'finger':f,'joint_index':j,'requested_range_deg':[lo,hi],'step_deg':1,'minimum':best,'bad_samples':bad};out['items'].append(row);print(f,j,'min',best,'bad',len(bad),flush=True)
   (D/'reviews'/('mesh_'+mode+'.json')).write_text(json.dumps(out,indent=2))
elif mode=='arm':
 for pitch,yaw in itertools.product(range(-35,56,5),range(-20,21,5)):
  for roll in [-60,0,60]:
   r=ck.evaluate(C['poses']['Open'],arm_q=(0,roll,pitch,yaw),cap=1.3)
   out['items'].append({'arm_q_deg':[0,roll,pitch,yaw],**r})
  if yaw==20:print('Arm pitch',pitch,'samples',len(out['items']),flush=True);(D/'reviews'/('mesh_'+mode+'.json')).write_text(json.dumps(out,indent=2))
out['elapsed_s']=time.monotonic()-t0
out['passed_at_threshold']=not any(x.get('below_threshold') or x.get('bad_samples') for x in out['items'])
(D/'reviews'/('mesh_'+mode+'.json')).write_text(json.dumps(out,indent=2));print('DONE',mode,out['elapsed_s'],out['passed_at_threshold'],flush=True)

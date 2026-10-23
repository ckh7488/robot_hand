import argparse
import sys,json,math,numpy as np,fcl
from pathlib import Path
D=Path(__file__).resolve().parent;_ap=argparse.ArgumentParser();_ap.add_argument('--output',type=Path);_args=_ap.parse_args();B=D.parent/'modular_roots_v9';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward
z=np.load(B/'collision_meshes.npz');ids={x['name']:i for i,x in enumerate(META)};models={}
for n in ['Elbow_fixed_cradle','Elbow_roll_carrier','AK_wrist_flex','Palm_wrist_saddle']:
 a=META[ids[n]]['asset'];v,f=z[a+'_vertices'],z[a+'_faces'];b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();models[n]=b
req=fcl.DistanceRequest();creq=fcl.CollisionRequest();rows=[]
for kind,angles,pair,index in [('elbow',range(0,111),['Elbow_fixed_cradle','Elbow_roll_carrier'],0),('wrist_deviation',range(-20,21),['AK_wrist_flex','Palm_wrist_saddle'],3)]:
 vals=[]
 for angle in angles:
  q=[0,0,0,0];q[index]=angle;M=forward(C['poses']['Open'],q);obs=[]
  for n in pair:m=M[ids[n]];obs.append(fcl.CollisionObject(models[n],fcl.Transform(m[:3,:3],m[:3,3])))
  collision=bool(fcl.collide(*obs,creq,fcl.CollisionResult()));distance=0 if collision else float(fcl.distance(*obs,req,fcl.DistanceResult()));vals.append({'angle_deg':angle,'gap_mm':distance,'collision':collision})
 rows.append({'joint':kind,'pair':pair,'rows':vals})
continuous=[];M0=forward(C['poses']['Open']);arm=C['arm'];ey=-arm['wrist_to_palm']-arm['wrist_axis_offset']-arm['elbow_to_wrist_flex']
for kind,n,origin,axis,lower,upper in [('elbow','Elbow_roll_carrier',[0,ey,0],[1,0,0],0,98),('wrist_deviation','Palm_wrist_saddle',[arm['wrist_lateral_offset'],-arm['wrist_to_palm'],0],[0,0,1],-20,17)]:
 i=ids[n];a=META[i]['asset'];m=M0[i];v=z[a+'_vertices']@m[:3,:3].T+m[:3,3]-np.array(origin);ax=np.array(axis);v-=np.outer(v@ax,ax);r=float(np.linalg.norm(v,axis=1).max())+.05;samplemin=min(x['gap_mm'] for rr in rows if rr['joint']==kind for x in rr['rows'] if lower<=x['angle_deg']<=upper);bound=samplemin-2*r*math.sin(math.radians(.5)/2);continuous.append({'joint':kind,'actual_angle_domain_deg':[lower,upper],'moving_radius_bound_mm':r,'nearest_sample_max_angle_error_deg':.5,'mesh_surface_distance_lower_bound_mm':bound,'CAD_lower_bound_if_0p05mm_tessellation_each_mm':bound-.1,'scope':'Only the named pair, one axis and other angles0; not global arm-space.'})
out={'continuous_pair_bounds':continuous,'scope':'Two known stop pairs only, sampled1deg in nominal arm CAD. Continuous bounds apply only to these two pairs in the stated one-axis domains; no global arm-space or actual vendor flange approval.','rows':rows,'manufacturing_release':False};(_args.output or D/'reviews'/'arm_stop_pairs.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps([{k:v for k,v in x.items() if k!='rows'}|{'selected':[v for v in x['rows'] if v['angle_deg'] in [0,15,16,17,18,19,20,95,97,99,100,101,102,103]]} for x in rows],indent=2))

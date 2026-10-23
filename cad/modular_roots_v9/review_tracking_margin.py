"""Single-joint tracking-error probes; not a simultaneous uncertainty or dynamics proof."""
import json,copy,numpy as np
from kinematics_numpy import D,C
from mesh_collision import Checker
B=D.parent/'modular_roots_v8';old=np.load(B/'collision_meshes.npz');over={}
for a in ['Root_cartridge','Thumb_internal_cradle','Compact_root']:
 p=D/'reviews'/('baseline_'+a+'.npz');np.savez_compressed(p,vertices=old[a+'_vertices'],faces=old[a+'_faces']);over[a]=str(p)
checks=[('V8',Checker('hand',overrides=over)),('V9',Checker('hand'))];rows=[]
for rev,ck in checks:
 for who in ['index','middle','ring','little']:
  for error in [-3,-2,-1,0,1,2,3]:
   pose=copy.deepcopy(C['poses']['Backbend']);pose[who][1]+=error;r=ck.evaluate(pose);rows.append({'revision':rev,'finger':who,'MCP_error_deg':error,**r});print(rev,who,error,'min',r['minimum']['distance_mm'],'bad',len(r['below_threshold']),flush=True)
out={'scope':'Backbend nominal pose with one MCP perturbed at a time by±3deg. Other joints fixed; no simultaneous error, printed tolerance, deflection or calibrated servo model.','rows':rows};(D/'reviews/tracking_margin_probes.json').write_text(json.dumps(out,indent=2))

"""Recompute changed-cover interfaces in baseline hand kinematics, using candidate meshes.
No claims about cable dynamics, every configuration, or motor performance.
"""
import sys,json,copy,time,argparse
from pathlib import Path
import numpy as np,trimesh
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';sys.path.insert(0,str(B))
from kinematics_numpy import C,META,forward,eligible,rigid_body
from mesh_collision import Checker
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path);args=ap.parse_args()
for asset in ['Palm_front_cover_recessed','M3x8_button_cover_screws']:
 x=json.loads((D/'parts'/(asset+'.mesh.json')).read_text());np.savez(D/'reviews'/(asset+'.npz'),vertices=x['vertices'],faces=x['faces'])
# Existing V10/V11/V12 replacements are also supplied, so no obsolete thumb source is tested.
overrides={'Palm_front_cover':D/'reviews/Palm_front_cover_recessed.npz','M3_cover_screws':D/'reviews/M3x8_button_cover_screws.npz'}
for asset,relative in [('Thumb_metacarpal','../audit_fixes_v12/parts/Thumb_metacarpal_tool_access.stl'),('Compact_root','../service_loops_v11/parts/Little_root_cable_saddle.stl')]:
 # The Compact_root is shared by every finger in META, while the V11 guide modifies little only.
 # Do not replace the common asset with the little-only geometry. This test scopes the changed cover.
 if asset=='Compact_root':continue
 m=trimesh.load_mesh(D/relative);np.savez(D/'reviews'/(asset+'.npz'),vertices=m.vertices,faces=m.faces);overrides[asset]=D/'reviews'/(asset+'.npz')
c=Checker(use_cache=True,overrides=overrides);old=Checker(use_cache=True)
changed={'Palm_front_cover','Palm_cover_screws'}
# Only pairs touching either modified object: all other geometry must be separately verified.
keep=np.array([any(c.items[k]['name'] in changed for k in p) for p in c.pairs]);c.pairs=c.pairs[keep];old.pairs=old.pairs[keep]
rows=[]
for name,p in C['poses'].items():
 r=c.evaluate(p,cap=5.);rows.append({'pose':name,'minimum':r['minimum'],'below_threshold':r['below_threshold']});print(name,r['minimum'],flush=True)
pathrows=[];a=C['poses']['Open'];b=C['poses']['Close'];start=time.monotonic()
for t in np.linspace(0,1,401):
 p={f:(np.array(a[f])*(1-t)+np.array(b[f])*t).tolist() for f in a}
 r=c.evaluate(p,cap=5.);pathrows.append({'t':float(t),'minimum':r['minimum'],'below_threshold':r['below_threshold']})
worst=min(pathrows,key=lambda r:r['minimum']['distance_mm']);
# The independent audit used a deeper fist (90,100,80), not the saved Close pose.
fistrows=[]
for t in np.linspace(0,1,401):
 p=copy.deepcopy(a)
 for f in ['index','middle','ring','little']:p[f]=[0.,90*t,100*t,80*t]
 r=c.evaluate(p,cap=5.);fistrows.append({'t':float(t),'minimum':r['minimum'],'below_threshold':r['below_threshold']})
p=copy.deepcopy(a)
for f in ['index','middle','ring','little']:p[f]=[0.,89.1,99.,79.2]
before=old.evaluate(p,cap=5.);after=c.evaluate(p,cap=5.)
# Continuous distance bound for ONLY the changed-cover interfaces on t in [0,.99].
# Four straight fingers have parallel flexion axes. A point's speed is bounded
# by sum(abs(joint_rate)*radius_to_joint), using upstream link lengths + local
# radial norm. Fixed cover/screws have zero speed on this path.
definition={f['name']:f for f in C['hand']['fingers']};maxspeed=0.;speed_source=None
for it in c.items:
 n=it['name'];finger=n.split('_')[0]
 if finger not in ['index','middle','ring','little']:continue
 for stage,tag in enumerate(['mcp','pip','dip']):
  if '_'+tag+'_' not in n:continue
  fixed=n.endswith(('_motor','_fixed_frame')) or '_'+tag+'_base_' in n
  active=stage if fixed else stage+1
  v=c.meshes[it['asset']].vertices;r=float(np.linalg.norm(v[:,1:3],axis=1).max())
  lengths=definition[finger]['links'];speed=sum(np.deg2rad([90.,100.,80.][j])*(sum(lengths[j:stage])+r) for j in range(active))
  if speed>maxspeed:maxspeed=float(speed);speed_source=n
  break
prefix=[r for r in fistrows if r['t']<=.99+1e-12];samplemin=min(r['minimum']['distance_mm'] for r in prefix);lower=samplemin-1e-6-maxspeed*(1/400)/2
continuous={'parameter_range':[0,.99],'distance_Lipschitz_bound_mm_per_t':maxspeed,'speed_bound_source_object':speed_source,'nearest_sample_parameter_radius':.00125,'sampled_minimum_mm':samplemin,'continuous_mesh_distance_lower_bound_mm':lower,'mesh_1_1mm_target_met':bool(lower>=1.1),'scope':'Only these 390 edited-cover moving pairs; not all moving pairs or V10/V11 wires/guides. Actual CAD interpretation also requires the per-surface tessellation error bounds.'}
mesh=trimesh.load_mesh(D/'parts/Palm_front_cover_recessed.stl');assert mesh.is_watertight and len(mesh.split())==1
out={'scope':'Changed cover and cover screw interfaces against baseline META hand moving parts. Includes V12 thumb replacement; additional V10/V11 wires and guide additions are NOT included. Other V12 source geometry retained. No global-space or continuous path certification.','moving_pairs_checked':len(c.pairs),'named_poses':rows,'open_close_samples':401,'open_close_sampled_minimum':worst,'open_close_below_1_1mm_samples':[r for r in pathrows if r['below_threshold']],'audit_fist_definition':'All four fingers [0,90*t,100*t,80*t], thumb held at Open as executed by received e2_range.py (its comment incorrectly says thumb Close).','audit_fist_samples':401,'audit_fist_profile':[{'t':r['t'],'minimum_mm':r['minimum']['distance_mm'],'pair':r['minimum']['pair']} for r in fistrows],'partial_path_continuous_bound':continuous,'audit_fist_sampled_minimum':min(fistrows,key=lambda r:r['minimum']['distance_mm']),'audit_fist_below_1_1mm_samples':[r for r in fistrows if r['below_threshold']],'audit_t0_99_before':before,'audit_t0_99_after':after,'cover_STL_closed_single_component':True,'mesh_deflection_request_mm':.03,'elapsed_s':time.monotonic()-start,'manufacturing_release':False}
(args.output or D/'reviews/cover_motion.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in out.items() if k not in ['named_poses','open_close_below_1_1mm_samples','audit_fist_below_1_1mm_samples','audit_fist_profile']},indent=2),flush=True)

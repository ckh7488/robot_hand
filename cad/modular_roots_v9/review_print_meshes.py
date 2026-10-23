"""Mesh integrity and candidate print orientation, not a print strength certification."""
import json,numpy as np,trimesh
from pathlib import Path
from kinematics_numpy import D,rot
S=json.loads((D/'scene.json').read_text(encoding='utf-8'))['Open'];assets=sorted(set(it['asset'] for it in S if it['role']=='printed'));out={'units':'mm','material_and_layer_strength_verified':False,'parts':[]};dest=D/'print_review';dest.mkdir(exist_ok=True)
for n in assets:
 m=trimesh.load_mesh(D/'parts'/(n+'.stl'));vol0=m.volume
 R=rot(0,180) if n=='Palm_front_cover' else rot(1,-90) if n=='Root_cartridge' else rot(1,-90)@rot(2,50) if n=='Thumb_internal_cradle' else rot(1,90) if n=='Compact_root' else np.eye(3)
 t=np.eye(4);t[:3,:3]=R;m.apply_transform(t);m.apply_translation(-m.bounds[0]);q=dest/(n+'_oriented_mm.stl');m.export(q)
 ext=m.extents.tolist();row={'asset':n,'quantity':sum(it['asset']==n for it in S),'watertight':bool(m.is_watertight),'winding_consistent':bool(m.is_winding_consistent),'connected_mesh_components':len(m.split(only_watertight=False)),'bbox_mm':ext,'volume_mm3':float(m.volume),'rigid_orientation_volume_relative_error':float(abs(m.volume-vol0)/max(1,abs(vol0))),'candidate_orientation':'outer cover face down' if n=='Palm_front_cover' else 'broad root wall face down; support and layer review pending' if n in ['Root_cartridge','Thumb_internal_cradle'] else 'C profile laid on its side' if n=='Compact_root' else 'modeled base plane down','fits_256_square_by_bbox':ext[0]<=246 and ext[1]<=246,'fits_220_square_by_bbox':ext[0]<=210 and ext[1]<=210,'notes':'Bounding-box fit includes10mm total brim/edge allowance; actual printer, support, layer direction and tolerances require slicer review.'};out['parts'].append(row);print(n,'watertight',m.is_watertight,'components',row['connected_mesh_components'],'bbox',ext,flush=True)
out['all_watertight']=all(x['watertight'] and x['winding_consistent'] and x['connected_mesh_components']==1 for x in out['parts']);(D/'reviews/print_mesh_review.json').write_text(json.dumps(out,indent=2))

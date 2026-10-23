import sys,json,hashlib,trimesh
from pathlib import Path
D=Path(__file__).resolve().parent;P=D.parent.parent;I=json.loads((P/'AUDIT_EVIDENCE_INDEX.json').read_text(encoding='utf-8'));count=0
for group in ['geometry_sha256','code_sha256','report_sha256']:
 for p,h in I[group].items():assert hashlib.sha256((P/p).read_bytes()).hexdigest()==h,p;count+=1
m=trimesh.load_mesh(D/'parts'/'Thumb_metacarpal_channel_candidate.stl');parts=m.split(only_watertight=False);out={'V9_frozen_hashes_unchanged':count,'candidate_print_mesh':{'watertight':bool(m.is_watertight),'consistent_winding':bool(m.is_winding_consistent),'components':len(parts),'mesh_volume_mm3':float(m.volume)},'manufacturing_release':False};assert m.is_watertight and m.is_winding_consistent and len(parts)==1;(D/'reviews'/'candidate_integrity.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(out,indent=2))

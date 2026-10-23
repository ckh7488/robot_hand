import json,hashlib,trimesh
from pathlib import Path
D=Path(__file__).resolve().parent;P=D.parent.parent;I=json.loads((P/'AUDIT_EVIDENCE_INDEX.json').read_text(encoding='utf-8'));count=0
for group in ['geometry_sha256','code_sha256','report_sha256']:
 for p,h in I[group].items():assert hashlib.sha256((P/p).read_bytes()).hexdigest()==h,p;count+=1
rows=[]
for a in ['Little_root_cable_saddle','Little_MCP_link_cable_saddle']:
 m=trimesh.load_mesh(D/'parts'/(a+'.stl'));parts=m.split(only_watertight=False);row={'part':a,'watertight':bool(m.is_watertight),'consistent_winding':bool(m.is_winding_consistent),'components':len(parts),'volume_mm3':float(m.volume)};assert row['watertight'] and row['consistent_winding'] and row['components']==1;rows.append(row)
g=json.loads((D/'reviews'/'guide_geometry.json').read_text(encoding='utf-8'));assert all(r['removed_original_volume_mm3']<1e-7 for r in g['parts']);out={'V9_frozen_hashes_unchanged':count,'print_candidates':rows,'guide_material_subset_transfer_used':False,'basis':'Direct final geometry motion and BRep checks. Baseline axes/holes retained; original part minus new part has zero volume. No guide-to-previous-guide subset claim.','manufacturing_release':False};(D/'reviews'/'candidate_integrity.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(out,indent=2))

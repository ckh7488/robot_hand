"""Transfer sampled V8 collision evidence only when every changed solid is a material subset.
Kinematics and placements must be identical. Not a strength or actuator-dynamics transfer.
"""
import sys,json,hashlib
from pathlib import Path
import FreeCAD as A,Part
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v8';rd=lambda p:json.loads(p.read_text(encoding='utf-8'));old=rd(B/'design.json');new=rd(D/'design.json');baseline=rd(B/'reviews/mesh_transitions.json');assert baseline['passed_at_threshold'] and len(baseline['items'])==15
for name,digest in baseline['input_sha256'].items():assert hashlib.sha256((B/name).read_bytes()).hexdigest()==digest,'Stale baseline '+name
for field in ['hand','arm','poses','handedness']:assert old[field]==new[field],field
assert rd(B/'scene.json')==rd(D/'scene.json'),'placements changed'
changed=[];identical=[];records={};dest=D/'baseline_evidence';dest.mkdir(exist_ok=True)
for n in rd(B/'cad_manifest.json')['assets']:
 p=B/'parts'/(n+'.step');q=D/'parts'/(n+'.step');hp=hashlib.sha256(p.read_bytes()).hexdigest();hq=hashlib.sha256(q.read_bytes()).hexdigest();records[n]={'old_sha256':hp,'new_sha256':hq}
 if hp==hq:identical.append(n);continue
 s=Part.read(str(p));t=Part.read(str(q));added=t.cut(s).Volume;removed=s.cut(t).Volume
 row={'asset':n,'new_material_outside_old_mm3':added,'removed_material_mm3':removed,'new_valid':t.isValid(),'new_solids':len(t.Solids),'subset_passed':added<.001 and removed>0 and t.isValid() and len(t.Solids)==1};changed.append(row);print(row,flush=True)
 (dest/(n+'_V8.step')).write_bytes(p.read_bytes())
assert {r['asset'] for r in changed}=={'Root_cartridge','Thumb_internal_cradle','Compact_root'}
out={'claim':'New fixed root CAD solids are subsets of V8 at identical placements, so removing material cannot introduce interference or reduce physical separation along unchanged rigid kinematics.','baseline':{'revision':8,'scene_sha256':hashlib.sha256((B/'scene.json').read_bytes()).hexdigest(),'motion_evidence_sha256':hashlib.sha256((B/'reviews/mesh_transitions.json').read_bytes()).hexdigest(),'paths':15,'maximum_joint_sample_step_deg':.5,'minimum_certified_sample_clearance_mm':min(x['minimum']['distance_mm'] for x in baseline['items'])},'identical_assets':identical,'changed_assets':changed,'asset_hashes':records,'placements_and_kinematics_identical':True,'nominal_motion_clearance_transfer_passed':all(r['subset_passed'] for r in changed),'not_transferred':['Stiffness and printed strength','Fastener retention/thread engagement','Motor dynamics or tracking error','Collision safety for unsampled or arbitrary input trajectories']}
(D/'reviews/subset_transfer.json').write_text(json.dumps(out,indent=2));(dest/'V8_mesh_transitions.json').write_text(json.dumps(baseline,indent=2));(dest/'V8_design.json').write_text(json.dumps(old,indent=2));print('TRANSFER',out['nominal_motion_clearance_transfer_passed'],flush=True)

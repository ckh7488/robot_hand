"""Read-only frozen evidence checks. No CAD or results are modified."""
import argparse,hashlib,json
from pathlib import Path
P=Path(__file__).resolve().parent;D=P/'cad'/'modular_roots_v9';B=P/'cad'/'modular_roots_v8';rd=lambda p:json.loads(p.read_text(encoding='utf-8'))
ap=argparse.ArgumentParser();ap.add_argument('--full',action='store_true',help='Also verify frozen report hashes; re-runs can change timing fields.');a=ap.parse_args();I=rd(P/'AUDIT_EVIDENCE_INDEX.json');n=0
for group in ['geometry_sha256','code_sha256']+(['report_sha256'] if a.full else []):
 for name,want in I[group].items():
  assert hashlib.sha256((P/name).read_bytes()).hexdigest()==want,'Changed or missing: '+name;n+=1
G=rd(D/'geometry_review.json');assert G['sampled_poses_passed'] and G['all_printed_hand_connected']
for name,want in G['cad_inputs_sha256'].items():assert hashlib.sha256((D/name).read_bytes()).hexdigest()==want,'Stale geometry report: '+name
for p,key in [('reviews/hand_fasteners.json','passed'),('reviews/all_cartridge_nut_insertion.json','passed'),('reviews/module_installation.json','passed'),('reviews/viewer_cad_alignment.json','passed'),('reviews/print_mesh_review.json','all_watertight'),('reviews/mesh_named.json','passed_at_threshold'),('reviews/subset_transfer.json','nominal_motion_clearance_transfer_passed')]:assert rd(D/p)[key],p
proof=rd(D/'reviews/subset_transfer.json');base=rd(B/'reviews/mesh_transitions.json');assert len(base['items'])==15 and base['passed_at_threshold'];assert proof['baseline']['motion_evidence_sha256']==hashlib.sha256((B/'reviews/mesh_transitions.json').read_bytes()).hexdigest()
for name,want in base['input_sha256'].items():assert hashlib.sha256((B/name).read_bytes()).hexdigest()==want,'Stale V8 baseline: '+name
for field in ['hand','arm','poses','handedness']:assert rd(B/'design.json')[field]==rd(D/'design.json')[field]
assert rd(B/'scene.json')==rd(D/'scene.json')
for name,r in proof['asset_hashes'].items():
 assert hashlib.sha256((B/'parts'/(name+'.step')).read_bytes()).hexdigest()==r['old_sha256']
 assert hashlib.sha256((D/'parts'/(name+'.step')).read_bytes()).hexdigest()==r['new_sha256']
V=rd(D/'reviews/viewer_cad_alignment.json');assert V['dynamic_bodies']==V['actuators']==0 and not V['claims_motor_response'];assert not rd(D/'reviews/arm_gravity_grid.json')['actuator_selection_approved']
print(json.dumps({'frozen_hashes_checked':n,'reported_checks_consistent':True,'BRep_subset_requires_FreeCAD_rerun_for_independent_proof':True,'manufacturing_release':False,'motor_performance_validated':False},indent=2))

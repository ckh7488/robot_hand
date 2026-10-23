"""Fail-closed membership in the limited V9 simultaneous-spread evidence domains.
This is OFFLINE. It never approves actuation or a transition, and does not cover V13.
"""
import argparse,hashlib,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def number(x):return not isinstance(x,bool) and isinstance(x,(int,float)) and math.isfinite(x)
def check_request(request,root=ROOT):
 result={'offline_geometry_domain_matched':False,'physical_actuation_approved':False,'continuous_path_approved':False,'geometry_basis':'V9 only','reasons':[]}
 try:
  limits=json.loads((root/'safe_joint_limits.json').read_text(encoding='utf-8'))
  for item in limits['evidence']:
   p=(root/item['path']).resolve()
   if not p.is_relative_to(root.resolve()) or hashlib.sha256(p.read_bytes()).hexdigest()!=item['sha256']:raise ValueError('Evidence missing or changed: '+item['path'])
  if limits['physical_actuation_approved'] or limits['continuous_paths_approved']:raise ValueError('Unsupported approval flags in contract')
  proof=json.loads((root/'cad/audit_fixes_v12/reviews/spread_neighborhoods.json').read_text(encoding='utf-8'))
  for domain in limits['domains']:
   eps=domain['simultaneous_four_spread_halfwidth_deg']
   if not any(row.get('pose')==domain['pose'] and row.get('certified') is True and row.get('other_joints_held_exactly_at_pose') is True and row.get('independent_four_spread_errors_deg')==[-eps,eps] for row in proof['rows']):raise ValueError('Contract domain exceeds its actual evidence')
  required={'geometry_basis','finger_pose_deg','arm_deg','spread_error_bound_deg','other_joint_error_bound_deg'}
  if not isinstance(request,dict) or set(request)!=required:raise ValueError('Request fields must exactly match the schema')
  if request['geometry_basis']!='V9':raise ValueError('Only V9 spread-domain evidence is available; later candidate geometry is not certified')
  fingers=limits['fingers'];q=request['finger_pose_deg']
  if not isinstance(q,dict) or set(q)!=set(fingers):raise ValueError('All five finger arrays must be present')
  if any(not isinstance(q[f],list) or len(q[f])!=4 or not all(number(v) for v in q[f]) for f in fingers):raise ValueError('Each finger needs four finite numeric joint angles')
  arm=request['arm_deg']
  if not isinstance(arm,list) or len(arm)!=4 or not all(number(v) for v in arm):raise ValueError('Arm needs four finite angles')
  if arm!=limits['required_arm_deg']:raise ValueError('Coupled arm domain is not available')
  e=request['spread_error_bound_deg'];other=request['other_joint_error_bound_deg']
  if not number(e) or e<0 or not number(other) or other<0:raise ValueError('Error bounds must be finite and nonnegative')
  if other!=0:raise ValueError('Non-spread uncertainty has not been covered')
  design=json.loads((root/'cad/modular_roots_v9/design.json').read_text(encoding='utf-8'))
  for domain in limits['domains']:
   center=design['poses'][domain['pose']];eps=domain['simultaneous_four_spread_halfwidth_deg']
   # Four simultaneous coordinates live in one proven error box. Each command offset
   # consumes its share of the box before adding the entire uncertainty interval.
   if any(q[f][1:]!=center[f][1:] for f in fingers) or q['thumb']!=center['thumb']:continue
   if all(abs(q[f][0]-center[f][0])+e<=eps+1e-12 for f in fingers if f!='thumb'):
    result.update(offline_geometry_domain_matched=True,evidence_pose=domain['pose'],spread_halfwidth_deg=eps)
    return result
  raise ValueError('Outside every coupled spread-domain box; no independent-axis clipping is applied')
 except (ValueError,OSError,KeyError,TypeError,json.JSONDecodeError) as error:
  result['reasons'].append(str(error));return result
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('request',type=Path);a=ap.parse_args()
 r=check_request(json.loads(a.request.read_text(encoding='utf-8')));print(json.dumps(r,indent=2))
 raise SystemExit(0 if r['offline_geometry_domain_matched'] else 2)

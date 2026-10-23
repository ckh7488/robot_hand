"""Exact single-hinge chord extrema of proposed fixed-exit endpoints. Not cable lengths or cable simulation."""
import sys,json,copy,math,numpy as np
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward
selected={r['motor']:r for r in json.loads((D/'selected_routes.json').read_text(encoding='utf-8'))['selected']};ids={it['name']:i for i,it in enumerate(META)}
def endpoint(template):
 side=-1 if template.startswith('negativeZ') else 1;stub=3 if 'short3' in template else 5 if 'short5' in template else 8.9;xb=-25.6-stub;y=-9.;z=8*side
 if 'outboard' in template:return np.array([xb-10,y,side*36.])
 if 'forward' in template:return np.array([xb-12.5,25.,z])
 return np.array([xb-12.5,-36.,z])
def end(pose,motor):
 m=forward(pose)[ids[motor]];return m[:3,:3]@endpoint(selected[motor]['template'])+m[:3,3]
ranges={'root':{'index':[-4,8],'middle':[-3,3],'ring':[-4,3],'little':[-8,3],'thumb':[-20,20]},'mcp':[-20,60],'pip':[0,60]};rows=[];maxfit=0.
for f in C['hand']['fingers']:
 who=f['name']
 for j,(up,down) in enumerate([('root','mcp'),('mcp','pip'),('pip','dip')]):
  a=who+'_'+up+'_motor';b=who+'_'+down+'_motor';lo,hi=ranges['root'][who] if j==0 else([0,35] if who=='thumb' and j==1 else[0,40] if who=='thumb' else ranges[up]);pose=copy.deepcopy(C['poses']['Open'])
  def sq(q):
   pose[who][j]=float(q);pa=end(pose,a);pb=end(pose,b);return float(np.sum((pb-pa)**2))
  q0,q90,q180=sq(0),sq(90),sq(180);K=(q0+q180)/2;A=(q0-q180)/2;Bv=q90-K
  crit=math.degrees(math.atan2(Bv,A));qs=[lo,hi]+[crit+180*k for k in range(-3,4) if lo<crit+180*k<hi];ds=[math.sqrt(max(0,K+A*math.cos(math.radians(q))+Bv*math.sin(math.radians(q)))) for q in qs]
  fit=max([abs(sq(float(q))-(K+A*np.cos(np.deg2rad(q))+Bv*np.sin(np.deg2rad(q)))) for q in np.linspace(lo,hi,41)]);maxfit=max(maxfit,fit);assert fit<1e-7
  imin=int(np.argmin(ds));imax=int(np.argmax(ds));rows.append({'finger':who,'rotating_joint':up,'endpoint_motors':[a,b],'range_deg':[lo,hi],'minimum_chord_mm':ds[imin],'minimum_at_deg':qs[imin],'maximum_chord_mm':ds[imax],'maximum_at_deg':qs[imax],'chord_excursion_mm':ds[imax]-ds[imin],'distance_squared_form_mm2':{'constant':K,'cos':A,'sin':Bv},'upstream_template':selected[a]['template'],'downstream_template':selected[b]['template']})
out={'scope':'Geometric necessary lower bounds between center loose ends of V11 candidate fixed exit templates. Proposed root→MCP→PIP→DIP connectivity only, not an approved electrical topology or a connected routed cable. Each pair varies one hinge; other upstream joints are common rigid transforms.','limits_scope':'Explicit per-joint exploration intervals, not complete safe tracking limits.','method':'Squared chord=K+Acos(q)+Bsin(q); extrema at endpoints and analytic stationary angles.0/90/180degree basis evaluations are mathematical fits, not safe poses.41in-range samples cross-check formula.','maximum_formula_error_mm2':maxfit,'endpoint_units':'mm','electrical_topology_approved':False,'full_harness_validated':False,'notes':['Wire length must exceed the maximum chord; finite bend radius, fixed tangents, guides, slack and obstacle routing require additional length.','Do not turn chord excursion into automatic cable-length allowance: slack changes shape and may collide.','One exit per motor requires a tap/star harness for intermediate nodes. Direct port-to-port chaining requires a second exit, not checked here.'],'pairs':rows};(D/'reviews'/'candidate_free_span_budget.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(rows,indent=2))

"""Sampled nominal arm packaging and CAD round-trip, not strength or dynamics."""
import FreeCAD as A
import Part,json,sys,time,hashlib
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent
C=json.loads((D/'design.json').read_text(encoding='utf-8'));J=json.loads((D/'cad_manifest.json').read_text(encoding='utf-8'));S=json.loads((D/'scene.json').read_text(encoding='utf-8'))['Open'];V=A.Vector
assets={n:Part.read(str(D/'parts'/(n+'.step'))) for n in J['assets']}
def itshape(it):
 m=A.Matrix()
 for i in range(3):
  for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),it['R'][i][j])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 s=assets[it['asset']].copy();s.Placement=A.Placement(m).multiply(s.Placement);return s
def about(p,axis,deg):
 t=A.Placement(V(*p),A.Rotation(V(*axis),deg));return t.multiply(A.Placement(-V(*p),A.Rotation()))
def move(s,t):
 s=s.copy();s.Placement=t.multiply(s.Placement);return s
wdev=-C['arm']['wrist_to_palm'];wflex=wdev-C['arm']['wrist_axis_offset'];lat=C['arm']['wrist_lateral_offset']
sel=[it for it in S if it['role'].startswith('arm_') or it['name'] in ['Palm_back_chassis','Palm_front_cover','Thumb_internal_cradle']];sh={it['name']:itshape(it) for it in sel}
inputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'design.json',D/'hand_forearm_v9.step',D/'hand_forearm_v9.FCStd']}
res={'cad_inputs_sha256':inputs,'manufacturing_release':False,'scope':'Finite BRep intersections of nominal arm motor cylinders and structural CAD, plus rigid palm. Does not verify vendor flange bolts, bearing ratings, or cables.','samples':[]}
for pitch,yaw in [(0,0),(-35,0),(55,0),(0,-20),(0,20),(-35,-20),(-35,20),(55,-20),(55,20)]:
 P=about((0,wflex,0),(1,0,0),pitch);Y=about((lat,wdev,0),(0,0,1),yaw);shapes=[]
 for it in sel:
  n=it['name'];t=A.Placement()
  if n in ['Wrist_box_carrier','AK_wrist_deviation']:t=P
  if n in ['Palm_wrist_saddle','Palm_back_chassis','Palm_front_cover','Thumb_internal_cradle']:t=P.multiply(Y)
  shapes.append(move(sh[n],t))
 bad=[]
 for i in range(len(sel)):
  for j in range(i+1,len(sel)):
   a,b=sel[i],sel[j];s,t=shapes[i],shapes[j]
   if not s.BoundBox.intersect(t.BoundBox):continue
   volume=s.common(t).Volume
   if volume>.01:bad.append({'pair':[a['name'],b['name']],'intersection_mm3':round(volume,5)})
 res['samples'].append({'pitch_deg':pitch,'yaw_deg':yaw,'interference':bad});print(pitch,yaw,'overlaps',len(bad),bad,flush=True)
res['sampled_nominal_arm_passed']=not any(x['interference'] for x in res['samples'])
step=Part.read(str(D/'hand_forearm_v9.step'));doc=A.openDocument(str(D/'hand_forearm_v9.FCStd'));ss=[o.Shape for o in doc.Objects if hasattr(o,'Shape')];native=Part.makeCompound(ss)
res['roundtrip']={'step_valid':step.isValid(),'native_all_valid':all(s.isValid() for s in ss),'step_solids':len(step.Solids),'native_solids':len(native.Solids),'native_parts':len(ss),'volume_relative_error':abs(step.Volume-native.Volume)/native.Volume,'bbox_error_mm':max(abs(a-b) for a,b in zip([step.BoundBox.XMin,step.BoundBox.XMax,step.BoundBox.YMin,step.BoundBox.YMax,step.BoundBox.ZMin,step.BoundBox.ZMax],[native.BoundBox.XMin,native.BoundBox.XMax,native.BoundBox.YMin,native.BoundBox.YMax,native.BoundBox.ZMin,native.BoundBox.ZMax]))}
res['roundtrip']['passed']=res['roundtrip']['step_valid'] and res['roundtrip']['native_all_valid'] and res['roundtrip']['step_solids']==res['roundtrip']['native_solids'] and res['roundtrip']['volume_relative_error']<1e-5 and res['roundtrip']['bbox_error_mm']<.001
(D/'arm_packaging_review.json').write_text(json.dumps(res,indent=2),encoding='utf-8');print('RESULT',res['sampled_nominal_arm_passed'],res['roundtrip'],flush=True)

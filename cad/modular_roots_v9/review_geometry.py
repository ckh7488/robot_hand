"""BRep interference review of newly printed hand parts. No dynamic claims."""
import FreeCAD as A
import Part,json,time,sys,hashlib
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent
S=json.loads((D/'scene.json').read_text(encoding='utf-8'));C=json.loads((D/'cad_manifest.json').read_text(encoding='utf-8'))
assets={n:Part.read(str(D/'parts'/(n+'.step'))) for n in C['assets']}
def placed(s,it):
 m=A.Matrix()
 for i in range(3):
  for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),it['R'][i][j])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 t=s.copy();t.Placement=A.Placement(m).multiply(t.Placement);return t
inputs={str(p.relative_to(D)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'design.json',D/'scene.json',*sorted((D/'parts').glob('*.step'))]}
out={'cad_inputs_sha256':inputs,'manufacturing_release':False,'scope':'Actual BRep volume intersections: new printed hand parts versus all hand parts, including same rigid body. Six sampled poses only; no swept paths or continuous limits certified. Arm flange and wiring movement not validated.','cad_connected_printed_parts':{},'poses':{}}
for n,s in assets.items():
 if n.startswith(('M2_','M3_','AK_','Elbow_','Forearm_','Roll_','Wrist_','Palm_wrist_','M077_','H101_','S101_')):continue
 out['cad_connected_printed_parts'][n]={'valid':s.isValid(),'solids':len(s.Solids),'one_connected_solid':len(s.Solids)==1}
for pose,items in S.items():
 items=[it for it in items if not it['role'].startswith('arm_')]
 shapes=[placed(assets[it['asset']],it) for it in items];bad=[];calls=0;t=time.monotonic()
 for i in range(len(items)):
  for j in range(i+1,len(items)):
   a,b=items[i],items[j]
   if a['role']!='printed' and b['role']!='printed':continue
   x,y=shapes[i],shapes[j]
   if not x.BoundBox.intersect(y.BoundBox):continue
   calls+=1;common=x.common(y);vol=common.Volume
   if vol>.01:bad.append({'pair':[a['name'],b['name']],'intersection_mm3':round(vol,5)})
 print(pose,'BRep pairs',calls,'overlaps',len(bad),'seconds',round(time.monotonic()-t,1),flush=True)
 out['poses'][pose]={'interference':sorted(bad,key=lambda x:-x['intersection_mm3']),'candidate_pairs':calls,'printed_part_interference_free':not bad}
 (D/'geometry_review.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
out['all_printed_hand_connected']=all(x['one_connected_solid'] and x['valid'] for x in out['cad_connected_printed_parts'].values())
out['sampled_poses_passed']=all(x['printed_part_interference_free'] for x in out['poses'].values())
(D/'geometry_review.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print('RESULT',out['sampled_poses_passed'],'connected',out['all_printed_hand_connected'],flush=True)

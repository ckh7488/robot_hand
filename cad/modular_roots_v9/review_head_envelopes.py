"""Hypothetical screw-head envelopes. Not a purchased fastener drawing."""
import sys,json,math
from pathlib import Path
import FreeCAD as A,Part
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent;V=A.Vector
ss={n:Part.read(str(D/'parts'/(n+'.step'))) for n in ['S101_relief','M077_stator','H101_vendor','Compact_root']};a=math.sqrt(32);base=[(sx*a-14.5,-7.5+sy*(7.5+a)) for sx in [-1,1] for sy in [-1,1]];out=[(sx*a-14.5,sz*a) for sx in [-1,1] for sz in [-1,1]];rows=[]
for dia,h in [(3,.6),(3.5,.6),(3.5,1.3),(3.8,1.6),(4,1.6)]:
 for kind in ['base','output']:
  bad=[]
  for k,p in enumerate(base if kind=='base' else out):
   head=Part.makeCylinder(dia/2,h,V(p[0],p[1],-11.8)) if kind=='base' else Part.makeCylinder(dia/2,h,V(p[0],15.8,p[1]),V(0,-1,0))
   for n in (['S101_relief','M077_stator'] if kind=='base' else ['H101_vendor','M077_stator']):
    vol=head.common(ss[n]).Volume
    if vol>.001:bad.append({'screw':k,'obstacle':n,'volume_mm3':vol})
  rows.append({'kind':kind,'diameter_mm':dia,'height_mm':h,'collisions':bad,'passed':not bad});print(rows[-1],flush=True)
(D/'reviews/screw_head_envelopes.json').write_text(json.dumps({'status':'Parametric cylindrical head envelope probe only. ROBOTIS product pages verify PHS M2 screw families but do not provide these head dimensions. No exact off-the-shelf part confirmed.','tests':rows},indent=2))

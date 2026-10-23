import FreeCAD as A
import Part,json,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8')
D=Path(__file__).resolve().parent
out={}
for name in ['AK70','AK60','AK45']:
 s=Part.read(str(D/(name+'.step')));bb=s.BoundBox
 row={'valid':s.isValid(),'solids':len(s.Solids),'bbox':[bb.XMin,bb.XMax,bb.YMin,bb.YMax,bb.ZMin,bb.ZMax],'solid_stats':[],'cylinders':[]}
 for i,t in enumerate(s.Solids):
  b=t.BoundBox
  row['solid_stats'].append({'index':i,'volume':t.Volume,'com':list(t.CenterOfMass),'bbox':[b.XMin,b.XMax,b.YMin,b.YMax,b.ZMin,b.ZMax]})
 for i,f in enumerate(s.Faces):
  if isinstance(f.Surface,Part.Cylinder):
   c=f.Surface;row['cylinders'].append({'face':i,'radius':c.Radius,'axis':list(c.Axis),'center':list(c.Center),'area':f.Area})
 out[name]=row
 print(name, 'valid',row['valid'],'solids',row['solids'],'bounds',row['bbox'],flush=True)
 print('large solids',sorted(row['solid_stats'],key=lambda x:-x['volume'])[:8],flush=True)
(D/'vendor_geometry.json').write_text(json.dumps(out,indent=2))

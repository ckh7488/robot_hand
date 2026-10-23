import FreeCAD as A
import Part,json,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent
result={}
for name in ['AK70','AK60','AK45']:
 src=Part.read(str(D/(name+'.step')))
 s=src.copy();s.sewShape()
 print(name,'whole sew',s.ShapeType,len(s.Shells),[(len(x.Faces),x.isClosed()) for x in s.Shells],flush=True)
 solids=[]
 for sh in s.Shells:
  if sh.isClosed():
   q=Part.makeSolid(sh)
   if q.Volume<0:q.reverse()
   if q.isValid() and q.Volume>1e-3:solids.append(q)
 result[name]={'sewn_valid':s.isValid(),'closed_solids':len(solids),'volumes':[x.Volume for x in solids],'source_faces':len(src.Faces),'sewn_faces':len(s.Faces)}
 if solids:Part.makeCompound(solids).exportStep(str(D/(name+'_sewn.step')))
(D/'vendor_sewing.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2),flush=True)

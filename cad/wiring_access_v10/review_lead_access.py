"""Exact BRep collisions of inherited fixed-lead envelopes with complete V9 hand."""
import FreeCAD as A,Part,json,sys,hashlib,time
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8')
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';P=D.parent.parent
sys.path.insert(0,str(P/'cad'/'index_joint_v1'));from wire_harness import wire_shape
V=A.Vector;spec=json.loads((D/'connector_reference.json').read_text(encoding='utf-8'));sc=json.loads((B/'scene.json').read_text(encoding='utf-8'));M=json.loads((B/'cad_manifest.json').read_text(encoding='utf-8'))
assets={n:Part.read(str(B/'parts'/(n+'.step'))) for n in M['assets']};c=spec['connector'];plug=Part.makeBox(*c['registered_outer_size_mm'],V(*c['registered_outer_origin_mm']));plug.translate(V(-14.5,0,0))
wire=[]
for i,y in enumerate(spec['wire']['start_y_mm']):
 s=wire_shape(y,spec,.3);s.translate(V(-14.5,0,0));wire.append(s);s.exportStep(str(D/'parts'/('lead_reserve_'+str(i+1)+'.step')))
plug.exportStep(str(D/'parts'/'EHR3_catalog_envelope.step'))
def placed(s,it):
 m=A.Matrix()
 for i in range(3):
  for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),it['R'][i][j])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 t=s.copy();t.Placement=A.Placement(m).multiply(t.Placement);return t
def overlaps(a,b):
 x,y=a.BoundBox,b.BoundBox
 return not(x.XMax<y.XMin or y.XMax<x.XMin or x.YMax<y.YMin or y.YMax<x.YMin or x.ZMax<y.ZMin or y.ZMax<x.ZMin)
out={'scope':'Inherited single-joint fixed lead paths transformed to actual V9 motor frames. A collision rejects this particular inherited route, not every possible harness. Connector is catalog outer envelope, mating fit unqualified.','wire_outer_diameter_mm':1.9,'radial_reserve_mm':.3,'canonical_old_to_new_translation_mm':[-14.5,0,0],'source_sha256':{p.relative_to(P).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [B/'design.json',B/'scene.json',D/'connector_reference.json',*sorted((B/'parts').glob('*.step'))]},'poses':[],'geometry_modified':False,'manufacturing_release':False,'full_harness_validated':False}
for pn,items in sc.items():
 items=[it for it in items if not it['role'].startswith('arm_')];ps=[placed(assets[it['asset']],it) for it in items];rows=[];calls=0
 for motor in [it for it in items if it['asset']=='M077_stator']:
  name=motor['name'];stem=name.removesuffix('_motor');res={'motor':name,'wire_collisions':[],'connector_obstacles':[],'own_case_catalog_envelope_overlap_mm3':None}
  for label,shape in [('plug',plug),*[(str(i+1),s) for i,s in enumerate(wire)]]:
   w=placed(shape,motor)
   for it,s in zip(items,ps):
    if not overlaps(w,s):continue
    calls+=1;vol=w.common(s).Volume
    if vol<1e-5:continue
    row={'part':it['name'],'asset':it['asset'],'overlap_mm3':vol,'wire_lane':label}
    if label=='plug':
     if it['name']==name:res['own_case_catalog_envelope_overlap_mm3']=vol
     else:res['connector_obstacles'].append(row)
    else:res['wire_collisions'].append(row)
  rows.append(res);print(pn,name,'wire',len(res['wire_collisions']),'plug',len(res['connector_obstacles']),flush=True)
 out['poses'].append({'pose':pn,'motors':rows,'BRep_common_calls':calls});(D/'reviews'/'lead_access.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print('FINISHED',flush=True)

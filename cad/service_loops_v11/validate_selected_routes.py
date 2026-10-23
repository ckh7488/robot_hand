"""Independent exact BRep check of selected fixed exits; no flexible harness claim."""
import FreeCAD as A,Part,json,hashlib,time,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent;W=D.parent/'wiring_access_v10';B=D.parent/'modular_roots_v9';V=A.Vector;S=json.loads((B/'scene.json').read_text(encoding='utf-8'));choice=json.loads((D/'selected_routes.json').read_text(encoding='utf-8'))['selected'];choice=[c for c in choice if len(sys.argv)==1 or c['motor'] in sys.argv[1:]];manifest=json.loads((B/'cad_manifest.json').read_text(encoding='utf-8'));assets={n:Part.read(str(B/'parts'/(n+'.step'))) for n in manifest['assets']};assets['Thumb_metacarpal']=Part.read(str(W/'parts'/'Thumb_metacarpal_channel_candidate.step'));wires={};guide_assets={'little_spread_carrier':Part.read(str(D/'parts'/'Little_root_cable_saddle.step')),'little_mcp_link':Part.read(str(D/'parts'/'Little_MCP_link_cable_saddle.step'))}
for c in choice:
 for lab in ['plug','wire1','wire2','wire3']:
  key=c['template']+'_'+lab
  if key not in wires:wires[key]=Part.read(str(W/'parts'/(key+'.step')))
def placement(it):
 m=A.Matrix()
 for i in range(3):
  for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),it['R'][i][j])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 return A.Placement(m)
def posed(s,p):
 t=s.copy();t.Placement=p.multiply(t.Placement);return t
def gap(a,b):
 x,y=a.BoundBox,b.BoundBox
 return max(0,x.XMin-y.XMax,y.XMin-x.XMax)**2+max(0,x.YMin-y.YMax,y.YMin-x.YMax)**2+max(0,x.ZMin-y.ZMax,y.ZMin-x.ZMax)**2
out={'scope':'Exact BRep for one chosen FIXED lead exit per motor in6stored poses. Replaces one thumb-link candidate locally only. Own connector/case catalog-envelope mating unqualified; no connected cable, moving slack, wire-wire or full joint sweep.','wire_nominal_radius_mm':.95,'wire_reserve_radius_mm':1.25,'additional_gap_target_mm':.3,'rows':[],'manufacturing_release':False,'full_harness_validated':False};cache={};calls=0;t0=time.monotonic()
for pn,items in S.items():
 items=[it for it in items if not it['role'].startswith('arm_')];lookup={it['name']:it for it in items};objects=[(it,placement(it),posed(guide_assets.get(it['name'],assets[it['asset']]),placement(it))) for it in items]
 for c in choice:
  motor=lookup[c['motor']];mp=placement(motor);best={'distance_mm':1.,'capped_lower_bound':True};bad=[]
  for lab in ['plug','wire1','wire2','wire3']:
   key=c['template']+'_'+lab;w=posed(wires[key],mp)
   for it,p,s in objects:
    if lab=='plug' and it['name']==motor['name']:continue
    if gap(w,s)>1.:continue
    rel=mp.inverse().multiply(p).toMatrix();stamp=tuple(round(getattr(rel,'A%d%d'%(i,j)),8) for i in range(1,4) for j in range(1,5));ck=(key,it['name'],stamp)
    if ck not in cache:
     d=w.distToShape(s)[0];volume=w.common(s).Volume if d<1e-7 else 0;cache[ck]=(d,volume);calls+=1
    d,volume=cache[ck]
    if d<best['distance_mm']:best={'distance_mm':d,'source':lab,'obstacle':it['name'],'overlap_mm3':volume,'capped_lower_bound':False}
    if d<.3-1e-6:bad.append({'source':lab,'obstacle':it['name'],'distance_mm':d,'overlap_mm3':volume})
  row={'pose':pn,'motor':motor['name'],'template':c['template'],'minimum':best,'failures':bad,'passed':not bad};out['rows'].append(row);print(pn,motor['name'],'pass',row['passed'],'min',best['distance_mm'],flush=True)
 out['BRep_uncached_distance_queries']=calls;out['elapsed_seconds']=time.monotonic()-t0;out['all_selected_fixed_exits_passed']=all(r['passed'] for r in out['rows']) and len(out['rows'])==6*len(choice);(D/'reviews'/('selected_exact_delta.json' if len(sys.argv)>1 else 'selected_exact.json')).write_text(json.dumps(out,indent=2),encoding='utf-8')
print('FINISHED',out['all_selected_fixed_exits_passed'],flush=True)

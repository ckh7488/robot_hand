import FreeCAD as A,Part,Mesh,json
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';V=A.Vector
j=json.loads((D/'reviews'/'service_loop_curves.json').read_text(encoding='utf-8'));sc=json.loads((B/'scene.json').read_text(encoding='utf-8'))['Open'];meta={x['name']:x for x in sc};out=[];doc=A.newDocument('V11LittleLoopPackaging')
def place(it):
 m=A.Matrix()
 for i in range(3):
  for k in range(3):setattr(m,'A%d%d'%(i+1,k+1),it['R'][i][k])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 return A.Placement(m)
for q in [-20,0,60]:
 for r in [r for r in j['chosen']['curves'] if r['q_deg']==q]:
  P=[V(*p) for p in r['controls_mm']];b=Part.BezierCurve();b.setPoles(P);path=Part.Wire([b.toShape()]);profile=Part.Wire([Part.makeCircle(.95,P[0],(P[1]-P[0]))]);s=path.makePipeShell([profile],True,False);name='Little_loop_q'+str(q)+'_wire'+str(r['lane']+1);assert s.isValid() and len(s.Solids)==1
  s.exportStep(str(D/'parts'/(name+'.step')));out.append({'name':name,'q_deg':q,'lane':r['lane'],'centerline_length_mm':path.Length,'valid_single_solid':True,'volume_mm3':s.Volume})
  if q==0:o=doc.addObject('PartDesign::Feature',name);o.Shape=s;o.Placement=place(meta['little_mcp_motor'])
# Two fixed exits join the loop; one-port test harness/taps remain an electrical design item.
selected=json.loads((D/'selected_routes.json').read_text(encoding='utf-8'))['selected']
for route in selected:
 if route['motor'] not in ['little_mcp_motor','little_pip_motor']:continue
 for label in ['plug','wire1','wire2','wire3']:
  asset=route['template']+'_'+label;o=doc.addObject('PartDesign::Feature',route['motor']+'_'+label);o.Shape=Part.read(str(W/'parts'/(asset+'.step')));o.Placement=place(meta[route['motor']])
# Open-hand true CAD preview, not simulation model.
for it in sc:
 if it['role'].startswith('arm_'):continue
 if it['name']=='little_spread_carrier':s=Part.read(str(D/'parts'/'Little_root_cable_saddle.step'))
 elif it['name']=='little_mcp_link':s=Part.read(str(D/'parts'/'Little_MCP_link_cable_saddle.step'))
 elif it['asset']=='Thumb_metacarpal':s=Part.read(str(W/'parts'/'Thumb_metacarpal_channel_candidate.step'))
 else:s=Part.read(str(B/'parts'/(it['asset']+'.step')))
 o=doc.addObject('PartDesign::Feature',it['name']);o.Shape=s;o.Placement=place(it)
doc.recompute();doc.saveAs(str(D/'V11_open_hand_loop_packaging.FCStd'));Part.export(doc.Objects,str(D/'V11_open_hand_loop_packaging.step'));back=Part.read(str(D/'V11_open_hand_loop_packaging.step'));expected=sum(len(o.Shape.Solids) for o in doc.Objects);assert len(back.Solids)==expected;assert back.isValid();(D/'reviews'/'cad_roundtrip.json').write_text(json.dumps({'expected_solids':expected,'roundtrip_solids':len(back.Solids),'valid':back.isValid(),'objects':len(doc.Objects),'scope':'Open hand CAD including two exit templates,3geometric loop wires and2guide candidates. Hand only; arm CAD unchanged outside this add-on.','manufacturing_release':False},indent=2),encoding='utf-8')
(D/'reviews'/'loop_cad_export.json').write_text(json.dumps({'scope':'Actual hand CAD in Open plus local loop wire solids at3angles. All mm. Packaging candidate only; cable tie fixation/dynamic equilibrium/electrical wiring not approved.','wire_display_mesh_generation':'Run make_loop_display_meshes.py after CAD export','collision_uses_display_mesh':False,'wire_radius_mm':.95,'loop_length_mm':j['chosen']['length_mm'],'loop_shapes':out,'maximum_length_error_mm':max(abs(r['centerline_length_mm']-j['chosen']['length_mm']) for r in out)},indent=2),encoding='utf-8');print('CAD exports complete',flush=True)

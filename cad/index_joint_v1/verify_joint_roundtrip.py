import FreeCAD as App,Part,json,math
from pathlib import Path
r=Path(__file__).resolve().parent;d=App.openDocument(str(r/'index_root_joint.FCStd'));obs=[o for o in d.Objects if o.TypeId=='PartDesign::Feature'];s=Part.makeCompound([o.Shape for o in obs]);t=Part.read(str(r/'index_root_joint.step'))
g=d.getObject('Moving');g.Placement=App.Placement(App.Vector(),App.Rotation(App.Vector(1,0,0),70));d.recompute();gpos=g.getGlobalPlacement().multVec(App.Vector(0,48,0))
res={'FCStd_parts':len(obs),'FCStd_solids':len(s.Solids),'STEP_solids':len(t.Solids),'FCStd_valid':s.isValid(),'STEP_valid':t.isValid(),'volume_delta_mm3':abs(s.Volume-t.Volume),'bbox_delta_mm':max(abs(getattr(s.BoundBox,k)-getattr(t.BoundBox,k)) for k in ['XMin','YMin','ZMin','XMax','YMax','ZMax']),'moving_group_70deg_point_mm':list(gpos),'moving_group_transform_pass':(gpos-App.Vector(0,48*math.cos(math.radians(70)),48*math.sin(math.radians(70)))).Length<1e-9}
res['relative_volume_error']=res['volume_delta_mm3']/s.Volume
res['passed']=res['FCStd_parts']==len(json.loads((r/'joint_manifest.json').read_text())['parts']) and res['FCStd_solids']==res['STEP_solids'] and res['FCStd_valid'] and res['STEP_valid'] and res['bbox_delta_mm']<1e-6 and res['relative_volume_error']<1e-5 and res['moving_group_transform_pass']
assert res['passed'],res
(r/'cad_roundtrip.json').write_text(json.dumps(res,indent=2));print(json.dumps(res))

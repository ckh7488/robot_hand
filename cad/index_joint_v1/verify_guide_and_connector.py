import FreeCAD as App,Part,json,math
from pathlib import Path
V=App.Vector;r=Path(__file__).resolve().parent;d=App.openDocument(str(r/'index_root_joint.FCStd'));o={x.Name:x.Shape for x in d.Objects if x.TypeId=='PartDesign::Feature'}
res=json.loads((r/'cable_validation.json').read_text());low=1e9
for ob in d.getObject('Moving').Group:
 if not hasattr(ob,'Shape'):continue
 b=ob.Shape.BoundBox
 for y in [b.YMin,b.YMax]:
  for z in [b.ZMin,b.ZMax]:
   values=[math.radians(-20),math.radians(100)];t=math.atan2(-z,y)+math.pi
   for k in range(-2,3):
    a=t+k*2*math.pi
    if math.radians(-20)<=a<=math.radians(100):values.append(a)
   low=min(low,*[y*math.cos(a)-z*math.sin(a) for a in values])
res['guide_motion_gap_lower_bound_mm']=low-o['Cable_guide'].BoundBox.YMax
body=Part.makeCompound([s for n,s in o.items() if n.startswith('Motor_part')]);res['guide_case_overlap_mm3']=o['Cable_guide'].common(body).Volume
res['extra_connector_pose_checks']=[]
for q in [-20,40,100]:
 for name in ['FPX330_H101','Idler_disc']:
  t=o[name].copy();t.rotate(V(),V(1,0,0),q);dist=t.distToShape(o['JST_EHR3_outer_envelope'])[0]
  row={'q_deg':q,'target':name,'distance_mm':dist,'overlap_mm3':0 if dist>1e-6 else t.common(o['JST_EHR3_outer_envelope']).Volume};res['extra_connector_pose_checks'].append(row)
res['wire_continuous_passed']=res['wire_continuous_passed'] and res['guide_motion_gap_lower_bound_mm']>.5 and res['guide_case_overlap_mm3']<1e-5 and all(x['overlap_mm3']<1e-5 for x in res['extra_connector_pose_checks'])
(r/'cable_validation.json').write_text(json.dumps(res,indent=2));print(json.dumps({'guide_gap_mm':res['guide_motion_gap_lower_bound_mm'],'guide_case_overlap':res['guide_case_overlap_mm3'],'connector_moving':res['extra_connector_pose_checks'],'passed':res['wire_continuous_passed']}))

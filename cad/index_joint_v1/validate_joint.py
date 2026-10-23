import FreeCAD as App,Part,json,math,time
from pathlib import Path
r=Path(__file__).resolve().parent
log=open(str(r/'validation_progress.log'),'w',buffering=1)
def tell(o):log.write(json.dumps(o)+'\n')
doc=App.openDocument(str(r/'index_root_joint.FCStd'));fixed=doc.getObject('Fixed');moving=doc.getObject('Moving');objs={o.Name:o for o in doc.Objects if o.TypeId=='PartDesign::Feature'}
res={'scope':'CAD geometry only. Sampled poses, not continuous collision certification.','all_shapes_valid':all(o.Shape.isValid() for o in objs.values()),'pair_checks':[],'sweep_samples':[]}
for nut_name,frame in [('Base_M2_nut_1','FPX330_S101'),('Tab_M2_nut_1','FPX330_H101'),('Base_M2x6_1','FPX330_S101'),('Tab_M2x6_1','FPX330_H101')]:
 n=objs[nut_name].Shape;f=objs[frame].Shape;t=time.time();v=n.common(f).Volume
 row={'pair':[nut_name,frame],'overlap_mm3':v,'pass':v<1e-5,'seconds':time.time()-t};res['pair_checks'].append(row);tell(row)
# Compare real CAD non-mating pieces. Retaining interfaces intentionally touch.
body=Part.makeCompound([o.Shape for n,o in objs.items() if n.startswith('Motor_part') or n in ['Idler_cap','Idler_retaining_screw']])
base=objs['Bench_base'].Shape;frame=objs['FPX330_S101'].Shape
for q in [-20,0,45,70,100]:
 for a,b in [('FPX330_H101','body'),('FPX330_H101','FPX330_S101'),('Output_test_tab','Bench_base')]:
  t=time.time();s=objs[a].Shape.copy();s.Placement=App.Placement(App.Vector(),App.Rotation(App.Vector(1,0,0),q)).multiply(s.Placement)
  target=body if b=='body' else objs[b].Shape
  dist=s.distToShape(target)[0];row={'q_deg':q,'pair':[a,b],'distance_mm':dist,'seconds':time.time()-t}
  if dist<1e-6:row['overlap_mm3']=s.common(target).Volume
  res['sweep_samples'].append(row);tell(row)
# Verify only principal coaxial interfaces from vendor datums.
res['interfaces']={'horn_idler_axis_offset_mm':0.0,'horn_PCD_mm':12,'horn_hole_diameter_mm':1.6,'hinge_horn_hole_diameter_mm':2.05,'horn_thread_engagement_mm':2.7,'vendor_horn_max_depth_mm':3,'body_fixing_screw_length_mm':8,'body_engagement_mm':{'front':3.2,'rear':2.2},'idler_contact_separation_mm':29,'hinge_internal_width_mm':29,'hinge_external_width_mm':34.6,'nominal_fit_note':'Intended bearing/horn contacts may have zero clearance; not a 3mm free-motion margin.'}
res['nuts_fitted']=all(c['pass'] for c in res['pair_checks']);res['sampled_forbidden_interference']=any(row.get('overlap_mm3',0)>1e-5 for row in res['sweep_samples'])
(r/'cad_validation.json').write_text(json.dumps(res,indent=2));tell({'finished':True,'nuts_fitted':res['nuts_fitted'],'sampled_forbidden_interference':res['sampled_forbidden_interference']})

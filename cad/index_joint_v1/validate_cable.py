"""Exact CAD checks + continuous geometric separation for FIXED single-motor leads."""
import FreeCAD as App,Part,json,math,time
from pathlib import Path
from wire_harness import wire_shape
V=App.Vector;r=Path(__file__).resolve().parent;spec=json.loads((r/'joint_hardware_spec.json').read_text());d=App.openDocument(str(r/'index_root_joint.FCStd'));o={x.Name:x.Shape for x in d.Objects if x.TypeId=='PartDesign::Feature'}
log=open(str(r/'cable_validation_progress.log'),'w',buffering=1)
def emit(x):log.write(json.dumps(x)+'\n')
res={'scope':'Single fixed-motor harness only; no downstream motor chain, torque or flex-life certification.','static':[],'moving_samples':[],'connector_envelope_checks':[]}
body=Part.makeCompound([s for n,s in o.items() if n.startswith('Motor_part') and n not in ['Motor_part_13','Motor_part_14']])
for i,y in enumerate(spec['wire']['start_y_mm']):
 w=wire_shape(y,spec,spec['wire']['extra_radial_allowance_mm'])
 for name,t in [('case',body),('fixed_frame',o['FPX330_S101'])]:
  dist=w.distToShape(t)[0];row={'wire':i+1,'target':name,'reserve_radius_mm':.3,'distance_mm':dist,'overlap_mm3':w.common(t).Volume};res['static'].append(row);emit(row)
# Actual holder, not a simulated compliant cable. CAD wire may touch channel after tying.
for name in ['Guide_M3x16','Guide_M3_nut']:
 row={'part':name,'case_overlap_mm3':o[name].common(body).Volume};res['static'].append(row);emit(row)
plug=o['JST_EHR3_outer_envelope']
for name in ['FPX330_S101','Idler_disc','Idler_cap','Idler_retaining_screw','Motor_part_02']:
 row={'target':name,'overlap_mm3':plug.common(o[name]).Volume,'distance_mm':plug.distToShape(o[name])[0]};res['connector_envelope_checks'].append(row);emit(row)
# Back half of all moving CAD must be inside a conservative radius8.5mm cylinder.
# This is a 3D BRep inclusion check; source horn/idler/hinge rear arcs are nominal R8.
moving=[x for x in d.getObject('Moving').Group if hasattr(x,'Shape')]
clip=Part.makeBox(100,100,100,V(-50,-100,-50));cyl=Part.makeCylinder(8.5,100,V(-50,0,0),V(1,0,0));outside=[]
for obj in moving:
 if obj.Shape.BoundBox.YMin>0:continue
 back=obj.Shape.common(clip);vol=back.cut(cyl).Volume
 outside.append({'part':obj.Name,'outside_radius_8p5_mm3':vol});emit(outside[-1])
radius=spec['wire']['insulation_od_max_mm']/2+spec['wire']['extra_radial_allowance_mm']
ymax=max(spec['wire']['start_y_mm']);z=spec['wire']['z_mm'];a0,a1=map(math.radians,[-20,100])
# local Y of a stationary point under inverse joint rotation = y cos(q)+z sin(q).
candidates=[a0,a1];crit=math.atan2(z,ymax)
for k in range(-3,4):
 a=crit+2*math.pi*k
 if a0<=a<=a1:candidates.append(a)
upper=max(ymax*math.cos(a)+z*math.sin(a) for a in candidates)+radius
radial=math.hypot(ymax,z)-radius-8.5
# Curved rear portions start X=-20 and go more negative; moving CAD nominal xmin >=-17.3.
xgap=min(s.Shape.BoundBox.XMin for s in moving)-(-20+radius)
res['continuous_separation']={'moving_back_outside_cylinder':outside,'straight_lead_max_local_y_mm':upper,'straight_lead_radial_gap_lower_bound_mm':radial,'rear_bend_axial_gap_lower_bound_mm':xgap,'wire_extra_radius_mm':.3,'basis':'Exact BRep cylinder inclusion plus rigid-rotation invariant radius and axial plane bounds. Stock CAD only; printed/manufacturing error not certified.'}
res['wire_continuous_passed']=all(x['outside_radius_8p5_mm3']<1e-5 for x in outside) and upper<0 and radial>.5 and xgap>.5 and all(x.get('overlap_mm3',0)<1e-5 for x in res['static'])
# Independent exact-distance checks at endpoints and a middle angle on worst (nearest-axis) lane.
w=wire_shape(ymax,spec,.3)
for q in [-20,40,100]:
 h=o['FPX330_H101'].copy();h.rotate(V(),V(1,0,0),q);row={'q_deg':q,'wire_lane':3,'hinge_distance_mm':w.distToShape(h)[0]};res['moving_samples'].append(row);emit(row)
res['connector_case_fit_closed']=False
res['connector_case_fit_note']='EHR-3 is a conservative public-drawing outer envelope. Record all overlap; do not treat it as a hollow vendor housing or hide it as a passed physical fit. Physical mated housing / pin seating and crimp are still required.'
res['frame_window_nominal_clearance_passed']=next(x for x in res['connector_envelope_checks'] if x['target']=='FPX330_S101')['distance_mm']>=.9
res['manufacturing_release']=False
(r/'cable_validation.json').write_text(json.dumps(res,indent=2));emit({'finished':True,'wire_continuous_passed':res['wire_continuous_passed'],'frame_window_clearance_passed':res['frame_window_nominal_clearance_passed']})

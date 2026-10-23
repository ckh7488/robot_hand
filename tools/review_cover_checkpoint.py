"""Fresh checks of the committed cover STEP and FCStd; does not import its generator."""
import argparse,json,sys
from pathlib import Path
import FreeCAD as A,Part
from compare_saved_breps import compare_objects
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);ap.add_argument('--output',type=Path);args=ap.parse_args();P=args.root.resolve();D=P/'cad/assembly_fixes_v13';B=P/'cad/modular_roots_v9';V=A.Vector
old=Part.read(str(B/'parts/Palm_front_cover.step'));cover=Part.read(str(D/'parts/Palm_front_cover_recessed.step'));screws=Part.read(str(D/'parts/M3x8_button_cover_screws.step'));nuts=Part.read(str(B/'parts/M3_cover_nuts.step'))
assert cover.isValid() and len(cover.Solids)==1 and len(screws.Solids)==5
assert cover.cut(old).Volume<1e-7 and screws.common(cover).Volume<1e-7
rows=[]
for s in screws.Solids:
 bb=s.BoundBox;x=(bb.XMin+bb.XMax)/2;y=(bb.YMin+bb.YMax)/2
 n=min(nuts.Solids,key=lambda n:(n.CenterOfMass.x-x)**2+(n.CenterOfMass.y-y)**2);nb=n.BoundBox
 assert (n.CenterOfMass.x-x)**2+(n.CenterOfMass.y-y)**2<1e-12
 # Read the under-head position from the planar annular face, without generator parameters.
 bearing=[f for f in s.Faces if type(f.Surface).__name__=='Plane' and abs(f.CenterOfMass.z-(bb.ZMax-1.65))<1e-8 and f.Area>10]
 assert len(bearing)==1
 seat=bearing[0].CenterOfMass.z
 assert abs((seat-bb.ZMin)-8)<1e-7 and bb.ZMax<26 and bb.ZMin<nb.ZMin and seat>nb.ZMax
 ann=Part.makeCylinder(3.05,seat-22,V(x,y,22)).cut(Part.makeCylinder(1.8,seat-22,V(x,y,22)))
 assert ann.cut(cover).Volume<1e-7
 rows.append({'xy_mm':[x,y],'shank_length_mm':seat-bb.ZMin,'head_top_below_cover_mm':26-bb.ZMax,'remaining_bearing_land_mm':seat-22,'full_nut_engagement_mm':nb.ZLength,'tip_past_nut_mm':nb.ZMin-bb.ZMin})
# Derive the critical little fingertip from the actual design and audited angle sequence.
design=json.loads((B/'design.json').read_text(encoding='utf-8'));f=next(x for x in design['hand']['fingers'] if x['name']=='little')
assert f['base_rpy']==[0,0,0]
p=A.Placement(V(*f['root'])+V(0,0,design['hand']['root_axis_offset_z']),A.Rotation())
for angle,L in [(89.1,f['links'][0]),(99.,f['links'][1])]:p=p.multiply(A.Placement(V(),A.Rotation(V(1,0,0),angle))).multiply(A.Placement(V(0,L,0),A.Rotation()))
p=p.multiply(A.Placement(V(),A.Rotation(V(1,0,0),79.2)))
tip=Part.read(str(B/'parts/Tip_28.step'));tip.Placement=p.multiply(tip.Placement)
old_s=Part.read(str(B/'parts/M3_cover_screws.step'))
contact={}
for label,shape in [('old_screws',old_s),('new_screws',screws),('new_cover',cover)]:contact[label]={'gap_mm':tip.distToShape(shape)[0],'overlap_mm3':tip.common(shape).Volume}
assert contact['old_screws']['overlap_mm3']>1e-6,'The received audit contact must reproduce'
assert contact['new_screws']['overlap_mm3']<1e-7 and contact['new_cover']['overlap_mm3']<1e-7
# Also inspect the remaining full-fist clearance in exact STEP, not only the mesh screen.
end=A.Placement(V(*f['root'])+V(0,0,design['hand']['root_axis_offset_z']),A.Rotation())
for angle,L in [(90.,f['links'][0]),(100.,f['links'][1])]:end=end.multiply(A.Placement(V(),A.Rotation(V(1,0,0),angle))).multiply(A.Placement(V(0,L,0),A.Rotation()))
end=end.multiply(A.Placement(V(),A.Rotation(V(1,0,0),80.)))
endtip=Part.read(str(B/'parts/Tip_28.step'));endtip.Placement=end.multiply(endtip.Placement)
endpoint={label:{'gap_mm':endtip.distToShape(shape)[0],'overlap_mm3':endtip.common(shape).Volume} for label,shape in [('new_screws',screws),('new_cover',cover)]}
endpoint['cover_1mm_clearance_target_met']=bool(endpoint['new_cover']['gap_mm']>=1.)
same=compare_objects(P/'cad/audit_fixes_v12/V12_hand_packaging.FCStd',D/'V13_hand_packaging.FCStd',exclude=['Palm_front_cover','Palm_cover_screws'])
doc=A.openDocument(str(D/'V13_hand_packaging.FCStd'))
for name,s in [('Palm_front_cover',cover),('Palm_cover_screws',screws)]:
 saved=doc.getObject(name).Shape;assert saved.cut(s).Volume+s.cut(saved).Volume<1e-7,name
out={'FreeCAD':A.Version()[:3],'cover_valid_single_solid':True,'modified_objects':['Palm_front_cover','Palm_cover_screws'],'cover_material_only_removed_mm3':old.cut(cover).Volume,'screw_cover_overlap_mm3':screws.common(cover).Volume,'screws_remeasured':rows,'independent_critical_little_tip_placement':list(p.toMatrix().A),'critical_t0_99':contact,'deep_fist_endpoint':endpoint,'unchanged_objects':same['unchanged_objects'],'max_saved_BRep_token_roundoff':same['max_numeric_roundoff'],'scope':'Candidate STEP and saved assembly consistency, critical actual-CAD contact and nominal bearing lands/engagement only. No complete harness, FDM strength, coupling-space or actuator approval.','manufacturing_release':False}
text=json.dumps(out,indent=2);print(text,flush=True)
if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text,encoding='utf-8')

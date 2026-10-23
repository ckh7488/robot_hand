"""Recess lower-profile cover screws; preserve joint axes and the audited V9 files."""
import json,math,sys
from pathlib import Path
import FreeCAD as A,Part,Mesh
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';V=A.Vector
spec=json.loads((D/'sources/cover_screw.json').read_text(encoding='utf-8'))
old=Part.read(str(B/'parts/Palm_front_cover.step'));old_s=Part.read(str(B/'parts/M3_cover_screws.step'))
# Top is Z26. The cover's complete screw lands start at Z22, not the relieved skin.
seat=24.05;head_h=spec['max_head_height_mm'];head_r=spec['max_head_diameter_mm']/2;length=8.;bore_r=3.15
locations=[];new=old.copy();screws=[];rows=[]
for solid in old_s.Solids:
 bb=solid.BoundBox;x=(bb.XMin+bb.XMax)/2;y=(bb.YMin+bb.YMax)/2;locations.append((x,y))
 new=new.cut(Part.makeCylinder(bore_r,4,V(x,y,seat)))
 shaft=Part.makeCylinder(1.5,length,V(x,y,seat-length));head=Part.makeCylinder(head_r,head_h,V(x,y,seat))
 screws.append(shaft.fuse(head).removeSplitter())
 # Test the retained complete bearing annulus, excluding original shaft clearance and the outer edge.
 annulus=Part.makeCylinder(3.05,seat-22,V(x,y,22)).cut(Part.makeCylinder(1.8,seat-22,V(x,y,22)))
 missing=annulus.cut(new).Volume
 assert missing<1e-7,(x,y,missing)
 rows.append({'xy_mm':[x,y],'seat_z_mm':seat,'head_top_z_mm':seat+head_h,'recess_below_cover_mm':26-seat-head_h,'remaining_full_bearing_land_mm':seat-22,'retained_annulus_missing_mm3':missing,'nut_z_mm':[17.5,19.9],'full_nut_engagement_mm':2.4,'thread_tip_z_mm':seat-length,'tip_beyond_nut_mm':17.5-(seat-length)})
new=new.removeSplitter();ns=Part.makeCompound(screws)
assert new.isValid() and len(new.Solids)==1 and len(ns.Solids)==5
assert new.cut(old).Volume<1e-7
assert ns.common(new).Volume<1e-7
for name,shape in [('Palm_front_cover_recessed',new),('M3x8_button_cover_screws',ns)]:
 shape.exportStep(str(D/'parts'/(name+'.step')));Mesh.Mesh(shape.tessellate(.03)).write(str(D/'parts'/(name+'.stl')))
 # Plain tuples allow use by analysis Python without sharing the FreeCAD runtime.
 vv,ff=shape.tessellate(.03);(D/'parts'/(name+'.mesh.json')).write_text(json.dumps({'vertices':[[p.x,p.y,p.z] for p in vv],'faces':ff}),encoding='utf-8')
doc=A.openDocument(str(D.parent/'audit_fixes_v12/V12_hand_packaging.FCStd'))
for name,shape in [('Palm_front_cover',new),('Palm_cover_screws',ns)]:
 obj=doc.getObject(name);pl=obj.Placement;obj.Shape=shape;obj.Placement=pl
# Candidate remains the hand packaging document, not a complete arm redesign.
doc.Label='V13 recessed cover screw candidate';doc.recompute();doc.saveAs(str(D/'V13_hand_packaging.FCStd'))
report={'source_cover':'../modular_roots_v9/parts/Palm_front_cover.step','source_screws':'../modular_roots_v9/parts/M3_cover_screws.step','source_assembly':'../audit_fixes_v12/V12_hand_packaging.FCStd','screw_spec':spec,'counterbore_diameter_mm':2*bore_r,'removed_cover_material_mm3':old.cut(new).Volume,'material_outside_source_mm3':new.cut(old).Volume,'screw_cover_overlap_mm3':ns.common(new).Volume,'cover_valid_single_solid':True,'screw_components':5,'screws':rows,'scope':'Nominal BRep geometry. No FDM clamp/creep approval or all-body servicing/retention proof. Screw envelope omits socket and rounded crown.','manufacturing_release':False}
(D/'reviews/build_geometry.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report,indent=2),flush=True)

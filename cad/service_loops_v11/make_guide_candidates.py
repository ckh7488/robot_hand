"""Little-finger-only broad cable saddles. Separate print candidates, no baseline overwrite."""
import FreeCAD as A,Part,Mesh,json,math
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';V=A.Vector
box=lambda x,y,z,p:Part.makeBox(x,y,z,V(*p))
root=Part.read(str(B/'parts'/'Compact_root.step'));link=Part.read(str(B/'parts'/'Flex_link_55.step'))
# Work in little MCP stator coordinates then translate +32Z to Compact_root coordinates.
a=box(22,14,4.5,(-46,-16,-18)).fuse(box(6,14,16,(-45,-16,-34)))
a=a.cut(box(10,20,25,(-54.5,-19,-38)))
for y in [-11.5,-9,-6.5]:a=a.cut(Part.makeCylinder(1.75,24,V(-44.5,y,-38),V(0,0,1)))
cuts=[]
for y in [-11.5,-9,-6.5]:
 p0=V(-25.6,y,-8);p1=V(-34.5,y,-8);pm=V(-34.5-10*math.sqrt(.5),y,-8-10*(1-math.sqrt(.5)));p2=V(-44.5,y,-18);p3=V(-44.5,y,-36)
 path=Part.Wire([Part.makeLine(p0,p1),Part.Arc(p1,pm,p2).toShape(),Part.makeLine(p2,p3)]);cuts.append(path.makePipeShell([Part.Wire([Part.makeCircle(1.75,p0,V(-1,0,0))])],True,False))
tool=cuts[0].fuse(cuts[1]).fuse(cuts[2]).removeSplitter();a=a.cut(tool).removeSplitter();a.translate(V(0,0,32));newroot=root.fuse(a).removeSplitter()
# Same construction on proximal link: wide plate, open-top three-wire saddle.
b=a.copy();b.translate(V(0,55,-32));newlink=link.fuse(b).removeSplitter()
doc=A.newDocument('LittleCableGuideCandidates');rows=[]
for name,old,new in [('Little_root_cable_saddle',root,newroot),('Little_MCP_link_cable_saddle',link,newlink)]:
 assert new.isValid() and len(new.Solids)==1,(name,new.isValid(),len(new.Solids))
 new.exportStep(str(D/'parts'/(name+'.step')));Mesh.Mesh(new.tessellate(.03)).write(str(D/'parts'/(name+'.stl')))
 o=doc.addObject('PartDesign::Feature',name);o.Shape=new
 added=new.cut(old);added.exportStep(str(D/'parts'/(name+'_added.step')));Mesh.Mesh(added.tessellate(.03)).write(str(D/'parts'/(name+'_added.stl')));rows.append({'part':name,'valid_single_solid':True,'old_volume_mm3':old.Volume,'new_volume_mm3':new.Volume,'added_volume_mm3':added.Volume,'removed_original_volume_mm3':old.cut(new).Volume,'added_bounds_mm':[added.BoundBox.XMin,added.BoundBox.YMin,added.BoundBox.ZMin,added.BoundBox.XMax,added.BoundBox.YMax,added.BoundBox.ZMax]})
doc.recompute();doc.saveAs(str(D/'Little_cable_saddle_candidates.FCStd'))
(D/'reviews'/'guide_geometry.json').write_text(json.dumps({'scope':'Only little-finger Compact_root and55mm MCP link candidates. Axes and holes unchanged; no original material removed. New saddle geometry must pass moving CAD checks. Cable ties, actual clamping, print strength and fatigue not qualified.','channel_radius_mm':1.75,'wire_nominal_radius_mm':.95,'wire_reserve_radius_mm':1.25,'manufacturing_release':False,'parts':rows},indent=2),encoding='utf-8');print(json.dumps(rows,indent=2))

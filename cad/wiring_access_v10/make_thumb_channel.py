"""Reversible thumb-link cable-channel candidate; exact CAD, no strength approval."""
import FreeCAD as A,Part,Mesh,json,math,hashlib
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';S=json.loads((B/'scene.json').read_text(encoding='utf-8'))['Open'];V=A.Vector
link=next(it for it in S if it['name']=='thumb_mcp_link');motor=next(it for it in S if it['name']=='thumb_pip_motor')
def placement(it):
 m=A.Matrix()
 for i in range(3):
  for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),it['R'][i][j])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 return A.Placement(m)
rel=placement(link).inverse().multiply(placement(motor));cuts=[]
for y in [-11.5,-9,-6.5]:
 z=-8;r=10;xb=-34.5;h=math.sqrt(.5);p0=V(-25.6,y,z);p1=V(xb,y,z);pm=V(xb-r*h,y,z-r*(1-h));p2=V(xb-r,y,z-r);p3=V(xb-r,y,-36)
 path=Part.Wire([Part.makeLine(p0,p1),Part.Arc(p1,pm,p2).toShape(),Part.makeLine(p2,p3)]);s=path.makePipeShell([Part.Wire([Part.makeCircle(1.75,p0,V(-1,0,0))])],True,False);s.Placement=rel.multiply(s.Placement);cuts.append(s)
tool=cuts[0]
for c in cuts[1:]:tool=tool.fuse(c)
tool=tool.removeSplitter();old=Part.read(str(B/'parts'/'Thumb_metacarpal.step'));new=old.cut(tool).removeSplitter();removed=old.common(tool)
assert new.isValid() and len(new.Solids)==1
new.exportStep(str(D/'parts'/'Thumb_metacarpal_channel_candidate.step'));Mesh.Mesh(new.tessellate(.05)).write(str(D/'parts'/'Thumb_metacarpal_channel_candidate.stl'));removed.exportStep(str(D/'parts'/'Thumb_channel_removed_material.step'))
doc=A.newDocument('ThumbCableChannelCandidate');o=doc.addObject('PartDesign::Feature','Thumb_metacarpal_channel_candidate');o.Shape=new;doc.recompute();doc.saveAs(str(D/'Thumb_metacarpal_channel_candidate.FCStd'))
fast=[]
for it in S:
 if it['role']!='fastener' or 'thumb_mcp_link' not in it.get('mating_with',[]):continue
 s=Part.read(str(B/'parts'/(it['asset']+'.step')));s.Placement=placement(link).inverse().multiply(placement(it)).multiply(s.Placement)
 fast.append({'fastener':it['name'],'removed_material_distance_mm':removed.distToShape(s)[0],'removed_material_overlap_mm3':removed.common(s).Volume})
out={'scope':'Only local cable channel in one printed thumb link. No motor, axis, bolt-hole or approved V9 geometry changed. Structural strength, tool access and genuine plug strain relief not qualified.','route_template':'negativeZ_outboard','wire_nominal_radius_mm':.95,'wire_reserve_radius_mm':1.25,'cut_radius_mm':1.75,'additional_nominal_gap_mm':.5,'old_volume_mm3':old.Volume,'new_volume_mm3':new.Volume,'removed_volume_mm3':removed.Volume,'new_material_outside_old_mm3':new.cut(old).Volume,'one_valid_connected_solid':True,'related_fasteners':fast,'baseline_link_sha256':hashlib.sha256((B/'parts'/'Thumb_metacarpal.step').read_bytes()).hexdigest(),'manufacturing_release':False}
(D/'reviews'/'thumb_channel.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(out,indent=2))

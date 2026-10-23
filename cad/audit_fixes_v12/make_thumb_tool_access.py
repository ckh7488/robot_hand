"""Fix audited straight-driver obstruction while retaining V10 cable channel and original axes/holes."""
import FreeCAD as A,Part,Mesh,json,math
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';S=json.loads((B/'scene.json').read_text(encoding='utf-8'))['Open'];V=A.Vector
meta={x['name']:x for x in S}
def pl(it):
 m=A.Matrix()
 for i in range(3):
  for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),it['R'][i][j])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 return A.Placement(m)
motor=pl(meta['thumb_pip_motor']);link=pl(meta['thumb_mcp_link']);candidates=[]
for sx in [-1,1]:
 for sy in [-1,1]:
  point=motor.multVec(V(sx*math.sqrt(32)-14.5,-7.5+sy*(7.5+math.sqrt(32)),-11.2));candidates.append(point)
p=min(candidates,key=lambda p:(p-V(87,73,-3.2)).Length);axis=motor.Rotation.multVec(V(0,0,1));local=link.inverse().multVec(p);axlocal=link.Rotation.inverted().multVec(axis)
old=Part.read(str(W/'parts'/'Thumb_metacarpal_channel_candidate.step'));cutter=Part.makeCylinder(1.75,60,local+axlocal*.05,axlocal);new=old.cut(cutter).removeSplitter();assert new.isValid() and len(new.Solids)==1
new.exportStep(str(D/'parts'/'Thumb_metacarpal_tool_access.step'));Mesh.Mesh(new.tessellate(.05)).write(str(D/'parts'/'Thumb_metacarpal_tool_access.stl'));doc=A.newDocument('ThumbToolAccessV12');o=doc.addObject('PartDesign::Feature','Thumb_metacarpal_tool_access');o.Shape=new;doc.recompute();doc.saveAs(str(D/'Thumb_tool_access.FCStd'))
# Exact tool clearance on the audited minimal subassembly: S101 + printed link + nuts.
posed_old=old.copy();posed_old.Placement=link.multiply(posed_old.Placement);posed_new=new.copy();posed_new.Placement=link.multiply(posed_new.Placement)
segments=[('head_pass',1.6,.05,8),('shaft',1.4,8,60),('handle',8,60,140)];rows=[]
for label,r,a,b in segments:
 tool=Part.makeCylinder(r,b-a,p+axis*a,axis)
 for variant,part in [('before',posed_old),('after',posed_new)]:rows.append({'tool_segment':label,'variant':variant,'obstacle':'thumb_mcp_link','distance_mm':tool.distToShape(part)[0],'overlap_mm3':tool.common(part).Volume})
 for n in ['thumb_pip_fixed_frame','thumb_pip_base_nuts']:
  it=meta[n];s=Part.read(str(B/'parts'/(it['asset']+'.step')));s.Placement=pl(it).multiply(s.Placement);rows.append({'tool_segment':label,'variant':'both','obstacle':n,'distance_mm':tool.distToShape(s)[0],'overlap_mm3':tool.common(s).Volume})
# Export the local removed volume for structural review and proof of unchanged support holes.
removed=old.cut(new);removed.exportStep(str(D/'parts'/'Thumb_tool_access_removed.step'));outside=new.cut(old).Volume
out={'source':'Independent V9 audit F6, thumb_pip_base_screws first screw; hole cut from V10 thumb-channel candidate.','exact_head_top_world_mm':[p.x,p.y,p.z],'exact_outward_axis_world':[axis.x,axis.y,axis.z],'bore_diameter_mm':3.5,'bore_start_above_head_mm':.05,'bore_length_mm':60,'removed_volume_mm3':removed.Volume,'new_material_outside_source_mm3':outside,'source_volume_mm3':old.Volume,'new_volume_mm3':new.Volume,'valid_single_solid':True,'rows':rows,'scope':'Straight tool on minimum preassembled link/S101/nut stage. Other screw positions, complete hand tool sweep, retention and FDM strength not approved.','manufacturing_release':False};assert all(r['overlap_mm3']<1e-6 for r in rows if r['variant'] in ['after','both']);(D/'reviews'/'thumb_tool_access.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(out,indent=2))

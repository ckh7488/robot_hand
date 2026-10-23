"""Fresh BRep computations from committed CAD, without importing candidate generators."""
import argparse,json,math,hashlib
from pathlib import Path
import FreeCAD as A
import Part
P=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path);args=ap.parse_args()
B=P/'cad/modular_roots_v9';W=P/'cad/wiring_access_v10';D=P/'cad/audit_fixes_v12'
scene=json.loads((B/'scene.json').read_text(encoding='utf-8'))['Open']
meta={x['name']:x for x in scene}
def placement(item):
    m=A.Matrix()
    for i in range(3):
        for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),item['R'][i][j])
        setattr(m,'A%d4'%(i+1),item['pos_mm'][i])
    return A.Placement(m)
def posed(shape,p):
    s=shape.copy();s.Placement=p.multiply(s.Placement);return s
old=Part.read(str(W/'parts/Thumb_metacarpal_channel_candidate.step'))
new=Part.read(str(D/'parts/Thumb_metacarpal_tool_access.step'))
base=Part.read(str(B/'parts/Thumb_metacarpal.step'))
assert new.isValid() and len(new.Solids)==1
outside=new.cut(old).Volume
outside_baseline=new.cut(base).Volume
removed=old.cut(new).Volume
assert outside < 1e-6 and outside_baseline < 1e-6 and removed > 38.
link=placement(meta['thumb_mcp_link'])
# Derive the first audited head directly from the saved screw solids in the Open assembly.
screw_item=meta['thumb_pip_base_screws']
screws=posed(Part.read(str(B/'parts'/(screw_item['asset']+'.step'))),placement(screw_item))
axis=placement(meta['thumb_pip_motor']).Rotation.multVec(A.Vector(0,0,1))
assert (axis-A.Vector(0,0,1)).Length < 1e-9
solid=min(screws.Solids,key=lambda s:(s.CenterOfMass.x-87)**2+(s.CenterOfMass.y-73)**2)
bb=solid.BoundBox
head=A.Vector((bb.XMin+bb.XMax)/2,(bb.YMin+bb.YMax)/2,bb.ZMax)
reported=json.loads((D/'reviews/thumb_tool_access.json').read_text(encoding='utf-8'))
assert (head-A.Vector(*reported['exact_head_top_world_mm'])).Length < 1e-6
new_world=posed(new,link);old_world=posed(old,link)
obstacles={'thumb_mcp_link':new_world}
for name in ['thumb_pip_fixed_frame','thumb_pip_base_nuts']:
    it=meta[name];obstacles[name]=posed(Part.read(str(B/'parts'/(it['asset']+'.step'))),placement(it))
rows=[]
for name,r,start,end in [('head_pass',1.6,.05,8),('shaft',1.4,8,60),('handle',8,60,140)]:
    tool=Part.makeCylinder(r,end-start,head+axis*start,axis)
    for obstacle,shape in obstacles.items():
        vol=tool.common(shape).Volume;gap=tool.distToShape(shape)[0]
        assert vol < 1e-6,(name,obstacle,vol)
        rows.append({'tool':name,'obstacle':obstacle,'overlap_mm3':vol,'gap_mm':gap})
    if name=='shaft':
        before=tool.common(old_world).Volume
        assert before > 24.
doc=A.openDocument(str(D/'V12_hand_packaging.FCStd'))
objects=[x for x in doc.Objects if hasattr(x,'Shape')]
assert len(objects)==252
assert sum(len(x.Shape.Solids) for x in objects)==976
modified=doc.getObject('thumb_mcp_link').Shape
symmetric_difference=modified.cut(new_world).Volume+new_world.cut(modified).Volume
assert symmetric_difference < 1e-6,symmetric_difference
# All other pre-existing objects retain V11 shape and placement.
prior=A.openDocument(str(P/'cad/service_loops_v11/V11_open_hand_loop_packaging.FCStd'))
unchanged=0
boolean_fallbacks=[]
for obj in objects:
    if obj.Name=='thumb_mcp_link':continue
    source=prior.getObject(obj.Name)
    assert source is not None,obj.Name
    assert abs(source.Shape.Volume-obj.Shape.Volume) < 1e-6,obj.Name
    assert abs(source.Shape.Area-obj.Shape.Area) < 1e-5,obj.Name
    for v,w in zip(source.Placement.toMatrix().A,obj.Placement.toMatrix().A):assert abs(v-w)<1e-9,obj.Name
    # Exact shape equality in saved documents, after global placement.
    if hashlib.sha256(source.Shape.exportBrepToString().encode()).digest()!=hashlib.sha256(obj.Shape.exportBrepToString().encode()).digest():
        delta=source.Shape.cut(obj.Shape).Volume+obj.Shape.cut(source.Shape).Volume
        assert delta<1e-6,(obj.Name,delta)
        boolean_fallbacks.append({'object':obj.Name,'symmetric_difference_mm3':delta})
    unchanged+=1
result={'FreeCAD':A.Version()[:3],'new_single_valid_solid':True,'material_outside_V10_mm3':outside,
        'material_outside_V9_mm3':outside_baseline,'removed_from_V10_mm3':removed,
        'head_rederived_from_actual_screw_BRep_mm':[head.x,head.y,head.z],
        'before_shaft_overlap_mm3':before,'tool_checks':rows,'assembly_objects':len(objects),'assembly_solids':976,
        'saved_thumb_shape_symmetric_difference_mm3':symmetric_difference,'other_objects_unchanged':unchanged,'serialization_difference_BRep_rechecks':boolean_fallbacks,
        'scope':'Minimum subassembly straight-tool access, source subtraction and saved-hand consistency. Not whole-hand servicing, screw tightening, nut retention or FDM strength.',
        'manufacturing_release':False}
text=json.dumps(result,indent=2)
if args.output:
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text,encoding='utf-8')
print(text)

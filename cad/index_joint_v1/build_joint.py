"""One-axis CAD candidate. FreeCAD 1.0 Python. No changes to hand24 baseline."""
import FreeCAD as App,Part,Mesh,json,math,sys,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent; PROJ=ROOT.parent.parent
V=App.Vector
SPEC=json.loads((ROOT/'joint_hardware_spec.json').read_text())
from wire_harness import add_harness
def placed(s,rot=None,xyz=(0,0,0)):
 t=s.copy();t.Placement=App.Placement(V(*xyz),rot or App.Rotation()).multiply(s.Placement);return t
def bolt(r,L,head_r=1.75,head_h=1.2):
 # envelope only; no thread or drive. Under-head plane local z=0, tip z=-L.
 return Part.makeCylinder(r,L,V(0,0,-L)).fuse(Part.makeCylinder(head_r,head_h)).removeSplitter()
def nut(af=4.0,h=1.6):
 rad=af/math.sqrt(3);ps=[V(rad*math.cos(i*math.pi/3),rad*math.sin(i*math.pi/3),0) for i in range(6)];ps.append(ps[0]);s=Part.Face(Part.makePolygon(ps)).extrude(V(0,0,h));return s.cut(Part.makeCylinder(.8,h))
def holed_box(size,origin,holes,axis):
 s=Part.makeBox(*size,V(*origin))
 for p in holes:s=s.cut(Part.makeCylinder(1.2,20,V(*p),V(*axis)))
 return s.removeSplitter()
# All vendor coordinates are millimetres.
m=Part.read(str(ROOT/'sources/XL330.stp'))
h=placed(Part.read(str(ROOT/'sources/FPX330-H101.step')),xyz=(0,14,-8))
f=placed(Part.read(str(ROOT/'sources/FPX330-S101.step')),App.Rotation(V(0,0,1),-90),(10,-7.5,-8))
R=App.Rotation(V(0,1,0),90)
def hand(s):return placed(s,R,(8,0,0))
items=[]
def add(n,s,moving=False,role='purchased',note='',color=(.3,.35,.4)):
 items.append(dict(name=n,shape=s,moving=moving,role=role,note=note,color=color))
# Vendor STEP explicitly contains stock horn, idler, cap and retaining screws.
for i,s in enumerate(m.Solids):
 n={3:'Output_horn',9:'Horn_retaining_screw',10:'Idler_disc',11:'Idler_cap',12:'Idler_retaining_screw'}.get(i,'Motor_part_%02d'%i)
 moving=i in [3,9,10]
 add(n,hand(s),moving,note='Vendor STEP solid '+str(i),color=(.88,.62,.2) if moving else (.28,.31,.35))
add('FPX330_H101',hand(h),True,color=(.18,.47,.8))
rel=SPEC['frame_relief'];fixed_frame=hand(f).cut(Part.makeBox(*rel['size_mm'],V(*rel['origin_mm']))).removeSplitter()
add('FPX330_S101',fixed_frame,False,'custom','Vendor S101 derived with 10.1 x11.5mm cable access window; post-machining required, strength not certified.',(.2,.63,.47))
# Reversible bench fixture; not a final palm. 40 x 38 x 3 mm.
a=math.sqrt(32)
base_holes=[(sx*a,-7.5+sy*(7.5+a),-18) for sx in [-1,1] for sy in [-1,1]]
# PCD16 diagonal pattern around the two S101 centres: pick far corners.
base=holed_box((40,38,3),(-20,-30,-16),base_holes,(0,0,1))
for x in [-15,15]:base=base.cut(Part.makeCylinder(1.7,3,V(x,-26,-16)))
add('Bench_base',base,False,'custom','3 mm prototype plate; four M2 clearance holes from S101 CAD. No strength approval.',(.63,.67,.73))
# Rigid output tab, carrying no downstream motor yet. Four diagonal PCD16 holes.
coords=[(sx*a,sz*a) for sx in [-1,1] for sz in [-1,1]]
tab=holed_box((24,3,20),(-12,17,-10),[(x,16,z) for x,z in coords],(0,1,0))
stem=Part.makeBox(12,28,4,V(-6,20,-2));tab=tab.fuse(stem).removeSplitter()
add('Output_test_tab',tab,True,'custom','Prototype load arm, NOT an index finger or next-motor mount.',(.87,.5,.17))
# Frame-to-horn and frame-to-idler: actual counterbore seat 1.3 mm outboard of contact face.
for side in [-1,1]:
 for k,(xx,yy) in enumerate([(6,0),(-6,0),(0,6),(0,-6)]):
  z=7.8 if side==1 else -23.8
  s=placed(bolt(.95,4),App.Rotation(V(0,0,1),V(0,0,side)),(xx,yy,z))
  add(('Horn' if side==1 else 'Idler')+'_M2x4_TAP_'+str(k+1),hand(s),True,'fastener-envelope','PHS M2x4 TAP; schematic head/thread. Effective penetration 2.7 mm <= vendor max3 mm.',(.77,.78,.8))
# S101 -> body, left/right-end counterbores. Two screws per face.
for side in [-1,1]:
 for k,y in enumerate([7.5,-22.5]):
  z=4.8 if side==1 else -20.8
  s=placed(bolt(.95,8),App.Rotation(V(0,0,1),V(0,0,side)),(8,y,z))
  add('Body_M2x8_TAP_%s_%s'%(side,k),hand(s),False,'fastener-envelope','Vendor M2x8 TAP body fixing; front 3.2 / rear 2.2mm into main-body pilot beyond cover clearance. Envelope only.',(.77,.78,.8))
# M2x6 machine bolts + nominal AF4 x1.6 nuts. Must verify supplied nut fit physically.
for k,(x,y,_) in enumerate(base_holes):
 add('Base_M2x6_'+str(k+1),placed(bolt(.98,SPEC['machine_screw']['length_mm'],SPEC['machine_screw']['head_diameter_mm']/2,SPEC['machine_screw']['head_height_mm']),xyz=(x,y,-11.8)),False,'fastener-envelope',color=(.8,.8,.83))
 # Nut is OUTSIDE prototype plate. Small screw head seats in existing recess.
 add('Base_M2_nut_'+str(k+1),placed(nut(),xyz=(x,y,-17.6)),False,'fastener-envelope','External AF4 x1.6 nominal nut. NBK SNZS-M2-6 selected: nominal head dia3 x0.6 mm.',(.68,.7,.74))
for k,(x,z) in enumerate(coords):
 add('Tab_M2x6_'+str(k+1),placed(bolt(.98,SPEC['machine_screw']['length_mm'],SPEC['machine_screw']['head_diameter_mm']/2,SPEC['machine_screw']['head_height_mm']),App.Rotation(V(0,0,1),V(0,-1,0)),(x,15.8,z)),True,'fastener-envelope',color=(.8,.8,.83))
 add('Tab_M2_nut_'+str(k+1),placed(nut(),App.Rotation(V(0,0,1),V(0,1,0)),(x,20,z)),True,'fastener-envelope','External nut y20..21.6. Head inserted from inner hinge BEFORE mounting motor. NBK SNZS-M2-6 nominal head dia3 x0.6 selected.',(.68,.7,.74))
add_harness(SPEC,add,placed,bolt,nut)
# CAD document preserves separate purchased, custom and fastener parts.
doc=App.newDocument('IndexRootJoint_v1');fixed=doc.addObject('App::Part','Fixed');moving=doc.addObject('App::Part','Moving')
fixed.Label='FIXED - motor / support / bench plate';moving.Label='MOVING - horn / idler disc / hinge / load tab'
moving.addProperty('App::PropertyAngle','Flexion','Joint');moving.Flexion=0
moving.addProperty('App::PropertyString','Instructions','Joint');moving.Instructions='Run set_joint_angle.FCMacro to move this rigid group about +X; CAD candidate -20..100deg. See cable_validation.json for fixed-lead coverage and connector mating caveats.'
objects=[];meshes={}
for it in items:
 s=it['shape'];obj=doc.addObject('PartDesign::Feature',it['name']);obj.Shape=s
 for key in ['role','note']:
  obj.addProperty('App::PropertyString',key.capitalize(),'Design');setattr(obj,key.capitalize(),it[key])
 (moving if it['moving'] else fixed).addObject(obj);objects.append(obj)
 vs,tri=s.tessellate(.2);meshes[it['name']]={'vertices':[list(v) for v in vs],'triangles':tri,'moving':it['moving'],'color':it['color'],'role':it['role']}
for it in items:
 if it['role']=='custom':it['shape'].exportStep(str(ROOT/(it['name']+'.step')))
# Independent part files for source CAD, neutral assembly and folded review.
doc.recompute();doc.saveAs(str(ROOT/'index_root_joint.FCStd'));Part.export(objects,str(ROOT/'index_root_joint.step'))
fold=[]
for it in items:fold.append(placed(it['shape'],App.Rotation(V(1,0,0),70)) if it['moving'] else it['shape'])
Part.makeCompound(fold).exportStep(str(ROOT/'index_root_joint_70deg.step'))
(ROOT/'assembly_meshes.json').write_text(json.dumps(meshes))
manifest={'status':'CAD interface candidate; not manufacturing release','axis':{'name':'index_mcp_flex','xyz':[1,0,0],'origin_mm':[0,0,0],'positive':'finger +Y bends toward palm +Z','requested_deg':[-20,100]},'vendor_coordinate_transform':{'R':[[0,0,1],[0,1,0],[-1,0,0]],'p_mm':[8,0,0]},'revision':2,'hardware_spec_file':'joint_hardware_spec.json','new_parts':['Bench_base','Output_test_tab','FPX330_S101','Cable_guide'],'base_holes_mm':base_holes,'tab_holes_xz_mm':coords,'parts':[{'name':it['name'],'moving':it['moving'],'role':it['role'],'valid':it['shape'].isValid(),'solids':len(it['shape'].Solids),'volume_mm3':it['shape'].Volume} for it in items],'open_items':['Final palm and next-joint mounting not designed.','M2 nuts and heads are nominal envelopes, no thread/contact/friction model. NBK SNZS-M2-6 dimensions fixed; procurement lead time/assembly preload not verified.','Dorsal header exits axially toward -X, with windowed S101. EHR-3 outer-box mating leaves a small envelope/case intersection; actual housing and cable alignment need physical fit check.','No stiffness, fatigue, insert pull-out or printed fit tests.','Idler cap and retaining screw treated as fixed; vendor bearing friction unmeasured.','No final link inertia or motor dynamics; do not reuse old hand route clearance claims.']}
(ROOT/'joint_manifest.json').write_text(json.dumps(manifest,indent=2));print('BUILT',len(items),'separate CAD parts; valid',all(i['valid'] for i in manifest['parts']),flush=True)

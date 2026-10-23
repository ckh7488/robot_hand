import sys,json,math
from pathlib import Path
import FreeCAD as A,Part,Mesh
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent;V=A.Vector

def box(x,y,z,p):return Part.makeBox(x,y,z,V(*p))
def hexcut(s,x,y,z,af,h):
 r=af/math.sqrt(3);p=[V(x+r*math.cos(i*math.pi/3),y+r*math.sin(i*math.pi/3),z) for i in range(6)];q=Part.Face(Part.makePolygon(p+[p[0]])).extrude(V(0,0,h));return s.cut(q)
assets={};rows=[]
for name,afs,hole,depth in [('M2_nut_fit',[4.2,4.3,4.4],2.4,2.2),('M3_nut_fit',[5.8,5.9,6.0],3.4,2.6)]:
 s=box(66,18,5,(0,0,0))
 for i,af in enumerate(afs):
  x=11+22*i;s=s.cut(Part.makeCylinder(hole/2,7,V(x,9,-1)));s=hexcut(s,x,9,5-depth,af,depth)
 assets[name]=s.removeSplitter();rows.append({'asset':name,'left_to_right_AF_mm':afs,'depth_mm':depth,'hole_diameter_mm':hole,'purpose':'Printer fit only. Actual purchased nut dimensions and orientation must be checked.'})
s=box(79,51,3,(0,0,0))
for i,c in enumerate([.2,.3,.4]):
 x=3+25*i;out=box(21.6,49.6,.8,(x-2.3,1,3));cut=box(17+2*c,45+2*c,2,(x-c,3.3-c,2.9));s=s.fuse(out.cut(cut))
assets['Root_foot_seat_fit']=s.removeSplitter();rows.append({'asset':'Root_foot_seat_fit','left_to_right_per_side_clearance_mm':[.2,.3,.4],'male_dimensions_mm':[17,45,5],'locator_lip_height_mm':.8,'purpose':'Choose locating clearance with the same printer, material and orientation before a whole palm print.'})
s=box(17,45,5,(0,0,0))
for y in [10,38]:
 s=s.cut(Part.makeCylinder(1.7,7,V(9,y,-1)));s=hexcut(s,9,y,2.4,5.9,2.6);s=s.cut(box(10,6.9,2.6,(9,y-3.45,2.4)))
assets['Root_foot_test_male']=s.removeSplitter();rows.append({'asset':'Root_foot_test_male','purpose':'Foot cross-section and bolt-hole/nut fit only; does not test cartridge strength.'})
for n,s in assets.items():
 assert s.isValid() and len(s.Solids)==1,n;s.exportStep(str(D/(n+'.step')));s.exportStl(str(D/(n+'.stl')))
 vs,tt=s.tessellate(.05);Mesh.Mesh([[(vs[i].x,vs[i].y,vs[i].z) for i in f] for f in tt]).write(str(D/(n+'_fine.stl')))
(D/'coupon_spec.json').write_text(json.dumps({'units':'mm','manufacturing_release':False,'test_parts':rows,'notes':['Print coupons first with the intended printer/material/line width.','Do not force nuts into undersized pockets; record fit and measured dimensions.','Side-open nut pockets prevent rotation but are not retention clips; keep nuts seated during assembly with removable tape or a separately tested retainer.','No servo, load bearing or dynamic qualification is implied.']},indent=2));print('BUILT',len(assets),'fit coupons')

"""New CAD-first structural packaging candidate. FreeCAD 1.0 Python, millimetres.
No prior trajectory or servo validation transfers to this new geometry.
"""
import sys,json,math,hashlib
from pathlib import Path
import FreeCAD as App
import Part,Mesh
sys.stdout.reconfigure(encoding='utf-8')
D=Path(__file__).resolve().parent;P=D.parent.parent
C=json.loads((D/'design.json').read_text(encoding='utf-8'));V=App.Vector
MODULAR=set(C.get('assembly',{}).get('modular_root_fingers',[]))
TOP_THUMB=C.get('assembly',{}).get('thumb_top_cartridge',False)
WALL_TOP=C.get('assembly',{}).get('root_wall_top_local_z_mm',4)
SEAT_X_MAX=C.get('assembly',{}).get('MCP_seat_positive_x_mm',-1.6)
SEAT_X_MIN=C.get('assembly',{}).get('MCP_seat_negative_x_mm',-27.4)
ID=App.Rotation()
def centroid(s):
 total=sum(a.Volume for a in s.Solids)
 return [sum(a.Volume*getattr(a.CenterOfMass,axis) for a in s.Solids)/total for axis in ['x','y','z']] if total>0 else None
def R(axis,angle):return App.Rotation(V(*axis),angle)
def rpy(v):return R((0,0,1),v[2]).multiply(R((0,1,0),v[1])).multiply(R((1,0,0),v[0]))
def pl(p=(0,0,0),r=None):return App.Placement(V(*p),r or ID)
def move(s,t):
 s=s.copy();s.Placement=t.multiply(s.Placement);return s
def box(x,y,z,p):return Part.makeBox(x,y,z,V(*p))
def union(ss):
 s=ss[0]
 for t in ss[1:]:s=s.fuse(t)
 return s.removeSplitter()
def prism(points,vec):return Part.Face(Part.makePolygon([V(*p) for p in points]+[V(*points[0])])).extrude(V(*vec))
def poly_xy(points,z,t):return prism([(x,y,z) for x,y in points],(0,0,t))
def offset_poly(points,d):
 out=[]
 for i,p in enumerate(points):
  a=points[i-1];b=points[(i+1)%len(points)]
  u=(p[0]-a[0],p[1]-a[1]);v=(b[0]-p[0],b[1]-p[1]);lu=math.hypot(*u);lv=math.hypot(*v)
  q=(p[0]+d*u[1]/lu,p[1]-d*u[0]/lu);r=(p[0]+d*v[1]/lv,p[1]-d*v[0]/lv)
  det=u[0]*v[1]-u[1]*v[0];t=((r[0]-q[0])*v[1]-(r[1]-q[1])*v[0])/det
  out.append((q[0]+t*u[0],q[1]+t*u[1]))
 return out
def hole(s,p,axis=(0,0,1),rad=1.2,length=20):return s.cut(Part.makeCylinder(rad,length,V(*p),V(*axis)))
def hexhole(s,p,axis=(0,0,1),af=4.3,depth=2.2,clock=0):
 rr=af/math.sqrt(3);sh=poly_xy([(rr*math.cos(i*math.pi/3),rr*math.sin(i*math.pi/3)) for i in range(6)],0,depth)
 return s.cut(move(sh,pl(p,App.Rotation(V(0,0,1),V(*axis)).multiply(R((0,0,1),clock)))))
a=math.sqrt(32)
mount_xy=[(sx*a-14.5,-7.5+sy*(7.5+a)) for sx in [-1,1] for sy in [-1,1]]
out_xz=[(sx*a-14.5,sz*a) for sx in [-1,1] for sz in [-1,1]]
def plate_screws(s):
 for x,y in mount_xy:
  s=hole(s,(x,y,-20),length=10)
  s=hexhole(s,(x,y,-18),depth=2.2)
 return s.removeSplitter()
def fixed_wall():return plate_screws(box(WALL_TOP+34,42,5,(-34,-31,-18)))
def root_cartridge():
 rz=C['hand']['fingers'][0]['root'][2];fz=C['hand']['palm_floor_z']+5-rz
 ss=[move(fixed_wall(),pl(r=R((0,1,0),-90))),box(17,45,5,(16,-34,fz))]
 for yy in [-29,6]:ss.append(prism([(18,yy,fz),(26,yy,fz),(18,yy,fz+18)],(0,4,0)))
 out=union(ss)
 for yy in [-24,4]:
  out=hole(out,(25,yy,fz-7),rad=1.7,length=15)
  out=hexhole(out,(25,yy,fz+2.4),af=5.9,depth=2.6)
  out=out.cut(box(10,6.9,2.6,(25,yy-3.45,fz+2.4)))
 # Straight insertion channels keep the gussets clear of the captive M2 nuts.
 for xx,yy in mount_xy:
  rad=4.3/math.sqrt(3)
  ch=poly_xy([(xx+rad*math.cos(i*math.pi/3),yy+rad*math.sin(i*math.pi/3)) for i in range(6)],-35,19.2)
  out=out.cut(move(ch,pl(r=R((0,1,0),-90))))
 return out.removeSplitter()
def connector(L,spread=False):
 # Back plate supports the next motor's S101 base; no 6mm isolated rail.
 b=box(32,L+11-21,5,(-30.5,21,-18))
 if spread:
  face=box(20,5,34,(-10,17,-30))
  coords=[(-z,x) for x,z in out_xz]
 else:
  face=box(30,5,28,(-29.5,17,-18));coords=out_xz
 for x,z in coords:
  face=hole(face,(x,16,z),(0,1,0),length=8)
  face=hexhole(face,(x,19.8,z),(0,1,0),depth=2.2)
 # Broad 45 degree inner corner, printed in the link's side plane.
 g=prism([(-29,22,-13),(-29,22,-10),(-29,25,-13)],(28,0,0))
 s=union([b,face,g])
 for x,y in mount_xy:
  s=hole(s,(x,L+y,-20),length=10)
  s=hexhole(s,(x,L+y,-18),depth=2.2)
 return s.removeSplitter()
def compact_root(h):
 # Front root hinge -> C-shaped stator seat. No long fore-aft spacer.
 # The seat lies between the MCP hinge cheeks. Its return arm crosses
 # the shaft plane behind the swept hinge, preserving flexion clearance.
 s=union([box(SEAT_X_MAX-SEAT_X_MIN,42,5,(SEAT_X_MIN,-31,h-18)),box(43,15,5,(-28,-31,h-18)),box(10,34,5,(5,-16,h-18)),box(10,5,h-9,(5,17,-3)),box(20,5,27,(-10,17,-30))])
 for x,z in [(-z,x) for x,z in out_xz]:
  s=hole(s,(x,16,z),(0,1,0),length=8)
  s=hexhole(s,(x,19.8,z),(0,1,0),depth=2.2)
 for x,y in mount_xy:
  s=hole(s,(x,y,h-20),length=10)
  s=hexhole(s,(x,y,h-18),depth=2.2)
 # 45-degree front upper bevel clears the MCP hinge in extension.
 # The screw row at Y5.657 retains the full 5mm seat thickness.
 bevel=prism([(-29,7,h-13),(-29,10,h-19),(-29,12,h-19),(-29,12,h-12),(-29,7,h-12)],(29,0,0))
 s=s.cut(bevel)
 # Root hinge is fixed before the next S101/motor: two axial screw-head access ports.
 # The positive bridge retains8mm depth around the open service slot.
 s=s.fuse(box(10,8,3,(5,-4,h-21)))
 for xx in [-6,6]:s=hole(s,(xx,0,-40),rad=2.2,length=110)
 return s.removeSplitter()
def tip(L):
 s=box(27,L-17,22,(-28,17,-11))
 try:s=s.makeFillet(1.5,[e for e in s.Edges if abs(e.Length-(L-17))<1e-4])
 except Exception:pass
 # Open dorsal pocket reduces material without removing the mounting face.
 if L>28:s=s.cut(box(17,L-28,8,(-23,26,-11)))
 # No grasping load: hollow the tip while preserving the nut-loading face.
 if L>28.6:s=s.cut(box(21.8,L-28.6,16.8,(-25.4,26,-8.4)))
 for x,z in out_xz:
  s=hole(s,(x,16,z),(0,1,0),length=9)
  s=hexhole(s,(x,19.8,z),(0,1,0),depth=L-19.1)
 return s.removeSplitter()
# Canonical motor frame: shaft +X, horn front X0, actual STEP, not a box.
v=Part.read(str(P/'sources/XL330.stp'));norm=pl((-6.5,0,0),R((0,1,0),90))
M=move(v,norm)
h=move(Part.read(str(P/'cad/index_joint_v1/sources/FPX330-H101.step')),pl((0,14,-8)))
H=move(h,norm)
# Reuse cable-relieved S101 from the one-axis interface study.
S=move(Part.read(str(P/'cad/index_joint_v1/FPX330_S101.step')),pl((-14.5,0,0)))
assets={'M077_stator':Part.makeCompound([move(s,norm) for i,s in enumerate(v.Solids) if i not in [3,9,10]]),'M077_rotor':Part.makeCompound([move(s,norm) for i,s in enumerate(v.Solids) if i in [3,9,10]]),'H101_vendor':H,'S101_relief':S}
asset_roles={'M077_vendor':'vendor_motor','H101_vendor':'vendor_hinge','S101_relief':'vendor_frame_modified'}
color={'motor':(.27,.31,.37),'hinge':(.22,.38,.56),'printed':(.76,.80,.85),'palm':(.77,.81,.86),'thumb':(.22,.58,.64),'cover':(.89,.90,.88),'arm':(.36,.43,.52),'ak':(.23,.25,.29),'pad':(.86,.66,.45)}

# Explicit assembly hardware: M2x6 seated in factory counterbores: effective frame grip1.2mm + printed grip2.8mm.
# Model low heads and captive nuts; threads are geometric envelopes, not helices.
color['hardware']=(.52,.54,.58)
def nut_local_z():
 rr=4/math.sqrt(3)
 q=poly_xy([(rr*math.cos(i*math.pi/3),rr*math.sin(i*math.pi/3)) for i in range(6)],0,1.6)
 return hole(q,(0,0,-1),rad=1.0,length=4)
def screw_local_z():
 # Underhead plane Z0; 6mm shaft toward -Z, head0.6mm toward +Z.
 return union([Part.makeCylinder(1,6,V(0,0,-6)),Part.makeCylinder(1.5,.6,V(0,0,0))])
base_s=[];base_n=[];out_s=[];out_n=[];out_root_n=[]
for x,y in mount_xy:
 base_s.append(move(screw_local_z(),pl((x,y,-11.8))))
 base_n.append(move(nut_local_z(),pl((x,y,-17.4))))
for x,z in out_xz:
 # Head seats at the factory Y15.8 counterbore, pointing through the H101 and printed carrier toward +Y.
 out_s.append(move(screw_local_z(),pl((x,15.8,z),R((1,0,0),90))))
 out_n.append(move(nut_local_z(),pl((x,19.8,z),R((1,0,0),-90))))
 root_n=move(nut_local_z(),pl(r=R((0,0,1),90)))
 out_root_n.append(move(root_n,pl((x,19.8,z),R((1,0,0),-90))))
for name,ss in [('M2_base_screws',base_s),('M2_base_nuts',base_n),('M2_output_screws',out_s),('M2_output_nuts',out_n),('M2_root_output_nuts',out_root_n)]:
 assets[name]=Part.makeCompound(ss)

# Purchased frame body and horn/idler assembly TAP screw envelopes.
# Raw vendor coordinates are normalized with the same motor datum as the STEP.
body_tap=[];hinge_tap=[]
def tap(L):return union([Part.makeCylinder(.95,L,V(0,0,-L)),Part.makeCylinder(1.75,1.2)])
for side in [-1,1]:
 for xx,yy in [(6,0),(-6,0),(0,6),(0,-6)]:
  seat=(7.9 if side==1 else -23.9) if yy==-6 else (7.8 if side==1 else -23.8)
  q=move(tap(4),pl((xx,yy,seat),App.Rotation(V(0,0,1),V(0,0,side))))
  hinge_tap.append(move(q,norm))
 for yy in [7.5,-22.5]:
  q=move(tap(8),pl((8,yy,4.8 if side==1 else -20.8),App.Rotation(V(0,0,1),V(0,0,side))))
  body_tap.append(move(q,norm))
assets['M2_body_TAP8']=Part.makeCompound(body_tap)
assets['M2_hinge_TAP4']=Part.makeCompound(hinge_tap)

# New tapered palm, not the previous rectangular plate with a short cover.
outline=[(-48,0),(48,0),(82,22),(116,65),(116,120),(108,182),(48,195),(-16,195),(-68,187),(-95,178),(-116,110),(-88,58),(-68,24)]
inner=[(-43,5),(43,5),(77,27),(111,67),(111,119),(103,178),(47,190),(-15,190),(-66,182),(-91,174),(-111,111),(-83,60),(-63,27)]
floor_z=C['hand']['palm_floor_z'];floor_top=floor_z+5;cover_z=22
palm=poly_xy(outline,floor_z,5)
# Hollow lower shell: walls terminate before the finger knuckle sweep.
rim=poly_xy(outline,floor_top,cover_z-floor_top).cut(poly_xy(inner,floor_top-1,cover_z-floor_top+2))
rim=rim.common(box(500,118,120,(-250,0,-80)))
pieces=[palm,rim]
root_frames={};root_mounts=[]
for f in C['hand']['fingers']:
 if f['name']=='thumb':continue
 x,y,z=f['root'];T=pl((x,y,z),R((0,1,0),-90));root_frames[f['name']]=T
 if f['name'] in MODULAR:
  root_mounts.extend([(x+25,y+yy) for yy in [-24,4]])
  continue
 pieces.append(move(fixed_wall(),T))
 for yy in [y-29,y+6]:
  pieces.append(prism([(x+18,yy,floor_top),(x+26,yy,floor_top),(x+18,yy,floor_top+18)],(0,4,0)))

# Low, broad ribs preserve open cable volume.
pieces.extend([box(82,4,9,(-62,30,floor_top)),box(124,4,9,(-62,83,floor_top))])
palm=union(pieces)
for f in C['hand']['fingers']:
 if f['name']=='thumb':continue
 x,y,z=f['root'];palm=palm.cut(box(8,18,8,(x+4,y-18,floor_z-2)))
# Thumb root is a bolted internal cartridge, not an external cantilever.
tf=C['hand']['fingers'][-1];TB=rpy(tf['base_rpy']);TR=TB.multiply(R((0,1,0),-90));tp=pl(tf['root'],TR);root_frames['thumb']=tp
thumbwall=move(plate_screws(box(WALL_TOP+39,42,5,(-39,-31,-18))),tp)
thumb_xy=pl((tf['root'][0],tf['root'][1],0),TB)
thumb_foot_outline=[(16,-28),(26,-28),(29,-10),(29.5,0),(26,18),(20,18),(16,9)]
thumbfoot=move(poly_xy(thumb_foot_outline,floor_top,5),thumb_xy)
thumb_g=[]
for yy in [-26,6]:thumb_g.append(move(prism([(18,yy,floor_top),(26,yy,floor_top),(18,yy,floor_top+18)],(0,4,0)),thumb_xy))
thumb=union([thumbwall,thumbfoot]+thumb_g)
for xx,yy in mount_xy:
 rad=4.3/math.sqrt(3)
 ch=poly_xy([(xx+rad*math.cos(i*math.pi/3),yy+rad*math.sin(i*math.pi/3)) for i in range(6)],-35,19.2)
 thumb=thumb.cut(move(ch,tp))
# The wall passes through a real assembly slot; the flange bears below the floor.
slot=move(box(46,43.2,6.2,(-40,-31.6,-18.6)),tp)
palm=palm.cut(slot)
thumb_bolts=[]
for x,y in [(24,-14),(24,8)]:
 p=thumb_xy.multVec(V(x,y,0));thumb_bolts.append((p.x,p.y))
 nutp=poly_xy([(x+5.9/math.sqrt(3)*math.cos(i*math.pi/3),y+5.9/math.sqrt(3)*math.sin(i*math.pi/3)) for i in range(6)],floor_top+2.4,2.6)
 thumb=thumb.cut(move(nutp,thumb_xy))
 thumb=thumb.cut(move(box(10,6.9,2.6,(x,y-3.45,floor_top+2.4)),thumb_xy))
for x,y in thumb_bolts:
 palm=hole(palm,(x,y,floor_z-7),rad=1.7,length=17)
 thumb=hole(thumb,(x,y,floor_z-7),rad=1.7,length=40)
assets['Thumb_internal_cradle']=thumb.removeSplitter()
# A continuous structural cover follows the knuckle arc rather than a 47mm strip.
cover_outline=[(-48,0),(48,0),(80,24),(110,66),(110,111),(82,130),(10,140),(-38,134),(-103,119),(-106,110),(-83,61),(-63,28)]
cover=poly_xy(cover_outline,cover_z,4)
# Open, curved-in-plan thenar bay permits the moving thumb metacarpal to emerge.
bay=[(-42,-35),(24,-35),(30,10),(33,57),(7,120),(-80,110)]
thumb_bay=move(poly_xy(bay,-20,85),thumb_xy)
cover=cover.cut(thumb_bay)
# Relieve only the upper shell in this bay; the floor and load-bearing root wall remain.
palm=palm.cut(thumb_bay)
cover_holes=[(-39,15),(-62,48),(-86,98),(-40,106),(2,121)]
for x,y in cover_holes:
 boss=Part.makeCylinder(6,cover_z-floor_top,V(x,y,floor_top))
 palm=palm.fuse(boss)
 palm=hole(palm,(x,y,15),rad=1.7,length=12)
 palm=hexhole(palm,(x,y,17.4),af=5.9,depth=2.6)
 palm=palm.cut(box(6.9,7,2.7,(x-3.45,y,17.35)))
 cover=hole(cover,(x,y,cover_z-1),rad=1.7,length=7)
saddle_bolts=[(x,y) for x in [-32,8] for y in [8,28]]
for x,y in saddle_bolts:palm=hole(palm,(x,y,floor_z-2),rad=1.7,length=12)
# Remove low-stress floor panels while retaining a broad perimeter, four
# load rails, motor wall pads and complete bolted-interface lands.
def strip(a,b,w,z,h):
 dx=b[0]-a[0];dy=b[1]-a[1];n=math.hypot(dx,dy);nx=-dy*w/(2*n);ny=dx*w/(2*n)
 return poly_xy([(a[0]+nx,a[1]+ny),(b[0]+nx,b[1]+ny),(b[0]-nx,b[1]-ny),(a[0]-nx,a[1]-ny)],z,h)
cut_outline=[(x*.91,95+(y-95)*.91) for x,y in outline]
floor_cut=poly_xy(cut_outline,floor_z-1,7)
webs=[]
for f,wx in zip([f for f in C['hand']['fingers'] if f['name']!='thumb'],[8,-3,-20,-32]):
 x,y,z=f['root'];a0=(wx,22);b0=(x+18,y-15)
 floor_cut=floor_cut.cut(strip(a0,b0,16,floor_z-2,10))
 floor_cut=floor_cut.cut(box(31,54,10,(x+4,y-38,floor_z-2)))
 webs.append(strip(a0,b0,14,floor_top,14).cut(strip(a0,b0,8.4,floor_top-0.01,11.61)))
for x,y in cover_holes+saddle_bolts:
 floor_cut=floor_cut.cut(Part.makeCylinder(9,10,V(x,y,floor_z-2)))
# Thumb flange bearing face stays continuous.
floor_cut=floor_cut.cut(move(thumbfoot,pl((0,0,-5))))
palm=palm.cut(floor_cut)
webs=union(webs)
for f in C['hand']['fingers']:
 if f['name']=='thumb':continue
 x,y,z=f['root']
 webs=webs.cut(box(30,43,20,(x-14,y-29,-54.5)))
webs=webs.cut(move(box(30,43,20,(-14,-29,-54.5)),thumb_xy))
palm=palm.fuse(webs)
# Chamfered wall windows leave top/bottom rails and wide corner columns.
for ya,yb in [(34,58),(76,102)]:
 for xa,xb in [(-140,-64),(70,140)]:
  wind=prism([(xa,ya,-44),(xa,yb,-44),(xa,yb,-4),(xa,yb-6,2),(xa,ya+6,2),(xa,ya,-4)],(xb-xa,0,0))
  palm=palm.cut(wind)
# Non-load-bearing cover: 2.4mm skin, retaining 4mm edges and screw lands.
ci=[(-42,6),(42,6),(74,28),(104,68),(104,108),(78,124),(10,134),(-36,128),(-98,115),(-100,110),(-77,64),(-58,31)]
cpocket=poly_xy(ci,cover_z,1.6).cut(thumb_bay)
for x,y in cover_holes:cpocket=cpocket.cut(Part.makeCylinder(9.5,4,V(x,y,cover_z-1)))
cover=cover.cut(cpocket)
# Join the left cover boss land to the perimeter after floor pocketing.
palm=palm.fuse(strip((-86,98),(-111,98),16,floor_z,5))
# Re-drill after adding webs so no rib caps a mounting bore.
for x,y in thumb_bolts+saddle_bolts:palm=hole(palm,(x,y,floor_z-2),rad=1.7,length=100)
for x,y in cover_holes:palm=hole(palm,(x,y,15),rad=1.7,length=12)


# Straight tool corridors for root-frame installation BEFORE motors/cover.
# The required root-frame order is checked separately from the motion envelope.
for finger,AA in root_frames.items():
 if finger in MODULAR or (finger=='thumb' and TOP_THUMB):continue
 for x,y in mount_xy:
  cutter=Part.makeCylinder(1.75,180,V(x,y,-11.1),V(0,0,1))
  palm=palm.cut(move(cutter,AA))

# Nuts on the inside floor need top-down access before root modules.
for x,y in saddle_bolts:
 palm=hexhole(palm,(x,y,floor_top),af=6.1,depth=20)
# Two bottom body screws per root need access after seating the motor.
# The corridor follows the screw axis; no load wall is removed wholesale.
for finger,AA in root_frames.items():
 if finger in MODULAR or (finger=='thumb' and TOP_THUMB):continue
 for yy in [7.5,-22.5]:
  cutter=move(Part.makeCylinder(1.75,180,V(-28.6,yy,-8),V(-1,0,0)),AA)
  palm=palm.cut(cutter);thumb=thumb.cut(cutter)
assets['Thumb_internal_cradle']=thumb.removeSplitter()
# Reopen existing nut cavities after adding the taller ribs.
palm=palm.cut(slot)
for ff,AA in root_frames.items():
 if ff=='thumb' or ff in MODULAR:continue
 for x,y in mount_xy:
  rr=4.3/math.sqrt(3)
  q=poly_xy([(x+rr*math.cos(i*math.pi/3),y+rr*math.sin(i*math.pi/3)) for i in range(6)],-18,2.2)
  palm=palm.cut(move(q,AA))
# Cartridge bearing lands stay at floor-top; a shallow lip registers the printed foot.
for f in C['hand']['fingers']:
 if f['name'] not in MODULAR:continue
 x,y,z=f['root'];palm=palm.cut(box(17.6,45.6,90,(x+15.7,y-34.3,floor_top)))
 wall_clear=box(5.6,42.6,90,(x+12.7,y-31.3,floor_top));palm=palm.cut(wall_clear)
 lip=box(21.6,49.6,.8,(x+13.7,y-36.3,floor_top)).cut(box(17.6,45.6,2,(x+15.7,y-34.3,floor_top-.5)))
 lip=lip.cut(wall_clear);palm=palm.fuse(lip)
for x,y in root_mounts:palm=hole(palm,(x,y,floor_z-1),rad=1.7,length=14)
# Top-down thumb foot bears on the retained floor. Clear only material above it.
thumb_clear=move(poly_xy(offset_poly(thumb_foot_outline,.3),floor_top,90),thumb_xy)
thumb_wall_clear=move(box(5.6,42.6,90,(12.7,-31.3,floor_top)),thumb_xy)
palm=palm.cut(thumb_clear).cut(thumb_wall_clear)
tl=move(poly_xy(offset_poly(thumb_foot_outline,2.3),floor_top,.8).cut(poly_xy(offset_poly(thumb_foot_outline,.3),floor_top-.5,2)),thumb_xy)
tl=tl.cut(thumb_wall_clear).common(poly_xy(outline,floor_top,1))
palm=palm.fuse(tl)
assets['Root_cartridge']=root_cartridge()
assets['Palm_back_chassis']=palm.removeSplitter()
assets['Palm_front_cover']=cover.removeSplitter()
assets['Compact_root']=compact_root(C['hand']['root_axis_offset_z'])
for L in sorted({f['links'][i] for f in C['hand']['fingers'] for i in [0,1]}):
 assets['Flex_link_'+str(L)]=connector(L,False)
for L in sorted({f['links'][2] for f in C['hand']['fingers']}):assets['Tip_'+str(L)]=tip(L)
def convex_hull(points):
 pts=sorted(set(points))
 def cross(o,a,b):return(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
 lo=[]
 for p in pts:
  while len(lo)>=2 and cross(lo[-2],lo[-1],p)<=0:lo.pop()
  lo.append(p)
 hi=[]
 for p in reversed(pts):
  while len(hi)>=2 and cross(hi[-2],hi[-1],p)<=0:hi.pop()
  hi.append(p)
 return lo[:-1]+hi[:-1]
def thumb_metacarpal(L,yaw):
 # Angled MCP seat makes the upper thumb point along the fingers.
 T=pl((0,L,0),R((0,0,1),yaw))
 pts=[(-30.5,17),(1.5,17),(-30.5,24),(1.5,24)]
 for x,y in [(-34,-31),(4,-31),(4,11),(-34,11)]:
  q=T.multVec(V(x,y,0));pts.append((q.x,q.y))
 base=poly_xy(convex_hull(pts),-18,5)
 face=box(30,5,28,(-29.5,17,-18))
 # A broad thenar shell, fixed to this metacarpal, shields its rear seat.
 wing=box(8,21,5,(-37,24,-18))
 side=box(4,21,30,(-37,24,-13))
 roof=box(34,21,4,(-37,24,17))
 s=union([base,face,wing,side,roof])
 for x,z in out_xz:
  s=hole(s,(x,16,z),(0,1,0),length=8)
  s=hexhole(s,(x,19.8,z),(0,1,0),depth=2.2)
 for x,y in mount_xy:
  p=T.multVec(V(x,y,-20));s=hole(s,tuple(p),length=10)
  p=T.multVec(V(x,y,-18));s=hexhole(s,tuple(p),depth=2.2,clock=yaw)
  # Rotate the nut pocket with the angled MCP seat, matching the motor datum.

 # Keep the C-carrier side return clear at the metacarpal neck.
 s=s.cut(box(25,8.2,6,(3,16,-18.5)))
 return s.removeSplitter()
assets['Thumb_metacarpal']=thumb_metacarpal(tf['links'][0],tf['mcp_mount_yaw'])
# Broad arm structures below are packaging solids. AK flange bolt CAD is pending.
aC=C['arm'];wdev=-aC['wrist_to_palm'];wflex=wdev-aC['wrist_axis_offset'];elbow=wflex-aC['elbow_to_wrist_flex'];roll=elbow+aC['elbow_to_roll']
AK=json.loads((P/'robot_config.json').read_text(encoding='utf-8'))['arm']['motors']
arm_geoms=[]
def arm_obj(name,s,c='arm',role='arm_packaging',parent='arm'):
 assets[name]=s;arm_geoms.append(dict(name=name,asset=name,t=pl(),color=color[c],role=role,parent=parent))
def ak(name,p,rot):
 a=AK[name];s=Part.makeCylinder(a['diameter_m']*500,a['length_m']*1000,V(0,0,-a['length_m']*1000-4));arm_obj('AK_'+name,move(s,pl(p,rot)),'ak','arm_motor_nominal')
ak('elbow_flex',(0,elbow,0),R((0,1,0),90));ak('forearm_roll',(0,roll,0),R((1,0,0),-90));ak('wrist_flex',(0,wflex,0),R((0,1,0),90));ak('wrist_deviation',(aC['wrist_lateral_offset'],wdev,0),ID)
# Bench elbow stator fixture, NOT a modeled upper arm.
arm_obj('Elbow_fixed_cradle',union([box(140,8,130,(-70,elbow-64,-65)),box(6,108,104,(-60.25,elbow-56,-52))]))
# Elbow output -> roll stator: broad right-side box corner, no thin rod.
arm_obj('Elbow_roll_carrier',union([box(54,28,30,(0,elbow-14,-15)),box(6,roll-47-elbow,44,(48,elbow,-22)),box(100,6,92,(-46,roll-53,-46))]))
# Forearm hollow beam, 44 x42 outer, 5mm walls.
y0=roll+4;y1=wflex-47
beam=box(44,y1-y0,42,(-22,y0,-21)).cut(box(34,y1-y0+2,32,(-17,y0-1,-16)))
beam=union([beam,box(76,6,44,(-54,y1,-22)),Part.makeCylinder(44,6,V(-53,wflex,0),V(1,0,0))])
arm_obj('Forearm_box_beam',beam)
# Distal roll flange at the beam base, packaging stand-off only.
arm_obj('Roll_output_flange',move(Part.makeCylinder(32,4),pl((0,roll,0),R((1,0,0),-90))))
# Compact side-offset wrist. Motor cases avoid one another across X.
lat=aC['wrist_lateral_offset'];rear=-AK['wrist_deviation']['length_m']*1000-4
upper=box(70,wdev+14-(wflex-13),6,(0,wflex-13,6))
upper=upper.cut(Part.makeCylinder(20,10,V(lat,wdev,4)))
wrist=union([box(6,26,30,(0,wflex-13,-15)),upper,box(6,46,67.2,(64,wdev-32,-55.2)),box(68,64,6,(2,wdev-32,rear-6))])
# 2mm radial/axial case relief in the flexion output corner.
relief=Part.makeCylinder(28.5,49.2,V(lat,wdev,-51.2))
# Only relieve the output corner; the rear mounting face remains untouched.
corner=box(6,26,30,(0,wflex-13,-15)).common(relief)
wrist=wrist.cut(corner).removeSplitter()
arm_obj('Wrist_box_carrier',wrist)
# Output passes through the stationary upper aperture with a 4mm radial gap.
# Two fore-aft ribs and the palm shoe lie clear of the motor case envelopes.
endwall=box(48,6,92,(6,-6,-68))
# Rounded rectangular lightening opening with broad 14mm side columns.
window=box(20,10,34,(20,-8,-28))
try:window=window.makeFillet(3,[e for e in window.Edges if abs(e.Length-10)<1e-5])
except Exception:pass
endwall=endwall.cut(window)
saddle=union([Part.makeCylinder(16,18,V(lat,wdev,0)),box(48,-(wdev-12),6,(6,wdev-12,18)),box(6,-wdev,12,(12,wdev,18)),box(6,-wdev,12,(48,wdev,18)),endwall,box(64,36,5,(-44,0,-68))])
for x,y in saddle_bolts:saddle=hole(saddle,(x,y,-70),rad=1.7,length=11)
arm_obj('Palm_wrist_saddle',saddle)

# Cover hardware uses short M3x10 screws and side-loaded AF5.5x2.4 nuts.
cv_s=[];cv_n=[]
for x,y in cover_holes:
 screw=union([Part.makeCylinder(1.5,10,V(x,y,16)),Part.makeCylinder(2.75,3,V(x,y,26))])
 rr=5.5/math.sqrt(3)
 nut=poly_xy([(x+rr*math.cos(i*math.pi/3),y+rr*math.sin(i*math.pi/3)) for i in range(6)],17.5,2.4)
 nut=hole(nut,(x,y,17),rad=1.5,length=4)
 cv_s.append(screw);cv_n.append(nut)
assets['M3_cover_screws']=Part.makeCompound(cv_s)
assets['M3_cover_nuts']=Part.makeCompound(cv_n)

# Explicit M3x14 ties the 5mm flange and 5mm palm floor to a full nut.
for label,locations in [('thumb_mount',thumb_bolts),('saddle_mount',saddle_bolts)]:
 ss=[];nn=[]
 for x,y in locations:
  length,seat=(12,floor_z) if label=='thumb_mount' else (14,floor_z-5)
  ss.append(union([Part.makeCylinder(1.5,length,V(x,y,seat)),Part.makeCylinder(2.75,3,V(x,y,seat-3))]))
  rr=5.5/math.sqrt(3)
  angle=math.radians(tf['base_rpy'][2]) if label=='thumb_mount' else 0
  nut=poly_xy([(x+rr*math.cos(i*math.pi/3+angle),y+rr*math.sin(i*math.pi/3+angle)) for i in range(6)],floor_top+2.5 if label=='thumb_mount' else floor_top,2.4)
  nn.append(hole(nut,(x,y,floor_top-1),rad=1.5,length=10))
 assets['M3_'+label+'_screws']=Part.makeCompound(ss)
 assets['M3_'+label+'_nuts']=Part.makeCompound(nn)

# Two accessible M3x12 cartridge screws and prefitted nuts per modular finger.
rs=[];rn=[];rz=C['hand']['fingers'][0]['root'][2];fz=floor_top-rz;under=floor_z-rz
for yy in [-24,4]:
 rs.append(union([Part.makeCylinder(1.5,12,V(25,yy,under)),Part.makeCylinder(2.75,3,V(25,yy,under-3))]))
 rr=5.5/math.sqrt(3);nn=poly_xy([(25+rr*math.cos(i*math.pi/3),yy+rr*math.sin(i*math.pi/3)) for i in range(6)],fz+2.5,2.4)
 rn.append(hole(nn,(25,yy,fz+2),rad=1.5,length=4))
assets['M3_root_mount_screws']=Part.makeCompound(rs);assets['M3_root_mount_nuts']=Part.makeCompound(rn)
# Static asset export: real CAD drives STEP, STL and preview placements.
for name,s in assets.items():
 assert s.isValid(),name+' invalid'
 if True:
  s.exportStep(str(D/'parts'/(name+'.step')))
  # Keep a printable native local STL for actual hand printed components only.
  if not name.startswith(('M2_','M3_')) and name not in ['M077_vendor','M077_stator','M077_rotor','H101_vendor','S101_relief'] and not name.startswith(('AK_','Elbow_','Forearm_','Roll_','Wrist_','Palm_wrist_')):s.exportStl(str(D/'parts'/(name+'.stl')))
 vs,tt=s.tessellate(.24)
 Mesh.Mesh([[(vs[i].x*.001,vs[i].y*.001,vs[i].z*.001) for i in t] for t in tt]).write(str(D/'meshes'/(name+'.stl')))
print('assets',len(assets),flush=True)
def pose_items(pose):
 items=[]
 def add(n,key,t,c,role='printed',parent='palm'):
  items.append(dict(name=n,asset=key,t=t,color=color[c],role=role,parent=parent))

 def hardware(tag,t,kind,parent,frame,support,root_nut=False):
  sk='M2_base_screws' if kind=='base' else 'M2_output_screws'
  nk='M2_base_nuts' if kind=='base' else ('M2_root_output_nuts' if root_nut else 'M2_output_nuts')
  sn=tag+'_'+kind+'_screws';nn=tag+'_'+kind+'_nuts'
  add(sn,sk,t,'hardware','fastener',parent)
  items[-1]['mating_with']=[frame,support,nn]
  add(nn,nk,t,'hardware','fastener',parent)
  items[-1]['mating_with']=[support,sn]
  tn=tag+('_base_body_tap' if kind=='base' else '_output_hinge_tap')
  add(tn,'M2_body_TAP8' if kind=='base' else 'M2_hinge_TAP4',t,'hardware','fastener',parent)
  motor=tag+'_motor' if kind=='base' else tag+'_rotor'
  items[-1]['threaded_engagement_with']=[motor]
  items[-1]['mating_with']=[motor,frame]

 add('Palm_back_chassis','Palm_back_chassis',pl(),'palm')
 add('Palm_front_cover','Palm_front_cover',pl(),'cover')
 add('Palm_cover_screws','M3_cover_screws',pl(),'hardware','fastener')
 add('Palm_cover_nuts','M3_cover_nuts',pl(),'hardware','fastener')
 add('Thumb_internal_cradle','Thumb_internal_cradle',pl(),'thumb')
 for label,ob in [('thumb_mount','Thumb_internal_cradle'),('saddle_mount','Palm_wrist_saddle')]:
  add('Palm_'+label+'_screws','M3_'+label+'_screws',pl(),'hardware','fastener')
  items[-1]['mating_with']=['Palm_back_chassis',ob,'Palm_'+label+'_nuts']
  add('Palm_'+label+'_nuts','M3_'+label+'_nuts',pl(),'hardware','fastener')
  items[-1]['mating_with']=['Palm_back_chassis','Palm_'+label+'_screws']
 for f in C['hand']['fingers']:
  name=f['name'];B=rpy(f['base_rpy']);root=f['root'];q=pose[name]
  A=pl(root,B.multiply(R((0,1,0),-90)))
  if name in MODULAR:
   tag=name.capitalize()+'_root_cartridge';rr=pl(root,B)
   add(tag,'Root_cartridge',rr,'printed')
   add(name.capitalize()+'_root_mount_screws','M3_root_mount_screws',rr,'hardware','fastener')
   items[-1]['mating_with']=[tag,'Palm_back_chassis',name.capitalize()+'_root_mount_nuts']
   add(name.capitalize()+'_root_mount_nuts','M3_root_mount_nuts',rr,'hardware','fastener')
   items[-1]['mating_with']=[tag,name.capitalize()+'_root_mount_screws']
  add(name+'_root_motor','M077_stator',A,'motor','vendor_motor',name+'_root_stator')
  add(name+'_root_frame','S101_relief',A,'hinge','vendor_frame_modified',name+'_root_stator')
  hardware(name+'_root',A,'base',name+'_root_stator',name+'_root_frame','Thumb_internal_cradle' if name=='thumb' else (name.capitalize()+'_root_cartridge' if name in MODULAR else 'Palm_back_chassis'))
  rootout=A.multiply(pl(r=R((1,0,0),-q[0])))
  add(name+'_root_rotor','M077_rotor',rootout,'motor','vendor_rotor',name+'_spread')
  add(name+'_root_hinge','H101_vendor',rootout,'hinge','vendor_hinge',name+'_spread')
  hardware(name+'_root',rootout,'output',name+'_spread',name+'_root_hinge',name+'_spread_carrier',True)
  # Canonical local X -> hand +Z for the base sweep joint.
  carriage=rootout.multiply(pl(r=R((0,1,0),90)))
  add(name+'_spread_carrier','Compact_root',carriage,'thumb' if name=='thumb' else 'printed',parent=name+'_spread')
  A=carriage.multiply(pl((0,0,C['hand']['root_axis_offset_z'])))
  for i in range(3):
   tag=name+'_'+['mcp','pip','dip'][i];parent=name+'_'+str(i)
   add(tag+'_motor','M077_stator',A,'motor','vendor_motor',parent+'_stator')
   add(tag+'_fixed_frame','S101_relief',A,'hinge','vendor_frame_modified',parent+'_stator')
   support=name+'_spread_carrier' if i==0 else name+'_'+['mcp','pip'][i-1]+'_link'
   hardware(tag,A,'base',parent+'_stator',tag+'_fixed_frame',support)
   O=A.multiply(pl(r=R((1,0,0),q[i+1])))
   add(tag+'_rotor','M077_rotor',O,'motor','vendor_rotor',parent+'_output')
   add(tag+'_hinge','H101_vendor',O,'hinge','vendor_hinge',parent+'_output')
   hardware(tag,O,'output',parent+'_output',tag+'_hinge',tag+('_link' if i<2 else '_tip'))
   if i<2:
    L=f['links'][i]
    key='Thumb_metacarpal' if name=='thumb' and i==0 else 'Flex_link_'+str(L)
    add(tag+'_link',key,O,'thumb' if name=='thumb' else 'printed',parent=parent+'_output')
    A=O.multiply(pl((0,L,0)))
    if name=='thumb' and i==0:A=A.multiply(pl(r=R((0,0,1),f['mcp_mount_yaw'])))
   else:add(tag+'_tip','Tip_'+str(f['links'][i]),O,'pad',parent=parent+'_output')
 items+=arm_geoms
 return items
scenes={};checks={};doc=App.newDocument('AssemblyMotion_v9');objects=[]
for pname,pose in C['poses'].items():
 items=pose_items(pose);scene=[];bb=None
 for it in items:
  t=it['t'];s=move(assets[it['asset']],t);r=t.Rotation.toMatrix();pos=t.Base
  scene.append({k:v for k,v in it.items() if k!='t'}|dict(pos_mm=list(pos),R=[[getattr(r,'A%d%d'%(i+1,j+1)) for j in range(3)] for i in range(3)]))
  if it['role'] not in ['arm_packaging','arm_motor_nominal']:
   if bb is None:bb=App.BoundBox(s.BoundBox)
   else:bb.add(s.BoundBox)
  if pname=='Open':
   ob=doc.addObject('PartDesign::Feature',it['name']);ob.Shape=s
   ob.addProperty('App::PropertyString','Role','Design');ob.Role=it['role'];objects.append(ob)
 scenes[pname]=scene;checks[pname]={'hand_bbox_mm':[bb.XLength,bb.YLength,bb.ZLength]}
print('placements generated',flush=True)
doc.recompute();doc.saveAs(str(D/'hand_forearm_v9.FCStd'));Part.export(objects,str(D/'hand_forearm_v9.step'))
(D/'scene.json').write_text(json.dumps(scenes,indent=2))
report={'revision':C['revision'],'manufacturing_release':False,'handedness':'right','assets':{n:{'valid':s.isValid(),'solids':len(s.Solids),'volume_mm3':s.Volume,'com_mm':centroid(s),'bbox_mm':[s.BoundBox.XLength,s.BoundBox.YLength,s.BoundBox.ZLength]} for n,s in assets.items()},'poses':checks,'axis_locations_y_mm':{'elbow':elbow,'forearm_roll':roll,'wrist_flex':wflex,'wrist_deviation':wdev,'palm_origin':0},'known_limits':C['notes'],'geometry_checks_file':'geometry_review.json','old_results_valid_for_this_geometry':False}
(D/'cad_manifest.json').write_text(json.dumps(report,indent=2))
print('BUILT',len(objects),'placed parts;',len(assets),'unique assets',flush=True)

# Derived convention export; hardware polarity remains a bench measurement.
old=json.loads((P/'robot_config.json').read_text(encoding='utf-8'))
hand_names=[f['name']+'_'+j for f in C['hand']['fingers'] for j in (['base_sweep','opposition','mcp','ip'] if f['name']=='thumb' else ['spread','mcp','pip','dip'])]
contract={'status':'CAD convention only; safe joint limits and encoder calibration pending','units':'degree','handedness':C['handedness'],'coordinates':{'x':'thumb side of authoring right hand','y':'finger direction','z':'palm side'},'joint_order':[j['name'] for j in old['arm']['joints']]+hand_names,'cad_root_motor_axis_relation':-1,'hardware_encoder_direction_verified':False,'poses':{n:[0,0,0,0]+[q for f in C['hand']['fingers'] for q in pose[f['name']]] for n,pose in C['poses'].items()},'safe_joint_limits':None,'retarget_routes_reused':False}
contract['kinematic_layout']={'units':'mm','fingers':C['hand']['fingers'],'root_axis_offset_z':C['hand']['root_axis_offset_z']}
(D/'joint_convention.json').write_text(json.dumps(contract,indent=2))

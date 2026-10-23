"""Static single-motor harness and guide, millimetres. No electrical or flex-life validation."""
import FreeCAD as App,Part,math
V=App.Vector
def wire_shape(y,spec,radius_extra=0):
 w=spec['wire'];x0=w['start_x_mm'];xb=w['bend_start_x_mm'];yc=w['bend_center_y_mm'];z=w['z_mm'];rr=y-yc;half=math.sqrt(.5)
 p0=V(x0,y,z);p1=V(xb,y,z);pm=V(xb-rr*half,yc+rr*half,z);p2=V(xb-rr,yc,z);p3=V(xb-rr,w['end_y_mm'],z)
 path=Part.Wire([Part.makeLine(p0,p1),Part.Arc(p1,pm,p2).toShape(),Part.makeLine(p2,p3)])
 circle=Part.Wire([Part.makeCircle(w['insulation_od_max_mm']/2+radius_extra,p0,V(-1,0,0))])
 return path.makePipeShell([circle],True,False)
def add_harness(spec,add,placed,bolt,nut):
 c=spec['connector'];plug=Part.makeBox(*c['registered_outer_size_mm'],V(*c['registered_outer_origin_mm']))
 add('JST_EHR3_outer_envelope',plug,False,'connector-envelope','Public-drawing outer box, not a hollow exact housing. Case/key mating overlap must be physically checked.',(.85,.85,.78))
 for i,y in enumerate(spec['wire']['start_y_mm']):
  add('Cable_lane_'+str(i+1),wire_shape(y,spec),False,'wire-envelope','OD <=1.9mm; path fixed at motor. Lane color does NOT encode electrical pin order.',[(.8,.2,.18),(.16,.36,.7),(.85,.65,.12)][i])
 g=spec['guide'];s=Part.makeBox(*g['size_mm'],V(*g['origin_mm']))
 s=s.cut(Part.makeBox(8,8,5,V(-36.5,-30,-9.25)))
 for x in [-39,-27.8]:s=s.cut(Part.makeBox(1.8,3,10,V(x,-27.5,-14)))
 s=s.cut(Part.makeCylinder(1.7,12,V(-15,-26,-16))).removeSplitter()
 add('Cable_guide',s,False,'custom','Static fixture guide; lace one 2.5mm tie through slots. Keep motor lead fixed; no flex-life or stiffness approval.',(.38,.63,.67))
 add('Guide_M3x16',placed(bolt(1.48,16,2.75,3),xyz=(-15,-26,-6.5)),False,'fastener-envelope','SNS-M3-16 nominal envelope; M3 nut below bench plate.',(.75,.77,.8))
 # nut() bores nominal 0.8mm for M2; widen to M3 clearance/root surrogate.
 n=nut(5.5,2.4).cut(Part.makeCylinder(1.3,2.4))
 add('Guide_M3_nut',placed(n,xyz=(-15,-26,-18.4)),False,'fastener-envelope','AF5.5 x2.4 nominal; guide fixture only, underside bolt projects.',(.72,.74,.77))

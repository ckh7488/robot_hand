import FreeCAD as A,Part,Mesh,json,math
from pathlib import Path
D=Path(__file__).resolve().parent;V=A.Vector;rows=[]
# Four alternatives: port-side Z +/-8 and a rearward/forward90-degree fixed bend.
# Existing radius provision10..15mm is retained, not a supplier flex-life rating.
for side in [-1,1]:
 for direction in [-1,1,0]:
  tag=('negativeZ' if side<0 else 'positiveZ')+('_back' if direction<0 else '_forward' if direction>0 else '_outboard');shapes=[]
  z=8*side
  for i,y in enumerate([-11.5,-9,-6.5]):
   cy=-21.5 if direction<0 else 3.5;radius=abs(y-cy) if direction else 10;x0=-25.6;xb=-34.5;h=math.sqrt(.5)
   p0=V(x0,y,z);p1=V(xb,y,z);pm=V(xb-radius*h,cy-direction*radius*h,z);p2=V(xb-radius,cy,z);p3=V(xb-radius,-36 if direction<0 else 25,z)
   if direction==0:
    pm=V(xb-radius*h,y,z+side*radius*(1-h));p2=V(xb-radius,y,z+side*radius);p3=V(xb-radius,y,side*36)
   path=Part.Wire([Part.makeLine(p0,p1),Part.Arc(p1,pm,p2).toShape(),Part.makeLine(p2,p3)])
   wire=path.makePipeShell([Part.Wire([Part.makeCircle(1.25,p0,V(-1,0,0))])],True,False);shapes.append(wire)
  plug=Part.makeBox(7.1,9.5,3.8,V(-25.6,-13.75,6.1 if side>0 else -9.9))
  for i,s in enumerate([plug,*shapes]):
   name=tag+('_plug' if i==0 else '_wire'+str(i));s.exportStep(str(D/'parts'/(name+'.step')));Mesh.Mesh(s.tessellate(.05)).write(str(D/'parts'/(name+'.stl')))
  rows.append({'name':tag,'port_side_z':side,'bend_direction_y':direction,'wire_assets':[tag+'_wire'+str(i) for i in [1,2,3]],'plug_asset':tag+'_plug','catalog_mating_qualified':False})
(D/'route_templates.json').write_text(json.dumps({'wire_OD_mm':1.9,'radial_reserve_mm':.3,'bend_centerline_radius_mm':[10,12.5,15],'basis':'Routing templates only. No wire flexibility, continuity between motors or bending lifetime simulation.','templates':rows},indent=2),encoding='utf-8');print('Templates exported')

# Earlier bend onset alternatives retain10..15mm centerline radius.
# 3/5mm straight wire beyond housing is a packaging assumption, not a qualified strain relief.
for side in [-1,1]:
 for plane in ['back','outboard']:
  for stub in [3,5]:
   tag=('negativeZ' if side<0 else 'positiveZ')+'_short'+str(stub)+'_'+plane;shapes=[];z=8*side
   for i,y in enumerate([-11.5,-9,-6.5]):
    radius=10 if plane=='outboard' else y+21.5;x0=-25.6;xb=x0-stub;h=math.sqrt(.5);p0=V(x0,y,z);p1=V(xb,y,z)
    if plane=='back':pm=V(xb-radius*h,-21.5+radius*h,z);p2=V(xb-radius,-21.5,z);p3=V(xb-radius,-36,z)
    else:pm=V(xb-radius*h,y,z+side*radius*(1-h));p2=V(xb-radius,y,z+side*radius);p3=V(xb-radius,y,side*36)
    path=Part.Wire([Part.makeLine(p0,p1),Part.Arc(p1,pm,p2).toShape(),Part.makeLine(p2,p3)]);shapes.append(path.makePipeShell([Part.Wire([Part.makeCircle(1.25,p0,V(-1,0,0))])],True,False))
   plug=Part.makeBox(7.1,9.5,3.8,V(-25.6,-13.75,6.1 if side>0 else -9.9))
   for i,s in enumerate([plug,*shapes]):
    name=tag+('_plug' if i==0 else '_wire'+str(i));s.exportStep(str(D/'parts'/(name+'.step')));Mesh.Mesh(s.tessellate(.05)).write(str(D/'parts'/(name+'.stl')))
   rows.append({'name':tag,'port_side_z':side,'bend_direction_y':-1 if plane=='back' else 0,'wire_assets':[tag+'_wire'+str(i) for i in [1,2,3]],'plug_asset':tag+'_plug','straight_stub_mm':stub,'strain_relief_qualified':False,'catalog_mating_qualified':False})
(D/'route_templates.json').write_text(json.dumps({'wire_OD_mm':1.9,'radial_reserve_mm':.3,'bend_centerline_radius_mm':[10,12.5,15],'basis':'Fixed exit templates only. No connected flexible harness, mated-housing or strain relief qualification.','templates':rows},indent=2),encoding='utf-8')

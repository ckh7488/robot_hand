import FreeCAD as App,Part,json,math
from pathlib import Path
V=App.Vector;r=Path(__file__).resolve().parent;d=App.openDocument(str(r/'index_root_joint.FCStd'));o={x.Name:x.Shape for x in d.Objects if x.TypeId=='PartDesign::Feature'}
log=open(str(r/'cable_probe2.log'),'w',buffering=1)
def emit(v):log.write(json.dumps(v)+'\n')
plug=Part.makeBox(7.1,9.5,3.8,V(-11.1,-13.75,-9.9))
body=Part.makeCompound([s for n,s in o.items() if n.startswith('Motor_part') and n not in ['Motor_part_13','Motor_part_14']])
for name,t in [('case',body),('fixed',o['FPX330_S101'])]:emit({'plug_vs':name,'overlap':plug.common(t).Volume,'distance':plug.distToShape(t)[0]})
def arc(a,m,b):return Part.Arc(V(*a),V(*m),V(*b)).toShape()
def lead(y):
 R=y+21.5;h=math.sqrt(.5);p0=(-11.1,y,-8);p1=(-20,y,-8);p2=(-20-R,-21.5,-8);p3=(-20-R,-36,-8)
 e=[Part.makeLine(V(*p0),V(*p1)),arc(p1,(-20-R*h,-21.5+R*h,-8),p2),Part.makeLine(V(*p2),V(*p3))]
 path=Part.Wire(e);circle=Part.Wire([Part.makeCircle(.95,V(*p0),V(-1,0,0))]);return path.makePipeShell([circle],True,False)
for y in [-11.5,-9,-6.5]:
 w=lead(y);emit({'wire_y':y,'valid':w.isValid(),'case_overlap':w.common(body).Volume,'case_distance':w.distToShape(body)[0],'fixed_overlap':w.common(o['FPX330_S101']).Volume,'fixed_distance':w.distToShape(o['FPX330_S101'])[0]});w.exportStep(str(r/('probe2_wire_'+str(y)+'.step')))
plug.exportStep(str(r/'probe2_plug.step'))

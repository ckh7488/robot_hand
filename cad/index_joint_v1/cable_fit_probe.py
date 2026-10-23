import FreeCAD as App,Part,json,math,time
from pathlib import Path
V=App.Vector;r=Path(__file__).resolve().parent;d=App.openDocument(str(r/'index_root_joint.FCStd'));o={x.Name:x.Shape for x in d.Objects if x.TypeId=='PartDesign::Feature'}
log=open(str(r/'cable_probe.log'),'w',buffering=1)
def emit(x):log.write(json.dumps(x)+'\n')
plug=Part.makeBox(7.1,9.5,3.8,V(-3.6,-13.75,6.1))
body=Part.makeCompound([s for n,s in o.items() if n.startswith('Motor_part') and n not in ['Motor_part_13','Motor_part_14']])
for name,t in [('case',body),('fixed',o['FPX330_S101'])]:emit({'plug_vs':name,'overlap':plug.common(t).Volume,'distance':plug.distToShape(t)[0]})
def arc(a,m,b):return Part.Arc(V(*a),V(*m),V(*b)).toShape()
def lead(y):
 # Starts on EHR-3 terminal exit; first bend carries wire OUT of case.
 s=math.sqrt(.5);p0=(3.5,y,8);p1=(4.5,y,8);p2=(8.5,y,12);p3=(8.5,y,26);p4=(8.5,y-6,32);p5=(8.5,-34,32)
 edges=[Part.makeLine(V(*p0),V(*p1)),arc(p1,(4.5+4*s,y,12-4*s),p2),Part.makeLine(V(*p2),V(*p3)),arc(p3,(8.5,y-6+6*s,26+6*s),p4),Part.makeLine(V(*p4),V(*p5))]
 path=Part.Wire(edges);circle=Part.Wire([Part.makeCircle(.75,V(*p0),V(1,0,0))]);return path.makePipeShell([circle],True,False)
for y in [-11.5,-9,-6.5]:
 w=lead(y);emit({'wire_y':y,'valid':w.isValid(),'case_overlap':w.common(body).Volume,'case_distance':w.distToShape(body)[0]});w.exportStep(str(r/('probe_wire_'+str(y)+'.step')))
plug.exportStep(str(r/'probe_plug.step'))

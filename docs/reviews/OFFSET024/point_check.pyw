import sys
from pathlib import Path
sys.dont_write_bytecode=True
root=Path(sys.argv[1]);output=Path(sys.argv[2]);assert not output.exists();sys.path.insert(0,str(root/'cad/thumb_sheath_terminal_v2'))
import selected_model as m
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.BRep import BRep_Tool
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCP.gp import gp_Pnt
fresh=next(z['shape'] for z in m.build()[0] if z['name']=='distal_housing_keeper_1');row=next(z for z in m.p.read(root/'cad/thumb_sheath_terminal_v2/selected/generated/parts/scene.json')['parts'] if z['name']=='distal_housing_keeper_1');saved=m.c.load([row])[0]['shape'];pt=[99.28777459522642,71.2950282308043,41.866736144525426];vertex=m.cq.Vertex.makeVertex(*pt);result=[]
for name,shape in [('source',fresh),('STEP',saved)]:
    face=shape.Faces()[7];project=GeomAPI_ProjectPointOnSurf(gp_Pnt(*pt),BRep_Tool.Surface_s(face.wrapped));record=dict(kind=name,own_face_distance=vertex.distance(face),whole_shape_distance=vertex.distance(shape),face_classification_3d=str(BRepClass_FaceClassifier(face.wrapped,gp_Pnt(*pt),1e-7).State()),torus_face_distances=[dict(index=i,distance=vertex.distance(f)) for i,f in enumerate(shape.Faces()) if f.geomType()=='TORUS'],projected_distances=[project.Distance(i) for i in range(1,project.NbPoints()+1)]);result.append(record)
m.p.write(output,dict(completed=True,point=pt,results=result));print(result,flush=True)

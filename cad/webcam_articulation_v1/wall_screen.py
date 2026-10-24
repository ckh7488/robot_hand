"""Sample local BRep wall thickness; this is not a global minimum or FEA."""
import json
import math
import numpy as np
import cadquery as cq
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.TopAbs import TopAbs_IN
from OCP.gp import gp_Pnt2d,gp_Pnt,gp_Dir,gp_Lin
from manufacturing_audit import load,save,ROOT


def sample_thickness(shape):
    values=[]
    ray=IntCurvesFace_ShapeIntersector()
    ray.Load(shape.wrapped,1e-7)
    for fi,face in enumerate(shape.Faces()):
        if face.Area()<.25:continue
        u0,u1,v0,v1=face.uvBounds()
        if not all(math.isfinite(t) for t in (u0,u1,v0,v1)):continue
        for pu,pv in ((.5,.5),(.2,.2),(.2,.8),(.8,.2),(.8,.8)):
            u=u0+(u1-u0)*pu;v=v0+(v1-v0)*pv
            if BRepClass_FaceClassifier(face.wrapped,gp_Pnt2d(u,v),1e-7).State()!=TopAbs_IN:continue
            normal,point=face.normalAt(u,v)
            n=np.array(normal.toTuple());p=np.array(point.toTuple())
            # Verify which side is material instead of assuming face orientation.
            minus=shape.isInside(p-n*.0001,tolerance=1e-8)
            plus=shape.isInside(p+n*.0001,tolerance=1e-8)
            if minus==plus:continue
            inward=-n if minus else n
            origin=p-inward*.002
            ray.Perform(gp_Lin(gp_Pnt(*origin),gp_Dir(*inward)),0,50)
            distances=[]
            for hit in range(1,ray.NbPnt()+1):
                q=ray.Pnt(hit);t=float((np.array([q.X(),q.Y(),q.Z()])-origin)@inward)
                if t>=-1e-8:distances.append(t)
            ordered=[]
            for t in sorted(distances):
                if not ordered or t-ordered[-1]>1e-6:ordered.append(t)
            if len(ordered)<2 or abs(ordered[0]-.002)>1e-4:continue
            thickness=ordered[1]-ordered[0]
            if not shape.isInside(origin+inward*(ordered[0]+ordered[1])/2,tolerance=1e-8):continue
            values.append(dict(thickness_mm=thickness,face_index=fi,point_mm=p.tolist(),normal_into_material=inward.tolist()))
    return values


def main():
    slab=cq.Workplane('XY').box(20,20,2).val()
    sleeve=cq.Solid.makeCylinder(3.35,16).cut(cq.Solid.makeCylinder(2.2,16))
    selfchecks=[min(v['thickness_mm'] for v in sample_thickness(s)) for s in (slab,sleeve)]
    assert np.allclose(selfchecks,[2,1.15],atol=1e-6),selfchecks
    design,scene,parts,hashes=load()
    names=[n for n in parts if any(s in n for s in ('segment_shell','thenar_shell','tip_core','dark_faceplate'))]
    import hashlib
    from pathlib import Path
    hashes[str(Path(__file__).resolve().relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report=dict(input_sha256=hashes,completed=False,method='5 UV samples per trimmed face with area >=0.25mm2; material-side classification and first two positive ray intersections within 50mm',
        limitations='Sampled local normal thickness only; edges/small faces and unsampled areas excluded. No minimum-thickness guarantee or strength approval.',
        analytical_selfcheck_min_mm=selfchecks,parts=[])
    for name in names:
        values=sample_thickness(parts[name]);values.sort(key=lambda v:v['thickness_mm'])
        row=dict(name=name,samples=len(values),sampled_min_mm=values[0]['thickness_mm'] if values else None,
            below_point45_mm=sum(v['thickness_mm']<.45 for v in values),
            below_one_mm=sum(v['thickness_mm']<1 for v in values),thinnest_samples=values[:12])
        report['parts'].append(row);save('walls.json',report)
        print(name,'samples',len(values),'min',row['sampled_min_mm'],flush=True)
    report['completed']=True;save('walls.json',report)


if __name__=='__main__':main()

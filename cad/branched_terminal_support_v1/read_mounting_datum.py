"""Read actual M3 faces/material points;do not replace the failed whole Boolean."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm074_datum',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN,TopAbs_ON


def read(sh):
    faces=[]
    for i,f in enumerate(sh.Faces()):
        if f.geomType()!='CYLINDER':continue
        c=BRepAdaptor_Surface(f.wrapped).Cylinder();point=np.array(c.Axis().Location().Coord());axis=np.array(c.Axis().Direction().Coord())
        if abs(c.Radius()-1.7)<1e-5 and abs(point[0]-71.)<1e-5 and abs(point[2]-13.)<1e-5 and abs(axis[1])>.999:
            faces.append(dict(index=i,radius_mm=c.Radius(),point=point,axis=axis,bbox=n.c.m.pack._bbox(f),area_mm2=f.Area()))
    pad=m.a.a.cy(3.,-127.,-126.,71.,13.).cut(m.a.a.cy(1.7,-127.01,-125.99,71.,13.))
    def state(s,p):
        x=BRepClass3d_SolidClassifier(s.wrapped,gp_Pnt(*p),1e-7).State();return 'IN' if x==TopAbs_IN else 'ON' if x==TopAbs_ON else str(x)
    points=[]
    for y in [-126.75,-126.5,-126.25]:
        for r in [1.8,2.35,2.9]:
            for a in np.arange(24)*np.pi/12:
                p=np.array([71.+r*np.cos(a),y,13.+r*np.sin(a)]);points.append(dict(point=p,pad=state(pad,p),candidate=state(sh,p)))
    return faces,points


def main():
    out=HERE/'reviews/mounting_datum.json';assert not out.exists();q=n.p.read(HERE/'reviews/check.json');source={};sh=n.c.load(n.p.read(ROOT/q['scene'])['parts'],source)[0]['shape'];faces,points=read(sh)
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,actual_upper_M3_bore_faces=faces,material_points=points,point_count=len(points),all_selected_points_inside_both=all(z['pad']=='IN' and z['candidate']=='IN' for z in points),new_comparison_Boolean_operations=0,reference_pad_construction_cut_operations=1,
        initial_whole_pad_difference_preserved=q['attachment_queries'][-1],cause_confirmed=False,whole_pad_containment_approved=False,
        scope='Actual source M3 bore faces and 216 interior annulus samples,not full volume containment/alltrim equivalence. Preserve the failed full pad cut even if every sample is inside. No adaptive precision,old Boolean or geometry change.',input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items())))
    print('new upperM3 faces',len(faces),'points',len(points),'inside',sum(z['candidate']=='IN' for z in points),'wholeBoolean retained false',flush=True)


if __name__=='__main__':main()

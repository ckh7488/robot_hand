"""Read the original M2/M3 stack and saved failure without another Boolean."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm068_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    n,ROOT,HERE=m.n,m.ROOT,m.HERE;out=HERE/'reviews/survey.json';assert not out.exists()
    assert OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source=m.context();p=m.packets(rows)[0];lookup={z['name']:z for z in rows}
    report=[]
    for name in p['names']:
        z=lookup[name];cyl=[]
        if name.endswith(('_adapter','_cradle','_base_screws','_base_nuts','_rack_screw_1','_guide_spacer_1')):
            for face in z['shape'].Faces():
                if face.geomType()=='CYLINDER':
                    c=BRepAdaptor_Surface(face.wrapped).Cylinder()
                    cyl.append(dict(radius_mm=c.Radius(),axis=list(c.Axis().Direction().Coord()),point=list(c.Axis().Location().Coord()),bounds_mm=m.bbox([dict(shape=face)])))
        report.append(dict(name=name,bounds_mm=m.bbox([z]),solids=len(z['shape'].Solids()),cylinders=cyl))
    ev=n.p.read(ROOT/'cad/oriented_fastener_entry_v1/reviews/evidence_final.json')
    common=n.c.load(n.p.read(ROOT/ev['common_scene'])['parts'],source)
    for path in ['cad/form_drive_v1/actuator_packaging.py','cad/assembly_sequence_v2/final_sequence.py','cad/motor_tendon_terminal_v1/service.py']:
        source[path]=n.m.drive.sha(ROOT/path)
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    n.a.save(out,dict(completed=True,source_sha256=source,packet=p,parts=report,saved_failed_common=[dict(name=z['name'],bounds_mm=m.bbox([z]),solids=len(z['shape'].Solids()),volume_mm3=z['shape'].Volume()) for z in common],
        new_native_Boolean_queries=0,new_geometry=0,OCC_pool_threads=1,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items())))
    for z in report:print(z['name'],z['bounds_mm'].tolist(),'solids',z['solids'],flush=True)


if __name__=='__main__':main()

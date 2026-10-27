"""Read whole-interface bounds and the old failures without repeating their Booleans."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm074_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm074_meta',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py')
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/survey.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    metas,matrix=old.metadata();meta={z['name']:z for z in metas};source={};selected={}
    for name in [m.OLD,'arm059_straight_jaw_1','thumb_ip_barrel_adjuster_1','thumb_ip_barrel_top_nut_1','thumb_ip_barrel_bottom_nut_1']:
        sh=n.g.original.transform(n.c.load([meta[name]],source)[0]['shape'],matrix(name));selected[name]=sh
    old59=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json');s58=n.p.read(ROOT/'cad/motor_end_integration_basis_v1/reviews/survey.json')
    planes=[]
    for i,f in enumerate(selected[m.OLD].Faces()):
        if f.geomType()!='PLANE':continue
        a=BRepAdaptor_Surface(f.wrapped);point=np.array(a.Plane().Location().Coord());normal=np.array(a.Plane().Axis().Direction().Coord())
        if any(abs(point[0]-x)<1e-5 and abs(normal[0])>.999 for x in [94.2,97.2]) or any(abs(point[2]-z)<1e-5 and abs(normal[2])>.999 for z in [11.5,14.5]):
            planes.append(dict(index=i,point=point,normal=normal,area_mm2=f.Area(),bbox=n.c.m.pack._bbox(f)))
    sourcebase,emeta,EM=m.electrical(source)
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/integrated_motor_end_v1/reviews/check.json',ROOT/'cad/captured_adjuster_kit_v1/reviews/conflict_continuity.json',ROOT/'cad/motor_end_integration_basis_v1/reviews/survey.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    b={k:n.c.m.pack._bbox(v) for k,v in selected.items()}
    raw=dict(completed=True,source_sha256=source,actual_selected_bounds=b,old_shared_web_planes=planes,
        protected_electrical_max_X_mm=old59['design']['parameter_basis']['actual_electrical_max_X_mm'],actual_upper_jaw_max_Z_mm=b['arm059_straight_jaw_1'][1][2],
        electrical_base_source=emeta,electrical_base_transform=EM,electrical_base_bbox=n.c.m.pack._bbox(sourcebase),actual_electrical_frame=s58['electrical_frame_proposal'],original_M3_stack=s58['M3_nominal_stack'],
        old_port1_final_conflicts=n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/conflict_continuity.json')['existing_ARM059_port1_final_conflicts'] if 'existing_ARM059_port1_final_conflicts' in n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/conflict_continuity.json') else [z for z in old59['native']['new4__moved16']['overlaps'] if z['lead']==m.OLD],
        old_M3_pad_queries_read=old59['attachment_queries'][2:4],old_native_Boolean_repeated=0,new_geometry=0,OCC_pool_threads=1,
        interpretation='Upper common web at X94.2..97.2/Z11.5..14.5 occupies the retained upper barrel/nuts. Replace the entire upper M3-to-R5 load route via an inboard riser and a branch above the full actual jaw boundary;retain the lower route,original electrical base/groove/lidM2 and both M3 seats/ports. Three-mm branch is a candidate structural section,not a strength result or required clearance. Existing +13 pad false is preserved;new candidate containment is a separate query.')
    raw['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items());n.a.save(out,raw);print('whole interface',len(planes),'planes; upperboundary',raw['actual_upper_jaw_max_Z_mm'],'old Boolean0',flush=True)


if __name__=='__main__':main()

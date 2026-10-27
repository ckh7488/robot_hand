"""Read the capture lip and old stored common without repeating old Booleans."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm073_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm073_metadata',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py')
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/survey.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    old59=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json');Q=np.array(old59['design']['ports'][0]['Q']);Qi=np.linalg.inv(Q)
    metas,matrix=old.metadata();meta={z['name']:z for z in metas};source={};parts=[]
    for name in ['arm059_straight_jaw_0','thumb_ip_barrel_adjuster_0']:
        z=dict(meta[name]);sh=n.g.original.transform(n.c.load([z],source)[0]['shape'],matrix(name));local=n.g.original.transform(sh,Qi);faces=[]
        for i,f in enumerate(local.Faces()):
            s=BRepAdaptor_Surface(f.wrapped)
            if name=='arm059_straight_jaw_0' and f.geomType()=='CYLINDER':
                c=s.Cylinder();v=np.array(c.Axis().Direction().Coord())
                if abs(v[1])>1.-1e-10:faces.append(dict(index=i,type='CYLINDER',radius_mm=c.Radius(),axis=v,point=c.Axis().Location().Coord(),bbox=n.c.m.pack._bbox(f)))
            if f.geomType()=='PLANE':
                pl=s.Plane();v=np.array(pl.Axis().Direction().Coord())
                if abs(v[1])>1.-1e-10:faces.append(dict(index=i,type='PLANE',axis=v,point=pl.Location().Coord(),area_mm2=f.Area(),bbox=n.c.m.pack._bbox(f)))
        parts.append(dict(name=name,actual_transform=matrix(name),capture_frame_faces=faces))
    old72=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/continuation_common_prepared.json');stored=n.c.load(n.p.read(ROOT/old72['common_scene'])['parts'],source)[0]
    conflicts=[z for z in old59['native']['new4__moved16']['overlaps'] if z['lead']=='arm059_integrated_electrical_R5_base' and z['part'] in ['thumb_ip_barrel_adjuster_1','thumb_ip_barrel_top_nut_1','thumb_ip_barrel_bottom_nut_1']];assert len(conflicts)==3
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/integrated_motor_end_v1/reviews/check.json',ROOT/'cad/special_thumb_module_service_v1/reviews/continuation_common_prepared.json',ROOT/'cad/sheath_retention_v1/retention_model.py',ROOT/'cad/motor_tendon_terminal_v1/terminal_model.py',ROOT/'cad/forward_terminal_support_v1/model.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw=dict(completed=True,source_sha256=source,port0_frame=Q,actual_capture_parts=parts,stored_old_common_name=stored['name'],stored_old_common_volume_mm3=stored['shape'].Volume(),stored_old_common_bbox_in_capture_frame=n.c.m.pack._bbox(n.g.original.transform(stored['shape'],Qi)),old_common_Boolean_repeated=0,old_native_groups_repeated=0,
        existing_ARM059_port1_final_conflicts=conflicts,port1_existing_conflicts_retested=0,port1_original_geometry_restoration_not_claimed=True,new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,OCC_pool_threads=1,
        interpretation='Original split inner jaw retains a head pocket and shaft shoulder,plus a closed sleeve farther along the housing path. Insert the inner half radially from its open side around a manually held adjuster on a separate bench;then opposite jaw and M2 hardware. Test the original complete kit entering the original R5 bore with both barrel nuts preplaced. ARM059 already reports three final port1/base intersections,so an order change cannot be claimed to clear those unchanged final shapes. Read those records only,no closed31-group repeat or extra precision.')
    raw['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items());n.a.save(out,raw)
    print('capture survey',[(z['name'],len(z['capture_frame_faces'])) for z in parts],'oldcommon',raw['stored_old_common_volume_mm3'],'oldport1',[(z['part'],z['material_intersection_mm3']) for z in conflicts],flush=True)


if __name__=='__main__':main()

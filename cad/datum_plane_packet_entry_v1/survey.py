"""Read actual planes and the saved failed common,without old Booleans."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm071_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm071_metadata',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py')
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/survey.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    prior=n.p.read(ROOT/'cad/open_guide_module_assembly_v1/reviews/continuation_check.json');c=prior['module_contracts'][0];name=c['module'];assert name=='ring_pip_actuator'
    metas,matrix=old.metadata();metas={z['name']:z for z in metas};source={};lookup={}
    names=c['body19_names']+['arm065_shared_output_beam_rack_root']
    for name in names:
        z=dict(metas[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=n.m.drive.sha(ROOT/rel))
        lookup[name]=n.g.original.transform(n.c.load([z],source)[0]['shape'],matrix(name))
    saved=n.p.read(ROOT/'cad/open_guide_module_assembly_v1/reviews/common_prepared.json');meta=n.p.read(ROOT/saved['common_scene'])['parts'];oldcommon=next(z for z in n.c.load(meta,source) if z['name']=='arm070_failed_entry_common_3')
    P=np.array(c['placement']);normal=np.array(c['normal']);radial=np.array(c['radial']);records=[]
    for name in names:
        sh=lookup[name];faces=[]
        if name.endswith('_adapter') or name=='arm065_shared_output_beam_rack_root':
            for i,f in enumerate(sh.Faces()):
                if f.geomType()!='PLANE':continue
                plane=BRepAdaptor_Surface(f.wrapped).Plane();axis=np.array(plane.Axis().Direction().Coord())
                if abs(axis@normal)<1.-1e-10:continue
                faces.append(dict(index=i,plane_axis=axis,point=np.array(plane.Location().Coord()),area_mm2=f.Area(),bbox=n.c.m.pack._bbox(f)))
        records.append(dict(name=name,bbox=n.c.m.pack._bbox(sh),bbox_in_module_frame=n.c.m.pack._bbox(n.g.original.transform(sh,np.linalg.inv(P))),normal_plane_faces=faces))
    body=[dict(name=k,shape=lookup[k]) for k in c['body19_names']]
    result=dict(completed=True,source_sha256=source,module=c['module'],placement=P,radial=radial,normal=normal,body19_names=c['body19_names'],actual_parts=records,
        stored_old_common_name=oldcommon['name'],stored_old_common_volume_mm3=oldcommon['shape'].Volume(),stored_old_common_bbox=n.c.m.pack._bbox(oldcommon['shape']),
        stored_old_common_bbox_in_module_frame=n.c.m.pack._bbox(n.g.original.transform(oldcommon['shape'],np.linalg.inv(P))),
        body_final_bbox=m.bbox(body),body_failed_plus8_bbox=m.bbox(m.moved(body,m.translate(8.*normal))),
        original_ARM069_axis_evidence='cad/rear_packet_restoration_v1/reviews/evidence.json;read only,no old axis query repeated',
        old_native_groups_repeated=0,old_common_Boolean_repeated=0,new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,
        conclusion='The +8normal side-entry offset is an assembly pose,not the installed datum. Test the complete outside radial entry at original finalnormal0 with original body19 and all598 current obstacles;no shallow+4/0 suffix or cut. Planar property reads do not approve fit/contact/load.',OCC_pool_threads=1)
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/open_guide_module_assembly_v1/reviews/continuation_check.json',ROOT/'cad/open_guide_module_assembly_v1/reviews/common_prepared.json',ROOT/'cad/shared_roll_root_v1/model.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    result['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items());n.a.save(out,result)
    print('survey',len(records),'oldcommon',result['stored_old_common_bbox'],'modulecommon',result['stored_old_common_bbox_in_module_frame'],flush=True)
    for z in records:
        if z['normal_plane_faces']:print(z['name'],sorted(set(round(float(x['point']@normal),8) for x in z['normal_plane_faces'])),flush=True)


if __name__=='__main__':main()

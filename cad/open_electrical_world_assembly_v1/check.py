"""Read the actual obstruction, then try the open-guide body before its leads/lid."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm079_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    (HERE/'reviews').mkdir(exist_ok=True);started=time.monotonic();rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};final=set(lookup);boxes={k:m.corners(n.c.m.pack._bbox(z['shape'])) for k,z in lookup.items()}
    prior=n.p.read(ROOT/'cad/front_packet_restoration_v1/reviews/check.json');world=set(prior['packet_final_present_names']);assert len(world)==718
    p=next(z for z in packets if z['module']==m.MODULE);f=prior['module_contracts'][0];assert f['module']==m.MODULE and not set(p['names'])&world
    name=m.MODULE;radial=np.array(f['radial']);normal=np.array(f['normal']);opening=np.array(f['opening']);frame=np.array(f['guide_frame']);base=name+'_base_nuts';body19=set(f['body19_names']);leads={name+'_lead'+str(i) for i in [1,2,3]};lid=name+'_wire_guide_lid';screws={name+'_lid_screw_'+str(i) for i in [1,2]};nuts={name+'_lid_nut_'+str(i) for i in [1,2]};body13=body19-leads-{lid}-screws;assert len(body13)==13 and name+'_plug_outer' in body13
    inherited=m.inheritance(lookup);carrier=set(inherited['carrier11_names']);assert carrier|nuts==body13
    ev=n.p.read(ROOT/'cad/front_packet_restoration_v1/reviews/evidence.json');common=n.c.load(n.p.read(ROOT/ev['common_scene'])['parts'],source)
    common_props=[dict(name=z['name'],volume_mm3=z['shape'].Volume(),solids=len(z['shape'].Solids()),bbox=n.c.m.pack._bbox(z['shape']),center_mm=z['shape'].Center().toTuple()) for z in common]
    region=np.array([z['bbox'] for z in common_props]);lo=region[:,0,:].min(axis=0)-20.;hi=region[:,1,:].max(axis=0)+20.
    survey=dict(completed=True,old_common_properties=common_props,old_common_Boolean_repeated=0,old_axis14_repeated=0,actual_properties={},module_frame={k:f[k] for k in ['placement','delta','radial','normal','opening','guide_frame']},body13_names=sorted(body13),later_electrical6_names=sorted(leads|{lid}|screws),selected_root_region=[lo,hi])
    for part in [m.ROOT_PART,name+'_wire_guide_base',lid,name+'_lid_nut_1',name+'_lid_nut_2',name+'_lid_screw_1',name+'_lid_screw_2']+sorted(leads):
        sh=lookup[part]['shape'];faces=[]
        for i,face in enumerate(sh.Faces()):
            if face.geomType() not in ['PLANE','CYLINDER']:continue
            bb=np.array(n.c.m.pack._bbox(face))
            if part==m.ROOT_PART and (np.any(bb[1]<lo) or np.any(bb[0]>hi)):continue
            ad=BRepAdaptor_Surface(face.wrapped);z=dict(index=i,type=face.geomType(),bbox=bb)
            if z['type']=='PLANE':s=ad.Plane();z.update(point=s.Location().Coord(),normal=s.Axis().Direction().Coord())
            else:s=ad.Cylinder();z.update(point=s.Location().Coord(),axis=s.Axis().Direction().Coord(),radius_mm=s.Radius())
            faces.append(z)
        survey['actual_properties'][part]=dict(volume_mm3=sh.Volume(),center_mm=sh.Center().toTuple(),bbox=n.c.m.pack._bbox(sh),faces=faces)
    n.a.save(HERE/'reviews/survey.json',survey)
    for file in [Path(__file__),HERE/'reviews/survey.json']:source[file.relative_to(ROOT).as_posix()]=n.m.drive.sha(file)
    raw=dict(completed=False,source_sha256=source,native={},paths=[],original16packets=packets,explicit_replacement={m.OLD:m.NAME},mapped_special_packet=prior['mapped_special_packet'],initial_world718_names=sorted(world),final886_names=sorted(final),restored_before=prior['restored_modules'],new_restored_modules=[],planned_front_modules=prior['planned_front_modules'],module_contracts=[{**f,'body13_names':sorted(body13),'later_electrical6_names':sorted(leads|{lid}|screws),'body19_bench_completed':False,'bench_inherited':True}],inherited_prefix=inherited,body13_bench_completed=False,inherited_sample_scope=69,inherited_new_native_groups=0,old_native_repeated=0,new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,preserved_upper_M3_whole_cut=prior['preserved_upper_M3_whole_cut'],reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        scope='Same718 plus special48 and rear8 preserved. Exact69 old independent bench prefix inherited,new0. Different nutbench11/12 tested afresh. Body13 enters world at original datum before three original lead references,lid and its twoM2screws. Plug retained. Then original rack2nuts/M3two/base_nuts. All original24 required. Finite event-enriched paths. Benchtransport/manualholding/registration/mating/harnessflex/captive/tools/threads/torque/force/fullfactory/exterior unverified.')
    raw['module_contracts'][0]['completed']=False
    def save():n.a.save(out,raw)
    def path(stage,label,names,present,vectors,axis):
        names=sorted(names);assert not set(names)&present;steps,events=m.enrich_vectors(vectors,names,sorted(present),boxes);rec=dict(id=name+'__'+label,module=name,stage=stage,moving_names=names,present_names=sorted(present),absent_names=sorted(final-present),approach_axis=axis,requested_vectors=vectors,projected_obstacle_events=events,planned_sample_count=len(steps),samples=[],completed=False);raw['paths'].append(rec);save();fixed=[lookup[k] for k in sorted(present)]
        for i,s in enumerate(steps):
            M=m.translate(s['translation_mm']);key=rec['id']+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);q=n.c.material(moving,fixed);raw['native'][key]=q;rec['samples'].append(dict(id=key,**s,transform=M))
            if i==0:
                mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);rec['initial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1]);assert rec['initial_projection_separation_mm']>0.
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:raw['first_blocked_sample']=key;raw['first_blocked_path']=rec['id'];return False
        rec['completed']=True;present.update(names);save();return True
    def whole(stage,label,names,present,axis,offset=None,tail=None):
        offset=np.zeros(3) if offset is None else offset;start=m.outside_start([lookup[k] for k in names],[lookup[k] for k in present],axis,offset);vectors=[x*axis+offset for x in list(np.arange(start,19.,-20.))+[15.,10.,5.,0.]]+(tail or []);return path(stage,label,names,present,vectors,axis)
    blocked=False
    for nut in sorted(nuts):
        side=(-1. if nut.endswith('_1') else 1.)*frame[:3,1];offset=-4.*opening
        if not whole('carrier_bench','open_body_nut_'+nut,[nut],carrier,side,offset,[-2.*opening,np.zeros(3)]):blocked=True;break
    if not blocked:
        assert carrier==body13;raw['body13_bench_completed']=True
        if not whole('world','body13_before_leads_lid',body13,world,radial):blocked=True
    if not blocked and not whole('world','lay_three_original_leads_in_open_groove',leads,world,opening):blocked=True
    if not blocked and not whole('world','original_lid_after_leads',[lid],world,opening):blocked=True
    if not blocked:
        for screw in sorted(screws):
            if not whole('world','original_lid_M2_'+screw,[screw],world,opening):blocked=True;break
    if not blocked:
        for nut in [k for k in p['mounting_names'] if '_nut_' in k]:
            if not whole('world','rack_nut_'+nut,[nut],world,radial,-20.*normal,[-10.*normal,np.zeros(3)]):blocked=True;break
    if not blocked:
        for screw in [k for k in p['mounting_names'] if '_screw_' in k]:
            if not whole('world','rack_M3_'+screw,[screw],world,radial,40.*normal,[20.*normal,10.*normal,np.zeros(3)]):blocked=True;break
    if not blocked and not whole('world','four_original_M2_base_nuts_last',[base],world,radial):blocked=True
    if not blocked:
        assert set(p['names'])<=world;raw['new_restored_modules'].append(name);raw['module_contracts'][0]['completed']=True
    total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],restored_modules=prior['restored_modules']+raw['new_restored_modules'],packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),all16_packet_restoration_completed=world==final,input_hash_guard=all(n.m.drive.sha(ROOT/file)==h for file,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['new_native_groups'],raw['overlap_records'],'world',len(world),'restored',len(raw['restored_modules']),flush=True)


if __name__=='__main__':main()

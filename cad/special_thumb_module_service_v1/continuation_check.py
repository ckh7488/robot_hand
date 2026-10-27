"""Keep the successful78 prefix; put both original nuts beside the bore first."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm072_continuation',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/continuation_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source=m.context();lookup={z['name']:z for z in rows};final=set(lookup);first=n.p.read(HERE/'reviews/check.json')
    selected=n.p.read(HERE/'reviews/selected_check.json');prefix=first['paths'][:10]+selected['paths'][:3]
    assert all(p['completed'] for p in prefix) and sum(len(p['samples']) for p in prefix)==99
    assert selected['first_blocked_sample']=='jaw_arm059_straight_jaw_0_3'
    bench=set(first['final_motor_bench_present_names']);carrier=set(selected['paths'][3]['present_names']);world=set(first['initial_world_present_names']);assert len(bench)==14 and len(carrier)==28 and len(world)==670
    for p in prefix:
        owner=first if p['id'] in {z['id'] for z in first['paths'][:10]} else selected
        assert all(not owner['native'][s['id']]['overlaps'] and not owner['native'][s['id']]['operation_failures'] for s in p['samples'])
    p=first['original_special_packet'];name=p['module'];base=name+'_base_nuts';body=set(first['body43_names']);radial=np.array(first['radial']);normal=np.array(first['normal']);opening=np.array(first['guide_opening_axis'])
    for f in [Path(__file__),HERE/'reviews/check.json',HERE/'reviews/selected_check.json',HERE/'reviews/selected_evidence.json',HERE/'reviews/selected_common_prepared.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw={k:first[k] for k in ['original16packets','original_special_packet','original_final886_names','initial_world_present_names','restored_before','unit_reference12_names','unit14_names','body43_names','mechanical18_names','radial','normal','guide_opening_axis','guide_actual_frame','conditional_scope','inherited_ARM065_corefalse_ARM059_M3plus13_and_all_rawfalse_holds_unresolved']}
    raw.update(completed=False,source_sha256=source,native={},paths=[],new_restored_modules=[],inherited_prefix_paths=prefix,inherited_native_sample_scope=99,inherited_native_groups_repeated=0,initial_carrier_bench_present_names=sorted(carrier),new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,old_native_groups_repeated=0,internal_reference_unit_retested=False,
        selected_order='Keep exact completed78 initial+21 nuts/adjuster samples,99 inherited not new. Both original jaws enter from the outer radial side,inner jaw first while outer jaw explicitly absent;the blocked inner-side path crossed adapter and3leads. Full outside starts and all original parts retained. Later opposite jaw/nuts/bolts and port1/world only if reached. Initial89 and selected25 rawfalse failures retained.')
    def save():n.a.save(out,raw)
    def path(stage,label,names,present,vectors,axis):
        names=sorted(names);assert not set(names)&present
        rec=dict(id=label,stage=stage,moving_names=names,present_names=sorted(present),absent_names=sorted(final-present),approach_axis=axis,samples=[],completed=False);raw['paths'].append(rec);save();fixed=[lookup[k] for k in present]
        for i,v in enumerate(vectors):
            M=m.translate(v);key=label+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);q=n.c.material(moving,fixed);raw['native'][key]=q;rec['samples'].append(dict(id=key,translation_mm=v,transform=M))
            if i==0:
                mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);rec['initial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1])
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:raw['first_blocked_sample']=key;raw['first_blocked_path']=label;save();return False
        rec['completed']=True;present.update(names);save();return True
    def benchpath(label,names,axis):return path('carrier_bench',label,names,carrier,m.bench_vectors([lookup[k] for k in names],[lookup[k] for k in carrier],axis),axis)
    def run():
        for i in [0,1]:
            if i==1:
                for label in ['top','bottom']:
                    nut='thumb_ip_barrel_'+label+'_nut_'+str(i)
                    if not benchpath('barrel_'+str(i)+'_'+label+'_nut_lateral_before_adjuster',[nut],radial):return False
                adjuster='thumb_ip_barrel_adjuster_'+str(i)
                if not benchpath('barrel_'+str(i)+'_through_two_preplaced_nuts',[adjuster],opening):return False
            for jaw,axis in [('arm059_straight_jaw_'+str(i),radial),('motor_housing_clamp_'+str(i)+'_jaw_1',radial)]:
                if not benchpath('jaw_outer_side_first_'+jaw,[jaw],axis):return False
            for k in [0,1]:
                nut='motor_housing_clamp_'+str(i)+'_nut_'+str(k)
                if not benchpath('clamp_nut_'+nut,[nut],-radial):return False
            for k in [0,1]:
                screw='motor_housing_clamp_'+str(i)+'_bolt_'+str(k)
                if not benchpath('clamp_M2_'+screw,[screw],radial):return False
        assert carrier==body
        start=m.outside_start([lookup[k] for k in body],[lookup[k] for k in world],radial,np.zeros(3))
        if not path('world','special_body43_final_datum_radial',body,world,[x*radial for x in m.a.radial_entry_steps(start)],radial):return False
        for nut in [k for k in p['mounting_names'] if '_nut_' in k]:
            offset=-20.*normal;start=m.outside_start([lookup[nut]],[lookup[k] for k in world],radial,offset)
            if not path('world','rack_nut_'+nut,[nut],world,[x*radial+offset for x in m.inward_steps(start)]+[-10.*normal,np.zeros(3)],radial):return False
        for screw in [k for k in p['mounting_names'] if '_screw_' in k]:
            offset=40.*normal;start=m.outside_start([lookup[screw]],[lookup[k] for k in world],radial,offset)
            if not path('world','rack_M3_'+screw,[screw],world,[x*radial+offset for x in m.inward_steps(start)]+[20.*normal,10.*normal,np.zeros(3)],radial):return False
        start=m.outside_start([lookup[base]],[lookup[k] for k in world],radial,np.zeros(3))
        if not path('world','four_M2_base_nuts_last',[base],world,[x*radial for x in list(np.arange(start,0.,-20.))]+[10.*radial,5.*radial,np.zeros(3)],radial):return False
        assert set(p['names'])<=world;raw['new_restored_modules']=[name];return True
    passed=run();total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],special48_restoration_completed=passed,
        final_motor_bench_present_names=sorted(bench),final_carrier_bench_present_names=sorted(carrier),restored_modules=[p['module'] for p in raw['original16packets'] if set(p['names'])<=world],packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),explicitly_later_front7_modules=[p['module'] for p in raw['original16packets'] if p['row']==1 and p['module']!=name],input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete selected',raw['new_native_groups'],raw['overlap_records'],'special48',passed,flush=True)


if __name__=='__main__':main()

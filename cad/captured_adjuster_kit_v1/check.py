"""Whole captured port0 kit bench and carrier entry; preserve known port1 blocks."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm073_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source=m.context();lookup={z['name']:z for z in rows};final=set(lookup)
    old=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/check.json');sel=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/selected_check.json');survey=n.p.read(HERE/'reviews/survey.json')
    prefix=old['paths'][:10]+sel['paths'][:2];assert all(p['completed'] for p in prefix) and sum(len(p['samples']) for p in prefix)==92
    for p in prefix:
        owner=old if p['id'] in {z['id'] for z in old['paths'][:10]} else sel
        assert all(not owner['native'][s['id']]['overlaps'] and not owner['native'][s['id']]['operation_failures'] for s in p['samples'])
    carrier=set(sel['paths'][2]['present_names']);world=set(old['initial_world_present_names']);assert len(carrier)==27 and len(world)==670
    adjuster='thumb_ip_barrel_adjuster_0';inner='arm059_straight_jaw_0';outer='motor_housing_clamp_0_jaw_1';nuts=['motor_housing_clamp_0_nut_'+str(i) for i in [0,1]];screws=['motor_housing_clamp_0_bolt_'+str(i) for i in [0,1]]
    kit={adjuster,inner,outer,*nuts,*screws};assert len(kit)==7 and not kit&carrier;bench={adjuster};radial=np.array(old['radial']);opening=np.array(old['guide_opening_axis'])
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    raw=dict(completed=False,source_sha256=source,native={},paths=[],original16packets=m.packets(rows),original_special_packet=old['original_special_packet'],original_final886_names=sorted(final),
        initial_world_present_names=sorted(world),initial_carrier_bench_present_names=sorted(carrier),initial_kit_bench_present_names=sorted(bench),kit7_names=sorted(kit),inherited_prefix_paths=prefix,inherited_native_sample_scope=92,inherited_native_groups_repeated=0,
        existing_ARM059_port1_final_conflicts=survey['existing_ARM059_port1_final_conflicts'],port1_existing_conflicts_retested=0,radial=radial,guide_opening_axis=opening,body43_names=old['body43_names'],new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,
        old_native_groups_repeated=0,old_common_Boolean_repeated=0,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        conditional_scope='Original reference12/motor bench and carrier/electrical78,plus exactly same port0 barrel nut lateral14 are inherited92,not new92. A manually held original adjuster starts an independent kit bench in the module reference frame;inner jaw from its open inner radial side,opposite jaw,originalM2nuts/bolts,then wholekit7 enters carrier from actual open axial side. Bench-to-entry transport/registration and temporaryholding are separate assumptions,not a new wholefactory proof. All original7 restored at finalidentity;both jamnuts2 already present. Source ARM059 three finalport1/base overlaps are read only,not rerun or cleared by assembly order. Original rear8/670 retained;no worldspecial48/front7 test or claim.',
        inherited_rawfalse_ARM065_corefalse_M3plus13_strictfalse_holds_unresolved=True)
    def save():n.a.save(out,raw)
    def path(stage,label,names,present,axis):
        names=sorted(names);assert not set(names)&present
        vectors=m.bench_vectors([lookup[k] for k in names],[lookup[k] for k in present],axis);rec=dict(id=label,stage=stage,moving_names=names,present_names=sorted(present),absent_names=sorted(final-present),approach_axis=axis,samples=[],completed=False);raw['paths'].append(rec);save();fixed=[lookup[k] for k in present]
        for i,v in enumerate(vectors):
            M=m.translate(v);key=label+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);q=n.c.material(moving,fixed);raw['native'][key]=q;rec['samples'].append(dict(id=key,translation_mm=v,transform=M))
            if i==0:
                mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);rec['initial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1])
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:raw['first_blocked_sample']=key;raw['first_blocked_path']=label;save();return False
        rec['completed']=True;present.update(names);save();return True
    def run():
        if not path('kit_bench','inner_jaw_from_open_half_before_carrier',[inner],bench,-radial):return False
        if not path('kit_bench','opposite_jaw_capture_adjuster',[outer],bench,radial):return False
        for nut in nuts:
            if not path('kit_bench','kit_M2_'+nut,[nut],bench,-radial):return False
        for screw in screws:
            if not path('kit_bench','kit_M2_'+screw,[screw],bench,radial):return False
        assert bench==kit
        if not path('carrier_bench','whole_original_kit7_into_R5_with_two_nuts',kit,carrier,opening):return False
        assert len(carrier)==34 and kit<=carrier;return True
    passed=run();total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],port0_kit_external_assembly_completed=passed,
        final_kit_bench_present_names=sorted(bench),final_carrier_bench_present_names=sorted(carrier),packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),restored_modules=old['restored_before'],new_restored_modules=[],special48_restoration_completed=False,
        port1_attempt_deferred_due_to_existing_final_geometry_conflicts=True,port1_other_mechanical9_names=sorted(set(old['body43_names'])-set(sel['paths'][2]['present_names'])-kit) if passed else [],
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['new_native_groups'],raw['overlap_records'],'port0kit',passed,'world670unchanged',flush=True)


if __name__=='__main__':main()

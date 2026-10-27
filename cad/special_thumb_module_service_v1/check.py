"""Check original special thumbIP service order; stop at the first new block."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm072_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source=m.context();lookup={z['name']:z for z in rows};final=set(lookup)
    prior=n.p.read(ROOT/'cad/datum_plane_packet_entry_v1/reviews/check.json');survey=n.p.read(HERE/'reviews/survey.json');p=survey['original_packet'];name=p['module']
    world=set(prior['packet_final_present_names']);assert len(world)==670 and len(prior['restored_modules'])==8
    radial=np.array(survey['radial']);normal=np.array(survey['normal']);opening=np.array(survey['guide_opening_axis']);frame=np.array(survey['guide_actual_frame'])
    assert np.max(abs(opening+normal))<1e-10
    core={name+'_motor',name+'_body_screws','thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer','thumb_ip_stock_rear_idler','thumb_ip_two_tail_spool',*[f'thumb_ip_spool_tap6_{i}' for i in range(1,5)],'thumb_ip_tail_clamp_1','thumb_ip_tail_clamp_2'}
    unit=core|{name+'_cradle',name+'_base_screws'};base=name+'_base_nuts';body=set(p['body_names'])-{base};guide='arm059_integrated_electrical_R5_base'
    electrical={name+s for s in ['_lead1','_lead2','_lead3','_plug_outer']};nuts={name+'_lid_nut_'+str(i) for i in [1,2]};lid=name+'_wire_guide_lid';screws={name+'_lid_screw_'+str(i) for i in [1,2]}
    mechanical=set(p['body_names'])-(unit|electrical|nuts|screws|{guide,lid,name+'_adapter',base})
    assert len(core)==12 and len(unit)==14 and len(body)==43 and len(mechanical)==18
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    raw=dict(completed=False,source_sha256=source,native={},paths=[],original16packets=m.packets(rows),original_special_packet=p,original_final886_names=sorted(final),
        initial_world_present_names=sorted(world),restored_before=prior['restored_modules'],new_restored_modules=[],unit_reference12_names=sorted(core),unit14_names=sorted(unit),body43_names=sorted(body),mechanical18_names=sorted(mechanical),
        radial=radial,normal=normal,guide_opening_axis=opening,guide_actual_frame=frame,new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,old_native_groups_repeated=0,internal_reference_unit_retested=False,
        conditional_scope='Original rear8/670 actual obstacles retained. Special thumbIP48=body44+rack4. Original motor/body screws/stockhorn/retainer/rearidler and custom7 output are a preassembled rigid reference12,with original ARM059 clock and vendor/TAP/mating assumptions retained;their internal assembly is not newly tested or approved. New M2/cradle approach and complete external carrier/electrical/R5/adjuster/jaw service stages only as reached. No generic rotor/spool/spacer substitutions. No long cord/housing present. Manual holding,rigid nominal leads/plug,tools/thread/torque/captive/harnessflex/wholefactory unverified.',
        inherited_ARM065_corefalse_ARM059_M3plus13_and_all_rawfalse_holds_unresolved=True)
    def save():n.a.save(out,raw)
    def path(stage,label,names,present,vectors,axis):
        names=sorted(names);assert not set(names)&present
        rec=dict(id=label,stage=stage,moving_names=names,present_names=sorted(present),absent_names=sorted(final-present),approach_axis=axis,samples=[],completed=False);raw['paths'].append(rec);save()
        fixed=[lookup[k] for k in present]
        for i,v in enumerate(vectors):
            M=m.translate(v);key=label+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);q=n.c.material(moving,fixed)
            raw['native'][key]=q;rec['samples'].append(dict(id=key,translation_mm=v,transform=M))
            if i==0:
                mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);rec['initial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1])
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:raw['first_blocked_sample']=key;raw['first_blocked_path']=label;save();return False
        rec['completed']=True;present.update(names);save();return True
    def benchpath(label,names,present,axis):return path('carrier_bench',label,names,present,m.bench_vectors([lookup[k] for k in names],[lookup[k] for k in present],axis),axis)
    bench={name+'_cradle'};carrier={name+'_adapter'};near=[80.,60.,40.,20.,10.,5.,0.]
    def run():
        if not path('motor_bench','M2_screws_before_reference_motor',[name+'_base_screws'],bench,[-x*radial for x in near],-radial):return False
        if not path('motor_bench','reference_motor_output12_into_cradle',core,bench,[-x*radial for x in near],-radial):return False
        if not benchpath('integrated_electrical_R5_base_side_before_motor',[guide],carrier,radial):return False
        if not benchpath('reference_unit14_with_own_plug_into_open_base',unit|{name+'_plug_outer'},carrier,-radial):return False
        if not benchpath('lay_three_leads_in_open_integrated_groove',electrical-{name+'_plug_outer'},carrier,opening):return False
        for nut in sorted(nuts):
            side=(-1. if nut.endswith('_1') else 1.)*frame[:3,1];offset=-4.*opening;start=m.outside_start([lookup[nut]],[lookup[k] for k in carrier],side,offset)
            if not path('carrier_bench','lid_nut_side_then_pocket_'+nut,[nut],carrier,[x*side+offset for x in m.inward_steps(start)]+[-2.*opening,np.zeros(3)],side):return False
        if not benchpath('lid_after_three_leads',[lid],carrier,opening):return False
        for screw in sorted(screws):
            if not benchpath('lid_M2_'+screw,[screw],carrier,opening):return False
        for i in [0,1]:
            adjuster='thumb_ip_barrel_adjuster_'+str(i)
            if not benchpath('barrel_'+str(i)+'_through_original_bore',[adjuster],carrier,opening):return False
            for side,label in [(normal,'top'),(opening,'bottom')]:
                nut='thumb_ip_barrel_'+label+'_nut_'+str(i)
                if not benchpath('barrel_'+str(i)+'_'+label+'_nut',[nut],carrier,side):return False
            for jaw,side in [('arm059_straight_jaw_'+str(i),-radial),('motor_housing_clamp_'+str(i)+'_jaw_1',radial)]:
                if not benchpath('jaw_'+jaw,[jaw],carrier,side):return False
            for k in [0,1]:
                nut='motor_housing_clamp_'+str(i)+'_nut_'+str(k)
                if not benchpath('clamp_nut_'+nut,[nut],carrier,-radial):return False
            for k in [0,1]:
                screw='motor_housing_clamp_'+str(i)+'_bolt_'+str(k)
                if not benchpath('clamp_M2_'+screw,[screw],carrier,radial):return False
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
        if not path('world','four_M2_base_nuts_last',[base],world,[x*radial for x in m.inward_steps(start)]+[np.zeros(3)],radial):return False
        assert set(p['names'])<=world;raw['new_restored_modules']=[name];return True
    passed=run();total=d.k.flat(list(raw['native'].values()));restored=[p['module'] for p in raw['original16packets'] if set(p['names'])<=world]
    raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],special48_restoration_completed=passed,
        final_motor_bench_present_names=sorted(bench),final_carrier_bench_present_names=sorted(carrier),restored_modules=restored,packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),explicitly_later_front7_modules=[p['module'] for p in raw['original16packets'] if p['row']==1 and p['module']!=name],
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['new_native_groups'],raw['overlap_records'],'special48',passed,flush=True)


if __name__=='__main__':main()

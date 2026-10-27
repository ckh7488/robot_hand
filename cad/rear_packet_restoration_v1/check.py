"""Restore the remaining seven rear modules,stopping at the first new block."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm069_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source=m.context();lookup={z['name']:z for z in rows};final=set(lookup)
    prior=n.p.read(ROOT/'cad/captive_module_assembly_v1/reviews/selected_check.json');ext=n.p.read(ROOT/'cad/captive_module_assembly_v1/reviews/entry_extension.json')
    assert prior['restored_modules']==['index_mcp_actuator'] and ext['extended_access_clear']
    packet=m.packets(rows);plans=[p for p in packet if p['row']==0 and p['module'] not in prior['restored_modules']];assert len(plans)==7
    placements={p['module']:p for p in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements']}
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    world=set(prior['packet_final_present_names']);assert len(world)==502
    raw=dict(completed=False,source_sha256=source,native={},paths=[],module_contracts=[],original16packets=packet,
        initial_world_present_names=sorted(world),original_final886_names=sorted(final),planned_rear_modules=[p['module'] for p in plans],
        explicitly_later_front8_modules=[p['module'] for p in packet if p['row']==1],restored_before=prior['restored_modules'],new_restored_modules=[],
        new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        first_packet_and_support_paths_repeated=0,all_old_native_replayed=0,
        conditional_scope='Inherit original conditional502 from ARM068,including original first24 and assumednonpacket478. Rear station before front station. Each remaining rear module has actual original M2/M3 basis and all original parts. Separate cradle/M2/motor+spool reference bench stages,then same motor plus own3leads/plug into adapter/guide carrier. Electrical/vendor/output internal groups remain references,not physical mating/torque proof. Body omits only own base_nuts temporarily;restore original racknuts/M3bolts then base_nuts. Whole-AABB outside starts for world entries. Manualholding/captive retention/tools/threadtorque/continuous paths and registration/other factory assembly unverified.',
        inherited_ARM065_corefalse_and_all_previous_holds_unresolved=True)
    def save():n.a.save(out,raw)
    def path(module,stage,label,names,present,vectors,radial):
        names=sorted(names);assert not set(names)&present
        rec=dict(id=module+'__'+label,module=module,stage=stage,moving_names=names,present_names=sorted(present),absent_names=sorted(final-present),samples=[],completed=False)
        raw['paths'].append(rec);save()
        for i,v in enumerate(vectors):
            v=np.array(v);M=m.translate(v);key=rec['id']+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);fixed=[lookup[k] for k in present]
            q=n.c.material(moving,fixed);raw['native'][key]=q;rec['samples'].append(dict(id=key,translation_mm=v,transform=M))
            if i==0:
                mb=m.project_bounds(moving,radial);fb=m.project_bounds(fixed,radial)
                rec['initial_radial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1])
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:
                raw['first_blocked_sample']=key;raw['first_blocked_path']=rec['id'];return False
        rec['completed']=True;present.update(names);save();return True
    blocked=False
    near=[80.,60.,40.,20.,10.,5.,0.]
    for p in plans:
        name=p['module'];P=np.array(placements[name]['proposed_installation_transform']);radial=-P[:3,2];normal=P[:3,0]
        assert np.max(abs(normal-np.array([0.,1.,0.])))<1e-12 and len(p['names'])==24
        base=name+'_base_nuts';body=set(p['body_names'])-{base}
        core={name+s for s in ['_motor','_rotor','_body_screws','_spool']};unit=core|{name+'_cradle',name+'_base_screws'}
        electrical={name+s for s in ['_lead1','_lead2','_lead3','_plug_outer']};moving_unit=unit|electrical;carrier=body-moving_unit
        assert len(body)==19 and len(unit)==6 and len(moving_unit)==10 and len(carrier)==9
        contract=dict(module=name,azimuth_deg=p['azimuth_deg'],placement=P,radial=radial,normal=normal,body19_names=sorted(body),unit6_names=sorted(unit),
            motor_and_own_electrical10_names=sorted(moving_unit),carrier9_names=sorted(carrier),temporarily_later_base_nuts=base,original24_names=p['names'],world_before_names=sorted(world))
        raw['module_contracts'].append(contract);save()
        bench={name+'_cradle'}
        if not path(name,'bench','M2_screws_before_motor',[name+'_base_screws'],bench,[-x*radial for x in near],radial):blocked=True;break
        if not path(name,'bench','motor_reference_into_cradle',core,bench,[-x*radial for x in near],radial):blocked=True;break
        bench_carrier=set(carrier)
        if not path(name,'carrier_bench','motor_and_own_leads_into_carrier',moving_unit,bench_carrier,[-x*radial for x in near],radial):blocked=True;break
        assert bench_carrier==body
        fixed=[lookup[k] for k in world];moving=[lookup[k] for k in body];offset=8.*normal
        start=m.outside_start(moving,fixed,radial,offset);contract['body_radial_start_mm']=start
        vectors=[x*radial+offset for x in m.inward_steps(start)]+[4.*normal,np.zeros(3)]
        if not path(name,'world','body19_side_then_normal',body,world,vectors,radial):blocked=True;break
        for nut in [k for k in p['mounting_names'] if '_nut_' in k]:
            offset=-20.*normal;start=m.outside_start([lookup[nut]],[lookup[k] for k in world],radial,offset)
            vectors=[x*radial+offset for x in m.inward_steps(start)]+[-10.*normal,np.zeros(3)]
            if not path(name,'world','rack_nut_'+nut,[nut],world,vectors,radial):blocked=True;break
        if blocked:break
        for screw in [k for k in p['mounting_names'] if '_screw_' in k]:
            offset=40.*normal;start=m.outside_start([lookup[screw]],[lookup[k] for k in world],radial,offset)
            vectors=[x*radial+offset for x in m.inward_steps(start)]+[20.*normal,10.*normal,np.zeros(3)]
            if not path(name,'world','rack_M3_'+screw,[screw],world,vectors,radial):blocked=True;break
        if blocked:break
        start=m.outside_start([lookup[base]],[lookup[k] for k in world],radial,np.zeros(3))
        vectors=[x*radial for x in m.inward_steps(start)]+[10.*radial,5.*radial,np.zeros(3)]
        # Keep the final0 only once when extending the inward sequence.
        vectors=[x*radial for x in list(np.arange(start,0.,-20.))]+[10.*radial,5.*radial,np.zeros(3)]
        if not path(name,'world','four_M2_base_nuts_last',[base],world,vectors,radial):blocked=True;break
        assert set(p['names'])<=world
        raw['new_restored_modules'].append(name);contract['completed']=True;contract['world_after_names']=sorted(world);save()
    total=d.k.flat(list(raw['native'].values()));restored=[p['module'] for p in packet if set(p['names'])<=world]
    raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),
        operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],
        rear_station_restoration_completed=not blocked and len(raw['new_restored_modules'])==7,restored_modules=restored,
        packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),
        all16_packet_restoration_completed=world==final,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['new_native_groups'],raw['overlap_records'],'restored',len(restored),'rear',raw['rear_station_restoration_completed'],flush=True)


if __name__=='__main__':main()

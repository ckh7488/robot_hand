"""Open guide before laying leads;restore all original parts and world poses."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm070_final',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/final_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source=m.context();lookup={z['name']:z for z in rows};final=set(lookup)
    prior=n.p.read(ROOT/'cad/rear_packet_restoration_v1/reviews/check.json');assert len(prior['packet_final_present_names'])==598 and len(prior['restored_modules'])==5
    world=set(prior['packet_final_present_names']);packet=m.packets(rows);plans=[p for p in packet if p['row']==0 and p['module'] not in prior['restored_modules']]
    assert [p['module'] for p in plans]==['ring_pip_actuator','ring_dip_actuator','middle_pip_actuator']
    placements={p['module']:p for p in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements']}
    guides={p['module']:p for p in n.p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')['groups']}
    wires={z['name']:z for z in n.p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts']}
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    for f in [HERE/'check.py',HERE/'reviews/check.json',HERE/'selected_check.py',HERE/'reviews/selected_check.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    selected=n.p.read(HERE/'reviews/selected_check.json');assert selected['completed'] and selected['new_native_groups']==33
    raw=dict(completed=False,source_sha256=source,native={},paths=[],module_contracts=[],inherited_bench_paths=[],inherited_open_paths=[],original16packets=packet,
        initial_world_present_names=sorted(world),original_final886_names=sorted(final),planned_rear_modules=[p['module'] for p in plans],restored_before=prior['restored_modules'],new_restored_modules=[],
        explicitly_later_front8_modules=[p['module'] for p in packet if p['row']==1],new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,all_old_native_replayed=0,
        conditional_scope='Original conditional598 retains completed5packets and assumed nonpacket478. Separate registered/manual-held bench. Original guidebase enters laterally onto adapter,spacers follow;motorunit6+ownplug1 seats,then lay original3leads through actual open groove,then original nut2/lid/screw2. Motor/plug attachment and each lead shape are nominal references,not supplier mating or flexible harness assembly. Same ringPIP guide/spacer21 and unit6 path7 inherited;new plug-vs-carrier only on same7. First5 lead3-vs-oldcarrier10 are exact moving subsets of selected electrical4 query,including blocked sample whose only overlap was movingplug;new lead3-vs-fixedplug5 tested. Last2 lead3-vs-fullcarrier11 new. Other modules tested afresh. Original base_nuts temporarily absent until rack M3;restore original24 at identity. Manualholding/captive/tools/torque/registration/otherfactory unverified.',
        inherited_ARM065_corefalse_and_all_previous_holds_unresolved=True)
    def save():n.a.save(out,raw)
    def path(module,stage,label,names,present,vectors,axis):
        names=sorted(names);assert not set(names)&present
        rec=dict(id=module+'__'+label,module=module,stage=stage,moving_names=names,present_names=sorted(present),absent_names=sorted(final-present),approach_axis=axis,samples=[],completed=False)
        raw['paths'].append(rec);save()
        for i,v in enumerate(vectors):
            M=m.translate(v);key=rec['id']+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);fixed=[lookup[k] for k in present]
            new_names=names;new_present=set(present);inherited=None
            if module=='ring_pip_actuator' and label=='unit7_with_own_plug_into_open_guide':
                old=next(z for z in selected['paths'] if z['id'].endswith('__unit6_into_open_guide'));old_s=old['samples'][i]
                assert np.max(abs(np.array(old_s['transform'])-M))<1e-10 and set(old['present_names'])==present and set(old['moving_names'])|{module+'_plug_outer'}==set(names)
                inherited=dict(report='cad/open_guide_module_assembly_v1/reviews/selected_check.json',id=old_s['id'],moving_names=old['moving_names'],present_names=old['present_names'])
                new_names=[module+'_plug_outer']
            if module=='ring_pip_actuator' and label=='lay_three_leads_in_open_groove' and i<5:
                old=next(z for z in selected['paths'] if z['id'].endswith('__lay_electrical4_in_open_groove'));old_s=old['samples'][i]
                assert np.max(abs(np.array(old_s['transform'])-M))<1e-10 and set(old['present_names'])|{module+'_plug_outer'}==present and set(names)<set(old['moving_names'])
                inherited=dict(report='cad/open_guide_module_assembly_v1/reviews/selected_check.json',id=old_s['id'],moving_names=names,present_names=old['present_names'])
                new_present={module+'_plug_outer'}
            if inherited:
                old_q=selected['native'][inherited['id']]
                assert not old_q['operation_failures'] and not [z for z in old_q['overlaps'] if z['lead'] in inherited['moving_names'] and z['part'] in inherited['present_names']]
            q=n.c.material(m.moved([lookup[k] for k in new_names],M),[lookup[k] for k in new_present]);raw['native'][key]=q
            rec['samples'].append(dict(id=key,translation_mm=v,transform=M,native_moving_names=new_names,native_present_names=sorted(new_present),inherited_query=inherited))
            if i==0:
                mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);rec['initial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1])
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:
                raw['first_blocked_sample']=key;raw['first_blocked_path']=rec['id'];return False
        rec['completed']=True;present.update(names);save();return True
    def benchpath(name,label,names,present,axis):
        return path(name,'carrier_bench',label,names,present,m.bench_vectors([lookup[k] for k in names],[lookup[k] for k in present],axis),axis)
    blocked=False
    for p in plans:
        name=p['module'];pl=placements[name];P=np.array(pl['proposed_installation_transform']);delta=np.array(pl['transform_from_current'])
        radial=-P[:3,2];normal=P[:3,0];opening=delta[:3,:3]@np.array([0.,0.,1.]);assert np.max(abs(opening+normal))<1e-10
        base=name+'_base_nuts';body=set(p['body_names'])-{base};core={name+s for s in ['_motor','_rotor','_body_screws','_spool']};unit=core|{name+'_cradle',name+'_base_screws'}
        electrical={name+s for s in ['_lead1','_lead2','_lead3','_plug_outer']};adapter={name+'_adapter'};spacers={name+'_guide_spacer_'+str(i) for i in [1,2]}
        guide=name+'_wire_guide_base';lid=name+'_wire_guide_lid';nuts={name+'_lid_nut_'+str(i) for i in [1,2]};screws={name+'_lid_screw_'+str(i) for i in [1,2]}
        assert len(body)==19 and body==unit|electrical|adapter|spacers|{guide,lid}|nuts|screws
        frame=delta@np.array(guides[name]['frame_world_mm']);wire_data=[]
        for suffix in ['_lead1','_lead2','_lead3']:
            z=wires[name+suffix];W=delta@np.array(z['transform_world_mm']);wire_data.append(dict(name=z['name'],actual_transform=W,original_local=z['local'],actual_end=(W@np.r_[z['local']['end_mm'],1.])[:3]))
        contract=dict(module=name,placement=P,delta=delta,radial=radial,normal=normal,guide_opening_axis=opening,guide_actual_frame=frame,
            original_guide=guides[name],original_leads=wire_data,body19_names=sorted(body),unit6_names=sorted(unit),electrical4_names=sorted(electrical),
            adapter_names=sorted(adapter),spacer_names=sorted(spacers),guide_name=guide,lid_name=lid,lid_nut_names=sorted(nuts),lid_screw_names=sorted(screws),original24_names=p['names'],temporarily_later_base_nuts=base,world_before_names=sorted(world))
        raw['module_contracts'].append(contract);save()
        bench={name+'_cradle'}
        if name=='ring_pip_actuator':
            inherited=[z for z in prior['paths'] if z['module']==name and z['stage']=='bench'];assert len(inherited)==2
            for old in inherited:
                assert old['completed'] and set(old['present_names'])==bench and len(old['samples'])==7
                assert all(not prior['native'][s['id']]['overlaps'] and not prior['native'][s['id']]['operation_failures'] for s in old['samples'])
                for s,x in zip(old['samples'],[80.,60.,40.,20.,10.,5.,0.]):assert np.max(abs(np.array(s['transform'])-m.translate(-x*radial)))<1e-10
                raw['inherited_bench_paths'].append(dict(source_path=old['id'],source_report='cad/rear_packet_restoration_v1/reviews/check.json',moving_names=old['moving_names'],present_names=old['present_names'],sample_ids=[s['id'] for s in old['samples']],source_shape_hashes={f:h for f,h in source.items() if any(k in f for k in set(old['moving_names'])|bench) and f.endswith('.step')},new_native_groups=0))
                bench.update(old['moving_names'])
            assert bench==unit;save()
        else:
            near=[80.,60.,40.,20.,10.,5.,0.]
            if not path(name,'bench','M2_screws_before_motor',[name+'_base_screws'],bench,[-x*radial for x in near],-radial):blocked=True;break
            if not path(name,'bench','motor_reference_into_cradle',core,bench,[-x*radial for x in near],-radial):blocked=True;break
        carrier=set(adapter)
        if name=='ring_pip_actuator':
            for old in selected['paths'][:3]:
                assert old['completed'] and set(old['present_names'])==carrier and not carrier&set(old['moving_names'])
                assert all(not selected['native'][s['id']]['overlaps'] and not selected['native'][s['id']]['operation_failures'] for s in old['samples'])
                raw['inherited_open_paths'].append(dict(report='cad/open_guide_module_assembly_v1/reviews/selected_check.json',path=old,new_native_groups=0));carrier.update(old['moving_names']);save()
        else:
            if not benchpath(name,'guide_base_side_before_motor',[guide],carrier,radial):blocked=True;break
            for spacer in sorted(spacers):
                if not benchpath(name,'spacer_'+spacer,[spacer],carrier,-opening):blocked=True;break
            if blocked:break
        if not benchpath(name,'unit7_with_own_plug_into_open_guide',unit|{name+'_plug_outer'},carrier,-radial):blocked=True;break
        if not benchpath(name,'lay_three_leads_in_open_groove',electrical-{name+'_plug_outer'},carrier,opening):blocked=True;break
        for nut in sorted(nuts):
            if not benchpath(name,'guide_nut_'+nut,[nut],carrier,-opening):blocked=True;break
        if blocked:break
        if not benchpath(name,'guide_lid_after_leads',[lid],carrier,opening):blocked=True;break
        for screw in sorted(screws):
            if not benchpath(name,'guide_M2_'+screw,[screw],carrier,opening):blocked=True;break
        if blocked:break
        assert carrier==body
        offset=8.*normal;start=m.outside_start([lookup[k] for k in body],[lookup[k] for k in world],radial,offset)
        if not path(name,'world','body19_side_then_normal',body,world,[x*radial+offset for x in m.inward_steps(start)]+[4.*normal,np.zeros(3)],radial):blocked=True;break
        for nut in [k for k in p['mounting_names'] if '_nut_' in k]:
            offset=-20.*normal;start=m.outside_start([lookup[nut]],[lookup[k] for k in world],radial,offset)
            if not path(name,'world','rack_nut_'+nut,[nut],world,[x*radial+offset for x in m.inward_steps(start)]+[-10.*normal,np.zeros(3)],radial):blocked=True;break
        if blocked:break
        for screw in [k for k in p['mounting_names'] if '_screw_' in k]:
            offset=40.*normal;start=m.outside_start([lookup[screw]],[lookup[k] for k in world],radial,offset)
            if not path(name,'world','rack_M3_'+screw,[screw],world,[x*radial+offset for x in m.inward_steps(start)]+[20.*normal,10.*normal,np.zeros(3)],radial):blocked=True;break
        if blocked:break
        start=m.outside_start([lookup[base]],[lookup[k] for k in world],radial,np.zeros(3))
        if not path(name,'world','four_M2_base_nuts_last',[base],world,[x*radial for x in list(np.arange(start,0.,-20.))]+[10.*radial,5.*radial,np.zeros(3)],radial):blocked=True;break
        assert set(p['names'])<=world;raw['new_restored_modules'].append(name);contract['completed']=True;contract['world_after_names']=sorted(world);save()
    total=d.k.flat(list(raw['native'].values()));restored=[p['module'] for p in packet if set(p['names'])<=world]
    raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),
        raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],rear_station_restoration_completed=not blocked and len(raw['new_restored_modules'])==3,restored_modules=restored,
        packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),all16_packet_restoration_completed=world==final,
        inherited_bench_groups=sum(len(z['sample_ids']) for z in raw['inherited_bench_paths']),inherited_open_path_groups=sum(len(z['path']['samples']) for z in raw['inherited_open_paths']),
        inherited_subset_sample_groups=sum(bool(s['inherited_query']) for p in raw['paths'] for s in p['samples']),
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['new_native_groups'],raw['overlap_records'],'restored',len(restored),'rear',raw['rear_station_restoration_completed'],flush=True)


if __name__=='__main__':main()

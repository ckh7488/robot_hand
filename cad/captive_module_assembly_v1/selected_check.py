"""Bench-preload M2 screws;mount M3 rack bolts before the four base nuts."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm068_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/selected_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source=m.context();lookup={z['name']:z for z in rows};final=set(lookup)
    old=n.p.read(ROOT/'cad/oriented_fastener_entry_v1/reviews/check.json');survey=n.p.read(HERE/'reviews/survey.json')
    assert old['completed'] and old['bench_completed'] and survey['completed']
    packets=m.packets(rows);p=packets[0];name=p['module'];assert name=='index_mcp_actuator' and len(p['names'])==24
    base=name+'_base_nuts';body=set(p['body_names'])-{base}
    unit={name+s for s in ['_motor','_rotor','_cradle','_body_screws','_base_screws','_spool']};carrier=body-unit
    assert len(body)==19 and len(unit)==6 and len(carrier)==13
    first=n.p.read(HERE/'reviews/check.json');assert first['completed'] and first['new_native_groups']==20
    electrical={name+s for s in ['_lead1','_lead2','_lead3','_plug_outer']};unit|=electrical;carrier-=electrical
    assert len(unit)==10 and len(carrier)==9
    source.update(survey['source_sha256'])
    for f in [Path(__file__),HERE/'reviews/survey.json',HERE/'reviews/check.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw=dict(completed=False,source_sha256=source,paths=[],native={},packets=packets,original_final886_names=sorted(final),
        bench_names=old['bench_names'],new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,
        module=name,body19_names=sorted(body),unit10_names=sorted(unit),carrier9_names=sorted(carrier),temporarily_later_base_nuts=base,
        inherited_ARM067_first_failure_preserved=True,old_failure_common_read_only=survey['saved_failed_common'],old_failure_common_Boolean_repeated=0,
        inherited_clear_paths=[],reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        conditional_scope='Fresh conditional bench,not disassembly of ARM067500. Original cradle is manually held;load four-M2screw compound before motor/rotor/body-screw/spool reference unit. Carry same motor unit with its own three leads/plug (10rows) into unchanged9-row adapter/guide carrier,without base_nuts. Electrical fixed shape moves with its motor;actual harness flexibility/mating not approved. Carrier electrical/lid and supplier/output internals are reference subassemblies,not newly proven internal assembly,electrical mating or torque couplings. Then register bench22 with nonpacket478 assumed assembled and reuse only subset-proven prior body/nut routes. Insert rack M3bolts before original four-M2nut compound. Physical holding/captive retention/jig/driver/threads/torque and continuous paths unverified.',
        support_bench22_paths_repeated=0,old_ARM067_native_paths_repeated=0,other15_packet_restoration_executed=False,
        inherited_ARM065_corefalse_and_previous_holds_unresolved=True)
    def save():n.a.save(out,raw)
    def path(stage,label,names,installed,vectors):
        names=sorted(names);assert not set(names)&installed
        rec=dict(id=label,stage=stage,moving_names=names,present_names=sorted(installed),absent_names=sorted(final-installed),samples=[],completed=False,manual_support=True)
        raw['paths'].append(rec);save()
        for i,v in enumerate(vectors):
            key=label+'_'+str(i);M=m.translate(v);moving=m.moved([lookup[k] for k in names],M)
            q=n.c.material(moving,[lookup[k] for k in installed]);raw['native'][key]=q
            rec['samples'].append(dict(id=key,transform=M,translation_mm=v))
            if i==0:
                bb=m.bbox(moving);ob=m.bbox([lookup[k] for k in installed]);rec['initial_AABB_axis_gaps_mm']=np.maximum(ob[0]-bb[1],bb[0]-ob[1])
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:
                raw['first_blocked_path']=label;raw['first_blocked_sample']=key;return False
        rec['completed']=True;installed.update(names);save();return True
    near=[80.,60.,40.,20.,10.,5.,0.]
    reused=first['paths'][:2]
    assert all(p['completed'] for p in reused) and sum(len(p['samples']) for p in reused)==14
    assert all(not first['native'][s['id']]['overlaps'] and not first['native'][s['id']]['operation_failures'] for p in reused for s in p['samples'])
    raw['inherited_bench14_paths']=reused;raw['inherited_bench14_native_repeated']=0
    raw['bench_initial_present_names']=first['bench_initial_present_names']
    raw['bench_cradle_unit_present_names']=first['bench_cradle_unit_present_names']
    carrier_present=set(carrier);raw['carrier_initial_present_names']=sorted(carrier_present)
    blocked=not path('carrier_bench','motor_and_own_leads_into_adapter_before_base_nuts',unit,carrier_present,[[-x,0,0] for x in near])
    raw['bench_body19_completed']=not blocked and carrier_present==body
    allpacket={k for pp in packets for k in pp['names']};world=final-allpacket
    raw['world_initial_assumed_present_names']=sorted(world);raw['packet_initial_pending_names']=sorted(allpacket)
    if not blocked:
        original_body=next(z for z in old['paths'] if z['id']=='packet_'+name)
        assert original_body['completed'] and set(original_body['moving_names'])==body|{base} and set(original_body['present_names'])==world
        proofs=[dict(id=original_body['id'],moving_names=sorted(body),omitted_moving_names=[base],present_names=sorted(world),
            original_moving_names=original_body['moving_names'],original_present_names=original_body['present_names'],samples=original_body['samples'],proof='Moving subset of identical saved shapes/transforms,identical obstacles;all original tests clear. No new Boolean query.')]
        assert all(not old['native'][s['id']]['overlaps'] and not old['native'][s['id']]['operation_failures'] for s in original_body['samples'])
        world.update(body)
        for nut in [k for k in p['mounting_names'] if '_nut_' in k]:
            prior=next(z for z in old['paths'] if z['id']=='mount_'+nut)
            assert prior['completed'] and set(prior['present_names'])==world|{base}
            assert all(not old['native'][s['id']]['overlaps'] and not old['native'][s['id']]['operation_failures'] for s in prior['samples'])
            proofs.append(dict(id=prior['id'],moving_names=prior['moving_names'],omitted_obstacle_names=[base],present_names=sorted(world),
                original_moving_names=prior['moving_names'],original_present_names=prior['present_names'],samples=prior['samples'],proof='Identical moving shapes/transforms;obstacle subset with base_nuts explicitly later. No new Boolean query.'))
            world.add(nut)
        raw['inherited_clear_paths']=proofs;raw['world_after_inherited_paths_present_names']=sorted(world);save()
        for screw in [k for k in p['mounting_names'] if '_screw_' in k]:
            blocked=not path('world','rack_M3_before_base_nuts_'+screw,[screw],world,[[0,y,0] for y in [60.,40.,20.,10.,0.]])
            if blocked:break
        if not blocked:
            blocked=not path('world','four_M2_base_nuts_last',[base],world,[[x,0,0] for x in near])
    raw['restored_modules']=[name] if not blocked and set(p['names'])<=world else []
    raw['packet_final_present_names']=sorted(world);raw['packet_final_pending_names']=sorted(final-world)
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],
        overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],
        subset_inherited_native_groups=sum(len(z['samples']) for z in raw['inherited_clear_paths']),
        all16_packet_restoration_completed=set(world)==final,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['new_native_groups'],raw['overlap_records'],'restored',raw['restored_modules'],flush=True)


if __name__=='__main__':main()

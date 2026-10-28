"""New complete carrier/dualport bench and mapped48 world restoration,stop first block."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm075_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    start=time.monotonic();rows,_,source,original_packets=m.context();lookup={z['name']:z for z in rows};final=set(lookup)
    survey=n.p.read(HERE/'reviews/survey.json');old=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/check.json');old73=n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json');proofs=m.inheritance()
    packet=survey['mapped_special_packet'];body=set(m.mapped(old['body43_names']));unit=set(old['unit14_names']);world=set(old73['packet_final_present_names']);assert len(world)==670 and not world&set(packet['names'])
    carrier={'thumb_ip_actuator_adapter'};kit0=set(survey['original_port0_kit_names']);kit1={'thumb_ip_barrel_adjuster_1'};kit1_goal=set(survey['original_port1_kit_names'])
    radial=np.array(survey['radial']);normal=np.array(survey['normal']);opening=np.array(survey['guide_opening_axis']);frame=np.array(survey['guide_actual_frame'])
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    raw=dict(completed=False,source_sha256=source,native={},paths=[],original16packets=original_packets,original_special_packet=old['original_special_packet'],mapped_special_packet=packet,explicit_replacement={m.OLD:m.NAME},final886_names=sorted(final),body43_names=sorted(body),unit_reference12_names=old['unit_reference12_names'],unit14_names=old['unit14_names'],
        initial_world_present_names=sorted(world),restored_before=old['restored_before'],new_restored_modules=[],initial_carrier_bench_present_names=sorted(carrier),initial_kit1_present_names=sorted(kit1),port0_kit7_names=sorted(kit0),port1_kit7_names=sorted(kit1_goal),independent_inherited_proofs=proofs,inherited_sample_scope=56,inherited_native_groups_repeated=0,
        radial=radial,normal=normal,guide_opening_axis=opening,guide_actual_frame=frame,preserved_upper_M3_whole_cut=survey['preserved_upper_M3_whole_cut'],old_inclusion_Boolean_or_points_repeated=0,old_full49_or92_inherited_to_changed_support=False,new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,old_native_groups_repeated=0,internal_reference12_retested=False,
        conditional_scope='Reference12 motor/output preassembly remains unqualified internally. Exact independent motorbench14 and originalport0kit42 reused with support absent and same source/placement chain;not oldfull49/92. Every changed-support-dependent carrier/electrical/dualjamnut/wholekitentry path newly tested. Port1 actual7 independentbench then wholekit. Manualholding,bench-to-entry transport/neutralregistration,rigid nominal plug/threeleads,mating/harnessflex/captive/tools/threads/torque/strength unverified. Rear8/world670 retained. Newmapped48 has explicit ARM074 support1 replacement,other47 originalparts/identity. No old inclusion/point rerun or physical/current862 installation claim.')
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
    def bench(stage,label,names,present,axis):return path(stage,label,names,present,m.bench_vectors([lookup[k] for k in names],[lookup[k] for k in present],axis),axis)
    def run():
        inner='arm059_straight_jaw_1';outer='motor_housing_clamp_1_jaw_1'
        if not bench('kit1_bench','port1_inner_jaw_open_half',[inner],kit1,-radial):return False
        if not bench('kit1_bench','port1_opposite_jaw_capture',[outer],kit1,radial):return False
        for tag,axis in [('nut',-radial),('bolt',radial)]:
            for i in [0,1]:
                name=f'motor_housing_clamp_1_{tag}_{i}'
                if not bench('kit1_bench','port1_'+name,[name],kit1,axis):return False
        assert kit1==kit1_goal
        if not bench('carrier_bench','new_branched_base_before_motor',[m.NAME],carrier,radial):return False
        if not bench('carrier_bench','reference_unit14_and_own_plug',unit|{'thumb_ip_actuator_plug_outer'},carrier,-radial):return False
        if not bench('carrier_bench','three_leads_into_open_groove',{'thumb_ip_actuator_lead'+str(i) for i in [1,2,3]},carrier,opening):return False
        for i in [1,2]:
            nut='thumb_ip_actuator_lid_nut_'+str(i);side=(-1. if i==1 else 1.)*frame[:3,1];offset=-4.*opening;s=m.outside_start([lookup[nut]],[lookup[k] for k in carrier],side,offset)
            if not path('carrier_bench','lid_nut_side_then_pocket_'+str(i),[nut],carrier,[x*side+offset for x in m.inward_steps(s)]+[-2.*opening,np.zeros(3)],side):return False
        if not bench('carrier_bench','lid_after_three_leads',['thumb_ip_actuator_wire_guide_lid'],carrier,opening):return False
        for i in [1,2]:
            screw='thumb_ip_actuator_lid_screw_'+str(i)
            if not bench('carrier_bench','original_lid_M2_'+str(i),[screw],carrier,opening):return False
        assert len(carrier)==25
        for i,kit in [(0,kit0),(1,kit1)]:
            for label in ['top','bottom']:
                nut=f'thumb_ip_barrel_{label}_nut_{i}'
                if not bench('carrier_bench',f'port{i}_{label}_jamnut_before_wholekit',[nut],carrier,radial):return False
            if not bench('carrier_bench',f'port{i}_whole_original_kit7_into_new_base',kit,carrier,opening):return False
        assert carrier==body and len(carrier)==43;raw['dualport_carrier_bench_completed']=True;save()
        s=m.outside_start([lookup[k] for k in body],[lookup[k] for k in world],radial,np.zeros(3));steps=list(np.arange(s,20.,-20.))+[20.,15.,10.,5.,0.]
        if not path('world','mapped_special_body43_final_datum_radial',body,world,[x*radial for x in steps],radial):return False
        for nut in [k for k in packet['mounting_names'] if '_nut_' in k]:
            offset=-20.*normal;s=m.outside_start([lookup[nut]],[lookup[k] for k in world],radial,offset)
            if not path('world','rack_nut_'+nut,[nut],world,[x*radial+offset for x in m.inward_steps(s)]+[-10.*normal,np.zeros(3)],radial):return False
        for screw in [k for k in packet['mounting_names'] if '_screw_' in k]:
            offset=40.*normal;s=m.outside_start([lookup[screw]],[lookup[k] for k in world],radial,offset)
            if not path('world','rack_M3_'+screw,[screw],world,[x*radial+offset for x in m.inward_steps(s)]+[20.*normal,10.*normal,np.zeros(3)],radial):return False
        base='thumb_ip_actuator_base_nuts';s=m.outside_start([lookup[base]],[lookup[k] for k in world],radial,np.zeros(3))
        if not path('world','original_four_M2_base_nuts_last',[base],world,[x*radial for x in np.arange(s,20.,-20.)]+[20.*radial,10.*radial,5.*radial,np.zeros(3)],radial):return False
        assert set(packet['names'])<=world;raw['new_restored_modules']=['thumb_ip_actuator'];return True
    passed=run();total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],mapped_special48_restoration_completed=passed,
        dualport_carrier_bench_completed=raw.get('dualport_carrier_bench_completed',False),final_kit1_present_names=sorted(kit1),final_carrier_bench_present_names=sorted(carrier),packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),restored_modules=old['restored_before']+raw['new_restored_modules'],input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('completed',raw['new_native_groups'],raw['overlap_records'],'dualbench',raw['dualport_carrier_bench_completed'],'mapped48',passed,'world',len(world),flush=True)


if __name__=='__main__':main()

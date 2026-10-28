"""Place both rack nuts axially on the independent bench, then retain all later obstacles."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm076_selected',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/selected_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};final=set(lookup);survey=n.p.read(HERE/'reviews/survey.json');old=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/check.json')
    proofs=m.inheritance();world=set(survey['independent_world670_start_names']);carrier=set(survey['initial_carrier25_names']);body45=set(survey['body45_names']);packet=survey['mapped_special_packet'];radial=np.array(survey['radial']);normal=np.array(survey['normal']);opening=-normal
    assert len(world)==670 and len(carrier)==25 and len(body45)==45 and not world&body45
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__));source[(HERE/'reviews/check.json').relative_to(ROOT).as_posix()]=n.m.drive.sha(HERE/'reviews/check.json')
    raw=dict(completed=False,source_sha256=source,native={},paths=[],original16packets=packets,original_special_packet=old['original_special_packet'],mapped_special_packet=packet,explicit_replacement={m.OLD:m.NAME},final886_names=sorted(final),body43_names=survey['body43_names'],body45_names=sorted(body45),initial_world_present_names=sorted(world),initial_carrier_bench_present_names=sorted(carrier),preserved_previous_world714_names=survey['preserved_previous_world714_names'],explicitly_separated_own44_names=survey['explicitly_separated_own44_names'],restored_before=old['restored_before'],new_restored_modules=[],independent_inherited_proofs=proofs,inherited_sample_scope=162,inherited_native_groups_repeated=0,radial=radial,normal=normal,port0_kit7_names=survey['original_port0_kit_names'],port1_kit7_names=survey['original_port1_kit_names'],unit_reference12_names=old['unit_reference12_names'],unit14_names=old['unit14_names'],preserved_upper_M3_whole_cut=survey['preserved_upper_M3_whole_cut'],old_inclusion_Boolean_or_points_repeated=0,new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        preserved_initial_check='cad/preloaded_rack_nut_module_v1/reviews/check.json',rack_nut_bench_entry_axis=opening,initial5_failure_repeated=0,conditional_scope='Original nut bore +Y axial entry on independent bench removes the lateral crossing of electrical support/lidnut,with no worldWrist present. Preserve initial5/2records;not a shallower lateral start. Independent order alternative from explicit670,previous714 preserved. Reuse only identical independentbench98 and electricalcarrier64,not body43worldentry or laterport paths. Both racknuts installed on carrier25 before either port kit;each portjamnut/kitentry freshly tested with nuts present. Wholebody45 into all670 obstacles,then originalM3/base_nuts only if reached. No own fixed/moving duplicates. Reverse disassembly,manualholding,bench-to-entry transport/registration,reference12internal,mating/harnessflex/captive/tools/torque/forces remain unverified.')
    def save():n.a.save(out,raw)
    def path(stage,label,names,present,vectors,axis,events=()):
        names=sorted(names);assert not set(names)&present;rec=dict(id=label,stage=stage,moving_names=names,present_names=sorted(present),absent_names=sorted(final-present),approach_axis=axis,samples=[],completed=False);raw['paths'].append(rec);fixed=[lookup[k] for k in present];save()
        for i,v in enumerate(vectors):
            M=m.translate(v);key=label+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);q=n.c.material(moving,fixed);raw['native'][key]=q;rec['samples'].append(dict(id=key,translation_mm=v,transform=M,actual_face_event=any(np.linalg.norm(v-float(x)*radial)<1e-7 for x in events)))
            if i==0:
                mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);rec['initial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1]);assert rec['initial_projection_separation_mm']>0
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:raw['first_blocked_sample']=key;raw['first_blocked_path']=label;save();return False
        rec['completed']=True;present.update(names);save();return True
    def bench(label,names,axis):return path('carrier_bench',label,names,carrier,m.bench_vectors([lookup[k] for k in names],[lookup[k] for k in carrier],axis),axis)
    def run():
        for i in [1,2]:
            if not bench('preload_rack_nut_axial_'+str(i),[f'thumb_ip_actuator_rack_nut_{i}'],opening):return False
        raw['rack_nuts_preloaded_before_both_ports']=True;save()
        for i in [0,1]:
            for tag in ['top','bottom']:
                if not bench(f'preloaded_port{i}_{tag}_jamnut',[f'thumb_ip_barrel_{tag}_nut_{i}'],radial):return False
            if not bench(f'preloaded_port{i}_whole_original_kit7',raw[f'port{i}_kit7_names'],opening):return False
        assert carrier==body45;raw['body45_bench_completed']=True;save()
        start=m.outside_start([lookup[k] for k in body45],[lookup[k] for k in world],radial,np.zeros(3));steps=list(np.arange(start,20.,-20.))+[20.,15.,10.,5.,0.]
        if not path('world','whole_body45_original_datum_entry',body45,world,[x*radial for x in steps],radial):return False
        for screw in [k for k in packet['mounting_names'] if '_screw_' in k]:
            offset=40.*normal;start=m.outside_start([lookup[screw]],[lookup[k] for k in world],radial,offset)
            if not path('world','rack_M3_'+screw,[screw],world,[x*radial+offset for x in m.inward_steps(start)]+[20.*normal,10.*normal,np.zeros(3)],radial):return False
        base='thumb_ip_actuator_base_nuts';start=m.outside_start([lookup[base]],[lookup[k] for k in world],radial,np.zeros(3));mb=m.bbox([lookup[base]]);faces=[]
        for i,f in enumerate(lookup[m.NAME]['shape'].Faces()):
            if f.geomType()!='PLANE':continue
            pl=BRepAdaptor_Surface(f.wrapped).Plane();axis=np.array(pl.Axis().Direction().Coord());point=np.array(pl.Location().Coord());bb=np.array(n.c.m.pack._bbox(f))
            if abs(axis[0])>.999 and bb[0,1]<mb[1,1] and bb[1,1]>mb[0,1] and bb[0,2]<mb[1,2] and bb[1,2]>mb[0,2] and point[0]>mb[1,0]:faces.append(dict(index=i,point=point,normal=axis,bbox=bb))
        levels=sorted({round(float(z['point'][0]-(mb[0,0]+mb[1,0])/2.),9) for z in faces if 0<z['point'][0]-(mb[0,0]+mb[1,0])/2.<start});events=levels+[(x+y)/2. for x,y in zip(levels,levels[1:])];steps=sorted(set([float(x) for x in np.arange(start,20.,-20.)]+[20.,10.,5.,0.]+events),reverse=True)
        raw['base_nuts_actual_support_crossing_faces']=faces;raw['base_nuts_extra_axial_projection_samples_mm']=events;save()
        if not path('world','original_four_M2_base_nuts_with_rail_events',[base],world,[x*radial for x in steps],radial,events):return False
        assert set(packet['names'])<=world;raw['new_restored_modules']=['thumb_ip_actuator'];return True
    passed=run();total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],body45_bench_completed=raw.get('body45_bench_completed',False),mapped_special48_restoration_completed=passed,final_carrier_bench_present_names=sorted(carrier),packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),restored_modules=raw['restored_before']+raw['new_restored_modules'],input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('completed',raw['new_native_groups'],raw['overlap_records'],'body45',raw['body45_bench_completed'],'world',len(world),'mapped48',passed,flush=True)


if __name__=='__main__':main()

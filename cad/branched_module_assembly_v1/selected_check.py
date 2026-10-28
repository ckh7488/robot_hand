"""Keep completed body43 and enter original rack nuts laterally at final axial datum."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm075_selected',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    out=HERE/'reviews/selected_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source,_=m.context();lookup={z['name']:z for z in rows};final=set(lookup);first=n.p.read(HERE/'reviews/check.json')
    prefix=first['paths'][:-1];assert all(p['completed'] for p in prefix) and sum(len(p['samples']) for p in prefix)==161 and first['first_blocked_path']=='rack_nut_thumb_ip_actuator_rack_nut_1'
    assert all(not first['native'][s['id']]['overlaps'] and not first['native'][s['id']]['operation_failures'] for p in prefix for s in p['samples'])
    world=set(first['packet_final_present_names']);assert len(world)==713;radial=np.array(first['radial']);normal=np.array(first['normal']);packet=first['mapped_special_packet']
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__));source[(HERE/'reviews/check.json').relative_to(ROOT).as_posix()]=n.m.drive.sha(HERE/'reviews/check.json')
    raw=dict(first,completed=False,source_sha256=source,native={},paths=[],new_restored_modules=[],new_native_groups=0,inherited_new_successful_prefix_paths=prefix,inherited_new_prefix_samples=161,inherited_new_prefix_native_repeated=0,selected_world_start_present_names=sorted(world),selected_initial_carrier_present_names=first['final_carrier_bench_present_names'],selected_initial_kit1_present_names=first['final_kit1_present_names'],new_order='Racknuts enter from wholeoutside radial side at their original final axial datum,without the blocked+Y20 offset. Same body43+rear8/713 retained. Original M3 then base_nuts only if reached. Preserve initial170/first wristblock. Check thin-rail crossing events from actual new support faces,not just20mm intervals.')
    raw.pop('first_blocked_sample',None);raw.pop('first_blocked_path',None)
    def save():n.a.save(out,raw)
    def path(label,names,vectors,axis):
        names=sorted(names);assert not set(names)&world;rec=dict(id=label,stage='world',moving_names=names,present_names=sorted(world),absent_names=sorted(final-world),approach_axis=axis,samples=[],completed=False);raw['paths'].append(rec);save();fixed=[lookup[k] for k in world]
        for i,v in enumerate(vectors):
            M=m.translate(v);key=label+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);q=n.c.material(moving,fixed);raw['native'][key]=q;rec['samples'].append(dict(id=key,translation_mm=v,transform=M))
            if i==0:
                mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);rec['initial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1])
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:raw['first_blocked_sample']=key;raw['first_blocked_path']=label;save();return False
        rec['completed']=True;world.update(names);save();return True
    def run():
        for nut in [k for k in packet['mounting_names'] if '_nut_' in k]:
            s=m.outside_start([lookup[nut]],[lookup[k] for k in world],radial,np.zeros(3))
            if not path('rack_nut_final_axial_'+nut,[nut],[x*radial for x in m.inward_steps(s)],radial):return False
        for screw in [k for k in packet['mounting_names'] if '_screw_' in k]:
            offset=40.*normal;s=m.outside_start([lookup[screw]],[lookup[k] for k in world],radial,offset)
            if not path('rack_M3_'+screw,[screw],[x*radial+offset for x in m.inward_steps(s)]+[20.*normal,10.*normal,np.zeros(3)],radial):return False
        base='thumb_ip_actuator_base_nuts';s=m.outside_start([lookup[base]],[lookup[k] for k in world],radial,np.zeros(3));mb=m.bbox([lookup[base]])
        # Interior samples at both actual X faces and their midpoint catch a thin rail
        # that a 40-to-20mm step can cross without observing its material.
        faces=[]
        for i,f in enumerate(lookup[m.NAME]['shape'].Faces()):
            if f.geomType()!='PLANE':continue
            pl=BRepAdaptor_Surface(f.wrapped).Plane();axis=np.array(pl.Axis().Direction().Coord());point=np.array(pl.Location().Coord());bb=np.array(n.c.m.pack._bbox(f))
            if abs(axis[0])>.999 and bb[0,1]<mb[1,1] and bb[1,1]>mb[0,1] and bb[0,2]<mb[1,2] and bb[1,2]>mb[0,2] and point[0]>mb[1,0]:faces.append(dict(index=i,point=point,normal=axis,bbox=bb))
        levels=sorted({round(float(z['point'][0]-(mb[0,0]+mb[1,0])/2.),9) for z in faces if 0<z['point'][0]-(mb[0,0]+mb[1,0])/2.<s})
        events=levels+[(x+y)/2. for x,y in zip(levels,levels[1:])];steps=sorted(set([float(x) for x in np.arange(s,20.,-20.)]+[20.,10.,5.,0.]+events),reverse=True)
        raw['base_nuts_actual_support_crossing_faces']=faces;raw['base_nuts_extra_axial_projection_samples_mm']=events;save()
        if not path('original_four_M2_base_nuts_last_with_rail_events',[base],[x*radial for x in steps],radial):return False
        assert set(packet['names'])<=world;raw['new_restored_modules']=['thumb_ip_actuator'];return True
    passed=run();total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],mapped_special48_restoration_completed=passed,packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),restored_modules=first['restored_before']+raw['new_restored_modules'],input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete selected',raw['new_native_groups'],raw['overlap_records'],'mapped48',passed,'world',len(world),flush=True)


if __name__=='__main__':main()

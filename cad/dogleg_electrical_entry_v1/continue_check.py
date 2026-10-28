"""Preserve world731 and test a boundary-derived radial-to-opening entry."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm080_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    out=HERE/'reviews/final_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};final=set(lookup);boxes={k:m.corners(n.c.m.pack._bbox(z['shape'])) for k,z in lookup.items()}
    saved=HERE/'history/check_saved_complete90.json';raw=n.p.read(saved);assert len(raw['native'])==90 and len(raw['paths'])==4 and not raw['completed']
    assert all(n.m.drive.sha(ROOT/file)==h for file,h in raw['source_sha256'].items())
    world=set(raw['initial_world731_names']);assert len(world)==731
    for path in raw['paths']:
        assert path['completed'] and len(path['samples'])==path['planned_sample_count'] and set(path['present_names'])==world and not set(path['moving_names'])&world
        assert np.max(abs(np.array(path['samples'][-1]['transform'])-np.eye(4)))<1e-10
        assert all(not raw['native'][x['id']]['overlaps'] and not raw['native'][x['id']]['operation_failures'] for x in path['samples'])
        world.update(path['moving_names'])
    assert len(world)==737
    name=m.MODULE;p=next(z for z in packets if z['module']==name);base=name+'_base_nuts';f=raw['module_contracts'][0];radial=np.array(f['radial']);normal=np.array(f['normal']);prior=n.p.read(ROOT/'cad/open_electrical_world_assembly_v1/reviews/check.json')
    for file in [Path(__file__),saved,HERE/'history/check_initial_source.py.txt',HERE/'history/check_last_atomic.json',HERE/'history/check_initial_stdout.txt',HERE/'history/check_initial_stderr.txt',HERE/'history/check_initial_execution.json',HERE/'history/save_failure.json']:raw['source_sha256'][file.relative_to(ROOT).as_posix()]=n.m.drive.sha(file)
    raw.update(initial_execution_succeeded=False,saved_prefix_reused_no_native_repeat=90,saved_prefix_path_count=4,saved_prefix_source=saved.relative_to(ROOT).as_posix(),continuation_initial_world737_names=sorted(world),initial_source_preserved=True)
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
    if not blocked:
        for nut in [k for k in p['mounting_names'] if '_nut_' in k]:
            if not whole('world','rack_nut_'+nut,[nut],world,radial,-20.*normal,[-10.*normal,np.zeros(3)]):blocked=True;break
    if not blocked:
        for screw in [k for k in p['mounting_names'] if '_screw_' in k]:
            if not whole('world','rack_M3_'+screw,[screw],world,radial,40.*normal,[20.*normal,10.*normal,np.zeros(3)]):blocked=True;break
    if not blocked and not whole('world','four_original_M2_base_nuts_last',[base],world,radial):blocked=True
    if not blocked:
        assert set(p['names'])<=world;raw['new_restored_modules'].append(name);raw['module_contracts'][0]['completed']=True
    total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],restored_modules=prior['restored_modules']+raw['new_restored_modules'],packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),all16_packet_restoration_completed=world==final,input_hash_guard=all(n.m.drive.sha(ROOT/file)==h for file,h in source.items()),continuation_elapsed_s=time.monotonic()-started)
    save();print('complete',raw['new_native_groups'],raw['overlap_records'],'world',len(world),'restored',len(raw['restored_modules']),flush=True)
    raw['new_continuation_native_groups']=len(raw['native'])-90;n.a.save(out,raw)


if __name__=='__main__':main()

"""Reuse completed beam/rear-cap paths; prefit front nuts and enter through gap."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm066_selected',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/selected_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,membership,source=m.context();lookup={z['name']:z for z in rows};first=n.p.read(HERE/'reviews/check.json')
    assert first['completed'] and first['first_blocked_sample']=='bench_arm063_recessed_front_cap_2'
    for f in [Path(__file__),HERE/'reviews/check.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    inherited=[p for p in first['paths'] if p['completed']];assert len(inherited)==2
    inherited_ids=[z['id'] for p in inherited for z in p['samples']];assert len(inherited_ids)==15
    present={m.a.NAME,'Forearm_box_beam',m.a.CAP};final_names=set(lookup)
    nuts=sorted(k for k in first['bench_names'] if 'nut' in k);frontnuts=[k for k in nuts if k.startswith('arm063_')];assert len(frontnuts)==2
    raw=dict(completed=False,source_sha256=source,paths=[],native={},inherited_first15_sample_ids=inherited_ids,
        inherited_first15_results_repeated=0,first_vertical_front_cap_failure_preserved=True,
        initial_bench_present_names=sorted(present),final_reference_names=sorted(lookup),
        bench_names=first['bench_names'],packet_restoration_started=False,packet_restoration_completed=False,
        packet_pending_names=first['packet_restoration_initial_absent_names'],new_hardware_geometry=0,current862_replaced=False,
        proposal886_adopted=False,whole_proposal_raw_clear=False,inherited_ARM065_geometry_common1_mismatch_unresolved=True,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        rationale='Front cap vertical path crosses outer front spokes. Prefit its two captive nuts on an isolated bench,lower cap/nuts into beam gap atY=-190,then slide+Y27.5 to original station. All final geometry unchanged.',
        conditional_factory_scope='Root/beam/rear cap start inherited from15 completed samples. Isolated front cap/nut work and main bench are separate conditional starts;temporary hands/fixtures unmodeled. Subsequent packet restoration is not run until bench22 completes.')
    def save():n.a.save(out,raw)
    def path(label,names,vectors,obstacles,stage='bench'):
        rec=dict(id=label,stage=stage,moving_names=names,present_names=sorted(obstacles),
            absent_names=sorted(final_names-set(obstacles)),samples=[],completed=False,manual_support=True)
        raw['paths'].append(rec);save()
        for i,v in enumerate(vectors):
            key=label+'_'+str(i);q=n.c.material(m.moved([lookup[k] for k in names],v),[lookup[k] for k in obstacles])
            raw['native'][key]=q;rec['samples'].append(dict(id=key,translation_mm=np.array(v).tolist()));save()
            print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:
                raw['first_blocked_path']=label;raw['first_blocked_sample']=key;return False
        rec['completed']=True;return True
    cap='arm063_recessed_front_cap';isolated={cap};blocked=False
    for name in frontnuts:
        if not path('prefit_'+name,[name],[[0.,0.,z] for z in [40.,20.,10.,0.]],isolated,'isolated_front_cap'):
            blocked=True;break
        isolated.add(name)
    if not blocked:
        vectors=[[0.,-27.5,z] for z in [90.,70.,50.,30.,14.,7.,0.]]+[[0.,y,0.] for y in [-20.,-10.,0.]]
        blocked=not path('gap_front_cap_with_two_nuts',sorted(isolated),vectors,present)
        if not blocked:present.update(isolated)
    # Full bounding-clear starts for the remaining fasteners avoid claiming a
    # shallow near-seat pose as an accessible approach through the spokes.
    for name in [k for k in nuts if k not in frontnuts]:
        if blocked:break
        blocked=not path('insert_'+name,[name],[[0.,0.,z] for z in [100.,80.,60.,40.,20.,10.,0.]],present)
        if not blocked:present.add(name)
    bolts=sorted(k for k in first['bench_names'] if 'M3x12' in k)
    for name in bolts:
        if blocked:break
        blocked=not path('insert_'+name,[name],[[0.,0.,-z] for z in [100.,80.,60.,40.,20.,10.,0.]],present)
        if not blocked:present.add(name)
    for name in sorted(k for k in first['bench_names'] if k.startswith('arm064_output_')):
        if blocked:break
        blocked=not path('insert_'+name,[name],[[0.,y,0.] for y in [20.,10.,0.]],present)
        if not blocked:present.add(name)
    raw['bench_completed']=not blocked and present==set(first['bench_names']);raw['bench_final_present_names']=sorted(present)
    # Keep the original full packet protocol pending rather than inventing a
    # restoration pass if the support's own fastener approach still blocks it.
    assert blocked, 'Bench completed:implement original packet restoration before publishing.'
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],
        overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),
        raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('selected complete',raw['native_groups'],raw['overlap_records'],'bench',raw['bench_completed'],flush=True)


if __name__=='__main__':main()

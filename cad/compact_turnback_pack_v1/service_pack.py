"""Conditional assembly and local bore fit of the new three rigid guides."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import pack_model as a
n,k,np,ROOT,HERE=a.n,a.k,a.np,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def local_housings():
    result=[]
    for i in [0,1]:
        es=[n.cq.Edge.makeLine(n.cq.Vector(0,5,0),n.cq.Vector(0,-12,0))]+a.motor_edges()
        sh=a.b.sweep(es,1.5).cut(a.b.sweep(es,.65)).clean();sh=n.g.original.transform(sh,a.b.frame(i))
        result.append(dict(name='compact_motor_local_housing_'+str(i),shape=sh,body='forearm',kind='housing',physical_line=i,
            length_mm=sum(e.Length() for e in es),installed_part_candidate=False,local_reference_only=True,color=[.62,.21,.78]))
        es=a.saddle_edges(i);sh=a.b.sweep(es,1.5).cut(a.b.sweep(es,.65)).clean()
        result.append(dict(name='compact_saddle_local_housing_'+str(i),shape=sh,body='palm',kind='housing',physical_line=i,
            length_mm=sum(e.Length() for e in es),installed_part_candidate=False,local_reference_only=True,color=[.1,.48,.78]))
    assert all(z['shape'].isValid() and len(z['shape'].Solids())==1 for z in result)
    return result


def main():
    out=HERE/'reviews/service.json';assert not out.exists();start=time.monotonic()
    raw=n.p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['candidate_static_clear']
    ctx=k.context();source=dict(ctx[0][0]);rows=n.c.load(n.p.read(ROOT/raw['scene'])['parts'],source)
    removed=set(raw['replaced_hardware']);groups=[(key,[z for z in obs if z['name'] not in removed]) for key,obs in k.pose(ctx,raw['states'][0]['case'])]
    allrows=[z for _,obs in groups for z in obs]+rows;lookup={z['name']:z for z in allrows};assert len(lookup)==862
    report=dict(completed=False,source_sha256=source,stations=[],local_bore_fit=None,
        all_whole_housings_and_cords_absent_during_service=True,conditional_separate_station_starts=True,
        loose_parts_manually_held_until_fastened=True,actual_tools_jigs_threads_torque_preload_or_force_approved=False,
        whole_flexible_link_connected=False,old_service_samples_reused=0,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    def save():n.a.save(out,report)
    source['cad/compact_turnback_pack_v1/reviews/check.json']=n.m.drive.sha(HERE/'reviews/check.json')
    source['cad/compact_turnback_pack_v1/service_pack.py']=n.m.drive.sha(Path(__file__))
    plans=[]
    for i in [0,1]:
        tag='motor_housing_clamp_'+str(i);axis=a.b.frame(i)[:3,2]
        plans.append((tag,[(lookup['compact_motor_turnback_'+str(i)],-axis,[14,7,2,0]),
            (lookup[tag+'_jaw_1'],axis,[14,7,2,0])]+
            [(lookup[tag+'_nut_'+str(j)],-axis,[10,5,0]) for j in [0,1]]+
            [(lookup[tag+'_bolt_'+str(j)],axis,[10,5,0]) for j in [0,1]]))
    plans.append(('saddle',[(lookup['compact_saddle_nested_turns'],np.array([0,1,0]),[-20,-10,-5,-1,0]),
        (lookup['wrist_edge_clip_front_cap'],np.array([0,1,0]),[12,6,3,1,0])]+
        [(lookup['wrist_edge_clip_bolt_'+str(j)],np.array([0,1,0]),[20,10,5,1,0]) for j in [0,1]]+
        [(lookup['wrist_edge_clip_nut_'+str(j)],np.array([0,1,0]),[-20,-10,-5,-1,0]) for j in [0,1]]))
    for name,sequence in plans:
        names={z['name'] for z,_,_ in sequence};fixed=[z for z in allrows if z['name'] not in names];installed=[]
        station=dict(id=name,missing_at_start=sorted(names),fixed_obstacle_names=[z['name'] for z in fixed],paths=[])
        report['stations'].append(station);save()
        for row,axis,distances in sequence:
            path=dict(moving=row['name'],installed_before=[z['name'] for z in installed],absent_before=sorted(names-{z['name'] for z in installed}),
                obstacle_names=[z['name'] for z in fixed+installed],samples=[])
            station['paths'].append(path);save()
            for distance in distances:
                delta=distance*axis;test=n.c.material([dict(row,shape=row['shape'].translate(tuple(delta)))],fixed+installed)
                path['samples'].append(dict(distance_mm=distance,translation_world_mm=delta,native=test));save()
                print(name,row['name'],distance,len(test['overlaps']),flush=True)
            path['clear']=all(n.m.drive.clear(z['native']) for z in path['samples']);save()
            if not path['clear']:break
            installed.append(row)
        station['clear']=len(installed)==6
        station['final_hardware_names']=sorted([z['name'] for z in fixed+installed])
        save()
        if not station['clear']:
            report['stop_reason']='One conditional assembly path failed; do not inherit old installation approval or infer structural impossibility. Other station starts and local bore fits not executed.'
            break
    if len(report['stations'])==3 and all(z['clear'] for z in report['stations']):
        material=local_housings();tests={key:n.c.material(material,obs) for key,obs in groups}
        tests['new_guides']=n.c.material(material,rows)
        tests['internal']=k.flat([n.c.material([z],material[j+1:]) for j,z in enumerate(material)])
        report['local_bore_fit']=dict(native=tests,scene=a.export(material,'local_housing_fit'),
            clear=all(n.m.drive.clear(v) for v in tests.values()),rows=4,
            whole_material_length_budgets_or_insertion_force_approved=False)
        save()
    tests=[sample['native'] for station in report['stations'] for path in station['paths'] for sample in path['samples']]
    if report['local_bore_fit']:tests+=list(report['local_bore_fit']['native'].values())
    agg=k.flat(tests);report.update(completed=True,conditional_three_station_assembly_clear=len(report['stations'])==3 and all(z['clear'] for z in report['stations']),
        assembly_samples=sum(len(path['samples']) for station in report['stations'] for path in station['paths']),
        pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save()


if __name__=='__main__':main()

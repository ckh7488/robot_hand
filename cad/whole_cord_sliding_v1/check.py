"""Check actual whole cords with simultaneously posed rotor and child hardware."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import slide_model as s
from OCP.OSD import OSD_ThreadPool


def flat(tests):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),exact_pairs=sum(z['exact_pairs'] for z in tests),overlaps=[v for z in tests for v in z['overlaps']],operation_failures=[v for z in tests for v in z['operation_failures']])


def main():
    out=s.HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();source,groups,housing,rotor,child=s.context()
    source['cad/whole_cord_sliding_v1/check.py']=s.m.drive.sha(Path(__file__))
    report=dict(completed=False,source_sha256=source,profiles=[],states=[],rotor_names=sorted(rotor),child_names=sorted(child),
        parent_and_wrist_fixed_neutral=True,IP_target_sequence_deg=s.ANGLES,ideal_tail_no_slip_assumed_not_verified=True,
        same_whole_housings_stationary=True,new_hardware=0,hand_geometry_axes_or_motor_installation_changed=False,
        material_EI_gravity_friction_preload_or_force_solved=False,whole_parent_wrist_or_hand_motion_approved=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():s.a.save(out,report)
    save()
    for q in s.ANGLES:
        report['profiles'].append(dict(IP_deg=q,lines=[s.profile(q,i) for i in [0,1]]));save()
    # Neutral is the already saved CORD043 installation boundary, not a new
    # neutral native pass. Only new nonzero geometry states are checked here.
    alt=s.p.read(s.ROOT/'cad/motor_full_cord_thread_v1/reviews/anchor_check.json');boundary=[]
    for i in [0,1]:
        old=next(z for z in reversed(alt['cases']) if z['line']==i);meta=s.p.read(s.ROOT/old['scene'])['parts'];eds=s.paths(0.,i)
        error=max(max(s.np.linalg.norm(s.np.array(ed.startPoint().toTuple())-z['endpoint_start_mm']),s.np.linalg.norm(s.np.array(ed.endPoint().toTuple())-z['endpoint_end_mm'])) for ed,z in zip(eds,meta))
        assert len(eds)==len(meta) and error<1e-5;boundary.append(dict(line=i,maximum_endpoint_difference_mm=float(error),existing_scene=old['scene']))
    report['neutral_boundary']=boundary;save()
    for q in [15.,30.,45.]:
        posed,changed=s.hardware(groups,q,rotor,child);cord=s.cords(q);fixed=[z for _,zz in posed for z in zz if z['name'] not in rotor|child]
        tests={key:s.c.material(cord,zz) for key,zz in posed};tests['whole_housings']=s.c.material(cord,housing)
        tests['cord_self']=flat([s.c.material([z],cord[j+1:]) for j,z in enumerate(cord)])
        tests['moving_hardware_fixed']=s.c.material(changed,fixed);tests['moving_hardware_housings']=s.c.material(changed,housing)
        tests['rotor_child']=s.c.material([z for z in changed if z['name'] in rotor],[z for z in changed if z['name'] in child])
        lookup={z['name']:z for z in cord};known=[];blocking=[]
        for key,test in tests.items():
            for pair in test['overlaps']:
                x,y=lookup.get(pair['lead']),lookup.get(pair['part'])
                adjacent=key=='cord_self' and x and y and x['physical_line']==y['physical_line'] and {x['path_segment'],y['path_segment']}=={0,1}
                (known if adjacent else blocking).append(dict(group=key,pair=pair))
        ok=not blocking and not any(z['operation_failures'] for z in tests.values())
        state=dict(IP_deg=q,motor_output_deg=.77*q,native=tests,known_same_cord_motor_joint_records=known,blocking_records=blocking,
            raw_all_clear=all(s.m.drive.clear(z) for z in tests.values()),conditional_motion_can_continue=ok,
            rigid_group_internal_pairs_not_recomputed='Rotor9 and child13 each preserve their own relative geometry; old nominal thread/reference contacts not reclassified as zero.',
            changed_hardware_names=[z['name'] for z in changed])
        report['states'].append(state);save()
        state['scene']=s.export(cord+changed,'q_'+str(int(q)));save()
        print('q',q,'motor',.77*q,'blocking',len(blocking),'same_cord',len(known),flush=True)
        if not ok:break
    agg=flat([v for z in report['states'] for v in z['native'].values()])
    report.update(completed=True,nonzero_native_states=len(report['states']),conditional_IP_path_clear=len(report['states'])==3 and all(z['conditional_motion_can_continue'] for z in report['states']),
        raw_all_clear=all(z['raw_all_clear'] for z in report['states']),pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],
        overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
        reverse_geometry_reuses_forward_finite_states=True,physical_hysteresis_or_continuous_motion_verified=False,
        old_curve_family_optimization_Newton_or_distance_field_runs=0,input_hash_guard=all(s.m.drive.sha(s.ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save()


if __name__=='__main__':main()

"""Continue the saved 25% state through parent approach/release and wrist.

All material budgets and numerical arc coordinates are inherited unchanged.
Only new increments are solved. Sampled contact and finite native snapshots
remain geometric evidence, never calibrated force or continuous-motion proof.
"""
import sys,time,copy
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/coupled_material_continuation_v1'))
import continuation as n
k=n.m.drive.module('_motion047_material',ROOT/'cad/coupled_material_continuation_v1/check.py')
from OCP.OSD import OSD_ThreadPool


def export(rows,label):
    return n.a.a.prior.export([{key:value for key,value in z.items() if key not in ['step','mesh','sha256_step','sha256_stl']} for z in rows],HERE/'generated'/label)


def plan(target):
    result=[]
    for percent in range(30,101,5):
        result.append(dict(id='approach_'+str(percent),phase='approach',parent_fraction=percent/100.,case=n.case_at(target*percent/100.),
            check_material=percent in [30,50,75,100]))
    for percent in [90,80,75,70,60,50,40,30,25,20,10,0]:
        result.append(dict(id='release_'+str(percent),phase='release',parent_fraction=percent/100.,case=n.case_at(target*percent/100.),
            check_material=percent in [75,50,25,0]))
    for sign in [-1,1]:
        for direction,values in [('out',[.25,.5,.75,1.]),('back',[.75,.5,.25,0.])]:
            for f in values:
                result.append(dict(id='wrist_'+str(sign)+'_'+direction+'_'+str(f),phase='wrist',parent_fraction=0.,
                    case=n.case_at(n.np.zeros(20),sign*20*f,sign*15*f),check_material=(direction=='out' and f in [.5,1.]) or (direction=='back' and f==0.)))
    return result


def classify(state,rows):
    lookup={z['name']:z for z in rows};known=[];blocking=[]
    for key,test in state['native'].items():
        for pair in test['overlaps']:
            a,b=lookup.get(pair['lead']),lookup.get(pair['part'])
            same=key=='self' and a and b and a['kind']==b['kind']=='cord' and a['physical_line']==b['physical_line'] and {a['path_segment'],b['path_segment']}=={0,1}
            (known if same else blocking).append(dict(group=key,pair=pair))
    state.update(known_same_cord_motor_seams=known,blocking_records=blocking,raw_clear=all(n.m.drive.clear(z) for z in state['native'].values()),
        conditional_geometry_clear=not blocking and not any(z['operation_failures'] for z in state['native'].values()))


def main():
    out=HERE/'reviews/check.json';assert not out.exists();started=time.monotonic()
    previous=n.p.read(n.HERE/'reviews/analysis.json');checks=n.p.read(n.HERE/'reviews/check.json')
    assert previous['completed'] and checks['all_five_planned_parent_increments_completed']
    seed=previous['steps'][-1];assert seed['solved'] and seed['parent_fraction']==.25
    x=n.np.array(seed['parameters']);weights=[[n.np.array(z) for z in row] for row in previous['arc_length_weights']];budgets=n.np.array(previous['free_contour_budgets_mm'])
    target=n.np.array(previous['target_parent_angles_deg']);sequence=plan(target);ctx=k.context();source=dict(ctx[0][0])
    for rel in ['cad/parent_wrist_material_motion_v1/run_motion.py','cad/coupled_material_continuation_v1/continuation.py',
                'cad/coupled_material_continuation_v1/check.py','cad/coupled_material_continuation_v1/reviews/analysis.json',
                'cad/coupled_material_continuation_v1/reviews/check.json','docs/reviews/CONT046/core_initial.json',
                'cad/whole_cord_sliding_v1/slide_model.py',checks['states'][-1]['scene']]:source[rel]=n.m.drive.sha(ROOT/rel)
    report=dict(completed=False,source_sha256=source,seed_parent_fraction=.25,seed_parameters=x,seed_case=seed['case'],seed_scene=checks['states'][-1]['scene'],
        prior_five_increments_resolved=0,prior_native_groups_reexecuted=0,arc_length_weights=weights,free_contour_budgets_mm=budgets,
        target_parent_angles_deg=target,plan=sequence,increments=[],states=[],profiles=[],failed_attempts=[],
        numerical_half_step_limit=1,half_steps_are_not_joint_angle_limits=True,actual_rotor_pose_used_in_new_point_contact_projection=True,
        hardware_rows=862,hand_rows=397,packaging_rows=385,electrical_rows=64,arm_rows=10,PAIR_hardware_rows=6,new_hardware=0,
        representative_thumb_IP_two_cords_only=True,other_19_motor_routes_integrated=False,whole_hand_drive_approved=False,
        tail_no_slip_conditional=True,housing_end_seating_nominal=True,calibrated_EI_gravity_twist_friction_preload_force_tension_or_equilibrium_solved=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():n.a.save(out,report)
    def solve(previous_x,case,label,phase,fraction,parent_id=None):
        groups=k.pose(ctx,case);allhw=[z for _,rows in groups for z in rows];assert len(allhw)==862
        print('begin',label,'actual pose contact projection',flush=True)
        fresh,rec=n.advance(previous_x,case,weights,budgets,allhw)
        rec.update(id=label,phase=phase,parent_fraction=fraction,parent_plan_id=parent_id)
        report['increments'].append(rec);save()
        print('solve',label,rec['solved'],'residual',rec['endpoint_scaled_residual'],'inequality',rec['minimum_inequality'],flush=True)
        return fresh,rec,groups
    def native(x,rec,groups):
        profiles=[k.profile(x,rec['case'],weights,budgets,kind,i) for kind in ['housing','cord'] for i in [0,1]]
        report['profiles'].append(dict(id=rec['id'],materials=profiles));save()
        rows=k.build(x,rec['case'],weights,budgets)
        state=dict(id=rec['id'],phase=rec['phase'],parent_fraction=rec['parent_fraction'],case=rec['case'],rows=len(rows),native={})
        report['states'].append(state);save()
        for key,obs in groups:
            state['native'][key]=n.c.material(rows,obs);save();print('native',rec['id'],key,len(state['native'][key]['overlaps']),flush=True)
        state['native']['self']=k.flat([n.c.material([z],rows[j+1:]) for j,z in enumerate(rows)]);save()
        classify(state,rows);state['scene']=export(rows,rec['id']);save()
        print('state',rec['id'],'blocking',len(state['blocking_records']),'same_cord',len(state['known_same_cord_motor_seams']),flush=True)
        return state['conditional_geometry_clear']
    save();last_case=seed['case'];last_fraction=.25;stop_reason=None
    for entry in sequence:
        fresh,rec,groups=solve(x,entry['case'],entry['id'],entry['phase'],entry['parent_fraction'])
        if not rec['solved']:
            report['failed_attempts'].append(dict(id=rec['id'],reason='Incremental projection did not satisfy sampled endpoint/contact conditions; not an actual swept-material collision or structural impossibility.'))
            save()
            # One numerical bisection retains the requested natural target.
            # It does not trim CAD, change budgets or restrict joint motion.
            q=(n.np.array(last_case['q'])+entry['case']['q'])/2
            flex=(last_case['wrist_flex_deg']+entry['case']['wrist_flex_deg'])/2
            yaw=(last_case['wrist_yaw_deg']+entry['case']['wrist_yaw_deg'])/2
            midpoint=n.case_at(q,flex,yaw);fraction=(last_fraction+entry['parent_fraction'])/2
            mid,sub,_=solve(x,midpoint,entry['id']+'_half',entry['phase'],fraction,entry['id'])
            if not sub['solved']:stop_reason='projection_failed_after_one_numerical_bisection';break
            fresh,rec,groups=solve(mid,entry['case'],entry['id']+'_after_half',entry['phase'],entry['parent_fraction'],entry['id'])
            if not rec['solved']:stop_reason='target_projection_failed_after_successful_half_increment';break
        x=fresh;last_case=entry['case'];last_fraction=entry['parent_fraction']
        if entry['check_material'] and not native(x,rec,groups):stop_reason='first_new_native_material_obstruction';break
        report['last_accepted_increment_id']=rec['id'];save()
    agg=k.flat([z for state in report['states'] for z in state['native'].values()])
    report.update(completed=True,stop_reason=stop_reason,all_planned_targets_reached=stop_reason is None,
        new_centerline_solved_increments=sum(z['solved'] for z in report['increments']),new_centerline_attempts=len(report['increments']),
        new_native_states=len(report['states']),pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],
        overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),raw_all_clear=bool(report['states']) and all(z['raw_clear'] for z in report['states']),
        all_executed_swept_states_clear=bool(report['states']) and all(z['conditional_geometry_clear'] for z in report['states']),
        whole_parent_wrist_continuous_or_physical_motion_approved=False,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save()


if __name__=='__main__':main()

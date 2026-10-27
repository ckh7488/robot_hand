"""One explicit length trial, then coupled same-material continuation."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import compact_model as a
n,k,np,ROOT,HERE=a.n,a.k,a.np,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def length_choice(basis):
    neutral=basis['required_movement_chord_bounds'][0]['lines']
    surplus=np.array(basis['neutral_seed_free_contours_mm'])-np.array([r['necessary_free_contour_lower_bound_mm'] for r in neutral])
    budgets=np.array(basis['maximum_required_chord_bound_mm'])+surplus
    return budgets,dict(derivation='Maximum of required-motion chord-sum lower bounds plus the neutral analytic/retained-bend surplus over its own chord sum. Explicit one-trial geometric reserve, not supplier length, force equilibrium, a feasibility proof or optimized minimum.',
        neutral_bend_surplus_mm=surplus,maximum_required_chord_bound_mm=basis['maximum_required_chord_bound_mm'],new_free_contour_budgets_mm=budgets)


def profile(x,case,x0,weights,budgets,i,kind):
    edges=a.paths(x,case,weights,budgets,i,kind);zero=a.paths(x0,n.case_at(np.zeros(20)),weights,budgets,i,kind)
    L=sum(e.Length() for e in zero);actual=sum(e.Length() for e in edges)
    gap=max((left.endPoint()-right.startPoint()).Length for left,right in zip(edges[:-1],edges[1:]))
    assert abs(actual-L)<1e-5 and gap<1e-5,(kind,i,actual-L,gap)
    idx=3+n.NUP+(6 if kind=='cord' else 0);rear=sum(e.Length() for e in edges[:idx]);rear0=sum(e.Length() for e in zero[:idx])
    station=rear0+20.;local=station-rear;guide=edges[idx:idx+2];guideL=sum(e.Length() for e in guide)
    point=np.array(n.t.point_at(edges,station));inside=0<=local<=guideL
    error=float(np.linalg.norm(point-n.t.point_at(guide,local))) if inside else None
    witness=dict(material_s_mm=station,world_mm=point,location_from_guide_rear_mm=local,inside_actual_whole_bore=bool(inside),
        guide_centerline_length_mm=guideL,actual_guide_axis_residual_mm=error,material_witness_reset=False)
    tails=None
    if kind=='cord':
        O,R=n.g.g.v.frames(n.g.DESIGN,np.array(case['q']))['thumb_ip'];A,B=n.g.g.v.frames(n.g.DESIGN,np.zeros(20))['thumb_ip']
        C=np.array(case['H'])@n.s.transform(O,R)@np.linalg.inv(n.s.transform(A,B));M=n.s.frames(case['q'][19])[1]
        tails=[float(np.linalg.norm(np.array(e.toTuple())-(T@np.r_[p.toTuple(),1.])[:3])) for e,T,p in
            [(edges[0].startPoint(),M,zero[0].startPoint()),(edges[-1].endPoint(),C,zero[-1].endPoint())]]
        assert max(tails)<1e-5
    return dict(kind=kind,line=i,material_length_mm=L,actual_length_mm=actual,maximum_join_gap_mm=gap,
        piece_lengths_mm=[e.Length() for e in edges],guide_rear_material_s_mm=rear,guide_rear_material_shift_mm=rear-rear0,
        material_points=[dict(material_s_mm=float(s),world_mm=n.t.point_at(edges,float(s))) for s in np.linspace(0,L,9)],
        bore_material_witness=witness,conditional_tail_no_slip_frame_errors_mm=tails)


def sequence(target):
    result=[dict(id='approach_'+str(p),phase='approach',case=n.case_at(target*p/100.),check_native=p in [25,50,75,100]) for p in range(5,101,5)]
    result += [dict(id='release_'+str(p),phase='release',case=n.case_at(target*p/100.),check_native=p in [50,0]) for p in [90,80,75,70,60,50,40,30,25,20,10,0]]
    for sign in [-1,1]:
        for direction,values in [('out',[.25,.5,.75,1.]),('back',[.75,.5,.25,0.])]:
            for f in values:result.append(dict(id='wrist_'+str(sign)+'_'+direction+'_'+str(f),phase='wrist',case=n.case_at(np.zeros(20),sign*20*f,sign*15*f),
                check_native=(direction=='out' and f==1.) or (sign==1 and direction=='back' and f==0.)))
    return result


def classify(state,rows):
    lookup={z['name']:z for z in rows};known=[];blocking=[]
    for key,test in state['native'].items():
        for pair in test['overlaps']:
            left,right=lookup.get(pair['lead']),lookup.get(pair['part'])
            same=key=='self' and left and right and left['kind']==right['kind']=='cord' and left['physical_line']==right['physical_line'] and {left['path_segment'],right['path_segment']}=={0,1}
            (known if same else blocking).append(dict(group=key,pair=pair))
    state.update(raw_clear=all(n.m.drive.clear(z) for z in state['native'].values()),known_same_cord_motor_seams=known,new_blocking_records=blocking,
        conditional_geometric_clear=not blocking and not any(z['operation_failures'] for z in state['native'].values()))


def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic()
    basis=n.p.read(HERE/'reviews/length_basis.json');assert basis['completed']
    budgets,choice=length_choice(basis);weights=[[np.array(z) for z in row] for row in basis['arc_length_weights']]
    seed=np.array(basis['neutral_parameters']);oldbudgets=np.array(basis['neutral_seed_free_contours_mm'])
    for i in [0,1]:seed[i*n.N+n.N-1]+=budgets[i]-oldbudgets[i]
    ctx,hw=a.context();source=dict(ctx[0][0]);source.update(basis['source_sha256'])
    for f in ['compact_model.py','length_basis.py','run_link.py','reviews/length_basis.json']:source[(HERE/f).relative_to(ROOT).as_posix()]=n.m.drive.sha(HERE/f)
    target=np.array(basis['target_parent_angles_deg']);plan=sequence(target)
    report=dict(completed=False,source_sha256=source,length_choice=choice,arc_length_weights=weights,free_contour_budgets_mm=budgets,
        target_parent_angles_deg=target,plan=plan,increments=[],states=[],profiles=[],stop_reason=None,
        preserved_old_MOTION047_housing_lengths_mm=[698.964345153791,761.108456948670],preserved_old_MOTION047_cord_lengths_mm=[853.356663204892,932.491923574900],
        previous_motion_native_or_local_hardware_checks_reexecuted=0,hand_rows=397,base_hand_rows=364,hardware_rows=862,
        new_hardware=0,PACK049_replacement_hardware=3,subdivision_nodes_are_supports=False,
        actual_motor_rotor_in_point_field=True,physical_EI_gravity_twist_friction_force_tension_preload_equilibrium_solved=False,
        no_slip_tails_and_housing_seating_are_conditional=True,whole_hand_drive_or_exterior_approved=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():n.a.save(out,report)
    save()
    def solve(previous,entry):
        groups=a.pose(ctx,hw,entry['case']);print('begin',entry['id'],flush=True)
        x,rec=n.advance(previous,entry['case'],weights,budgets,[z for _,rows in groups for z in rows])
        rec.update(id=entry['id'],phase=entry['phase']);report['increments'].append(rec);save()
        print('solve',entry['id'],rec['solved'],rec['endpoint_scaled_residual'],rec['minimum_inequality'],flush=True)
        return x,rec,groups
    def native(x,rec,groups,x0):
        rows=a.build(x,rec['case'],weights,budgets)
        state=dict(id=rec['id'],phase=rec['phase'],case=rec['case'],parameters=x,rows=len(rows),native={});report['states'].append(state)
        profiles=[profile(x,rec['case'],x0,weights,budgets,i,kind) for kind in ['housing','cord'] for i in [0,1]]
        report['profiles'].append(dict(id=rec['id'],materials=profiles));save()
        for key,obs in groups:
            state['native'][key]=n.c.material(rows,obs);save();print('native',rec['id'],key,len(state['native'][key]['overlaps']),flush=True)
        state['native']['self']=k.flat([n.c.material([z],rows[j+1:]) for j,z in enumerate(rows)])
        classify(state,rows);state['scene']=a.export(rows,rec['id']);save()
        print('native done',rec['id'],len(state['new_blocking_records']),flush=True)
        return state['conditional_geometric_clear']
    entry=dict(id='neutral_length_trial',phase='initial',case=n.case_at(np.zeros(20)))
    x,rec,groups=solve(seed,entry);x0=x.copy();report['neutral_parameters']=x0;save()
    if not rec['solved']:report['stop_reason']='neutral_same_length_contact_projection_failed'
    elif not native(x,rec,groups,x0):report['stop_reason']='neutral_native_block'
    else:
        for entry in plan:
            fresh,rec,groups=solve(x,entry)
            if not rec['solved']:report['stop_reason']='increment_projection_failed_'+entry['id'];break
            report['profiles'].append(dict(id=rec['id'],materials=[profile(fresh,rec['case'],x0,weights,budgets,i,kind) for kind in ['housing','cord'] for i in [0,1]])) if not entry['check_native'] else None
            if entry['check_native'] and not native(fresh,rec,groups,x0):report['stop_reason']='new_native_block_'+entry['id'];break
            x=fresh;save()
    tests=[v for state in report['states'] for v in state['native'].values()];agg=k.flat(tests)
    report.update(completed=True,all_planned_material_motion_completed=report['stop_reason'] is None,
        increments_solved=sum(z['solved'] for z in report['increments']),increments_attempted=len(report['increments']),native_material_states=len(report['states']),
        pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],raw_overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('finished',report['stop_reason'],report['increments_solved'],report['native_material_states'],flush=True)


if __name__=='__main__':main()

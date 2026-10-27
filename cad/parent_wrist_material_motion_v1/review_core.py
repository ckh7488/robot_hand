"""Exact-commit replay of saved new material states, without re-solving motion."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_motion as d
n,k,ROOT,HERE=d.n,d.k,d.ROOT,d.HERE
gw=n.m.drive.module('_motion047_review_witness',HERE/'guide_witnesses.py')
hw=n.m.drive.module('_motion047_review_hardware',HERE/'moving_hardware.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/MOTION047.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=n.p.read(HERE/'reviews/check.json');witness=n.p.read(HERE/'reviews/guide_witnesses.json');ev=n.p.read(HERE/'reviews/evidence.json');hardware=n.p.read(HERE/'reviews/moving_hardware.json');ctx=k.context()
    weights=[[n.np.array(z) for z in row] for row in raw['arc_length_weights']];budgets=n.np.array(raw['free_contour_budgets_mm']);increments={z['id']:z for z in raw['increments']}
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],kinematic_steps=[],profiles=[],guide_witnesses=[],geometry_comparisons=[],adaptive_common_pairs=[])
    def save():n.a.save(args.output,result)
    for rec in raw['increments']:
        x=n.np.array(rec['parameters']);error=float(max(abs(n.endpoints(x,rec['case'],weights,budgets))))
        ok=abs(error-rec['endpoint_scaled_residual'])<1e-10
        result['kinematic_steps'].append(dict(id=rec['id'],original_solved=rec['solved'],endpoint_scaled_residual=error,matched=bool(ok)))
        if not ok:result['differences'].append('endpoint_'+rec['id'])
    states=raw['states'];indices={0,len(states)-1,max(range(len(states)),key=lambda j:states[j]['parent_fraction']),
        min(range(len(states)),key=lambda j:states[j]['case']['wrist_flex_deg']),max(range(len(states)),key=lambda j:states[j]['case']['wrist_flex_deg'])}
    hardware_wants={z['id']:z for z in hardware['states']};hardware_ids={z['id'] for z in [hardware['states'][0],hardware['states'][-1]]}
    hardware_ids.update(states[j]['id'] for j in indices if states[j]['id'] in hardware_wants)
    for pair in ev['new_common_pairs']:indices.add(next(j for j,z in enumerate(states) if z['id']==pair['id']))
    chosen=[states[j] for j in sorted(indices)];saved_by_id={}
    result['native_selection_rule']='First new snapshot, maximum parent fraction, minimum/maximum actual wrist flexion, final snapshot; duplicate state IDs run once.'
    result['hardware_native_selection_rule']='First/last completed hardware snapshots plus the selected material snapshot IDs that have a hardware result; duplicate IDs run once, two groups per state.'
    for rec in chosen:
        rows=n.c.load(n.p.read(ROOT/rec['scene'])['parts']);saved_by_id[rec['id']]=rows;groups=k.pose(ctx,rec['case'])
        tests={key:n.c.material(rows,obs) for key,obs in groups};tests['self']=k.flat([n.c.material([z],rows[j+1:]) for j,z in enumerate(rows)])
        for key,got in tests.items():
            want=rec['native'][key];aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']}
            error=max([abs(aa[q]-bb[q]) for q in aa.keys()&bb.keys()]+[0.])
            ok=aa.keys()==bb.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            label=rec['id']+'/'+key;result['native_samples'].append(dict(id=label,native=got,maximum_basic_volume_difference_mm3=error,matched=bool(ok)))
            if not ok:result['differences'].append(label)
            print(label,ok,flush=True);save()
    for ident in sorted(hardware_ids,key=lambda name:next(j for j,z in enumerate(states) if z['id']==name)):
        rec=next(z for z in states if z['id']==ident);changed,rest,rotor,child=hw.sets(ctx,rec['case'])
        for key,got in [('actuated22_vs_other840',n.c.material(changed,rest)),('rotor9_vs_child13',n.c.material(rotor,child))]:
            want=hardware_wants[ident]['native'][key];aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']}
            err=max([abs(aa[q]-bb[q]) for q in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            label=ident+'/'+key;result['native_samples'].append(dict(id=label,native=got,maximum_basic_volume_difference_mm3=err,matched=bool(ok)))
            if not ok:result['differences'].append(label)
            print(label,ok,flush=True);save()
    cases={z['id']:z['case'] for z in raw['states']};wants={z['id']:z['witnesses'] for z in witness['states']}
    for state in raw['profiles']:
        rec=increments[state['id']];x=n.np.array(rec['parameters'])
        for want,want_guide in zip(state['materials'],wants[state['id']]):
            got=k.profile(x,rec['case'],weights,budgets,want['kind'],want['line']);errors=[abs(got[q]-want[q]) for q in ['actual_length_mm','maximum_join_gap_mm','PAIR_rear_material_s_mm','PAIR_rear_material_coordinate_change_mm']]
            errors+=[float(max(abs(n.np.array(got['piece_lengths_mm'])-want['piece_lengths_mm'])))]
            errors+=[float(max(abs(n.np.array(a['world_mm'])-b['world_mm']))) for a,b in zip(got['material_points'],want['material_points'])]
            errors+=[float(max(abs(n.np.array(got['axis_sliding_witness']['world_mm'])-want['axis_sliding_witness']['world_mm'])))]
            ok=max(errors)<1e-5;result['profiles'].append(dict(id=state['id'],kind=want['kind'],line=want['line'],maximum_difference=max(errors),matched=bool(ok)))
            if not ok:result['differences'].append('profile_'+state['id']+'_'+want['kind']+'_'+str(want['line']))
            guide=gw.evaluate(cases[state['id']],got);gerr=max(abs(guide['guide_path_station_mm']-want_guide['guide_path_station_mm']),float(max(abs(n.np.array(guide['world_mm'])-want_guide['world_mm']))))
            if guide['actual_path_tangent_world'] is not None:gerr=max(gerr,float(max(abs(n.np.array(guide['actual_path_tangent_world'])-want_guide['actual_path_tangent_world']))))
            gok=gerr<1e-5 and guide['inside_actual_PAIR_path']==want_guide['inside_actual_PAIR_path'] and guide['inside_first_40mm_straight']==want_guide['inside_first_40mm_straight'] and guide['actual_guide_segment_index']==want_guide['actual_guide_segment_index']
            result['guide_witnesses'].append(dict(id=state['id'],kind=want['kind'],line=want['line'],maximum_difference=gerr,matched=bool(gok),actual_guide_witness=guide))
            if not gok:result['differences'].append('guide_'+state['id']+'_'+want['kind']+'_'+str(want['line']))
            save()
    peak=max(states,key=lambda z:z['parent_fraction']);negative=min(states,key=lambda z:z['case']['wrist_flex_deg'])
    metric_states={peak['id']:peak,(negative if negative['case']['wrist_flex_deg']<0 else states[-1])['id']:(negative if negative['case']['wrist_flex_deg']<0 else states[-1])}
    result['metric_selection_rule']='Eight free housing arcs at maximum parent fraction and at maximum negative wrist flexion (or final snapshot if no negative wrist snapshot), duplicate IDs once.'
    names=['continued_housing_'+str(i)+'_'+str(j) for i in [0,1] for j in [3,3+n.NUP//2,3+n.NUP+4,3+n.NUP+4+n.NDOWN//2]]
    for state in metric_states.values():
        rec=increments[state['id']];fresh={z['name']:z for z in k.build(n.np.array(rec['parameters']),rec['case'],weights,budgets)};saved={z['name']:z for z in saved_by_id[state['id']]}
        for name in names:
            got=dict(id=state['id']+'/'+name,name=name,**n.s.e.measure.match(n.s.e.measure.metrics(fresh[name]['shape']),n.s.e.measure.metrics(saved[name]['shape'])));result['geometry_comparisons'].append(got)
            if not got['matched']:result['differences'].append('metric_'+state['id']+'/'+name)
            save()
    for z in ev['new_common_pairs']:
        last=next(v for v in states if v['id']==z['id'])
        pair=z['pair'];lookup={v['name']:v for v in [u for _,zz in k.pose(ctx,last['case']) for u in zz]+saved_by_id[last['id']]}
        sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6);values=n.s.e.measure.metrics(sh)['global_values'];err=float(max(abs(n.np.array(values)-z['adaptive_common_values'])));ok=err<1e-5
        result['adaptive_common_pairs'].append(dict(pair=pair,STEP_common_values=values,source_common_values=z['adaptive_common_values'],maximum_difference=err,matched=bool(ok)))
        if not ok:result['differences'].append('common_'+pair['lead'])
        save()
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),manifest_files=len(manifest['files']),frozen_files=len(frozen),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        solver_reexecuted=False,prior_CONT046_native_reexecuted=0,independent_audit=False,physical_or_continuous_motion_approved=False,
        inherited_SLIDE045_CORD043_BIARC039_false_preserved=True)
    save()


if __name__=='__main__':main()

"""Exact-commit bounded replay, preserving failed and refined native lists."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_link as check
import check_hardware as hardware_check
a,n,k,np,ROOT,HERE=check.a,check.n,check.k,check.np,check.ROOT,check.HERE
measure=n.m.drive.module('_link050_review_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool


def material_shape(row,state,report):
    weights=[[np.array(z) for z in line] for line in report['arc_length_weights']];budgets=np.array(report['free_contour_budgets_mm'])
    e=a.paths(np.array(state['parameters']),state['case'],weights,budgets,row['physical_line'],row['kind'])[row['path_segment']]
    return n.m.swept(e,1.5).cut(n.m.swept(e,.65)).clean() if row['kind']=='housing' else n.m.swept(e,.3)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/LINK050.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=n.p.read(HERE/'reviews/check.json');refined=n.p.read(HERE/'reviews/refined.json');continuous=n.p.read(HERE/'reviews/continuous.json');basis=n.p.read(HERE/'reviews/length_basis.json');hwraw=n.p.read(HERE/'reviews/hardware.json')
    ctx,hw=a.context();result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],profiles=[],increment_kinematics=[],chord_bounds=[],planned_common_pairs=[])
    def save():n.a.save(args.output,result)
    def compare(label,got,want):
        gg={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};ww={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']}
        err=max([abs(gg[key]-ww[key]) for key in gg.keys()&ww.keys()]+[0.]);ok=gg.keys()==ww.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,maximum_basic_volume_difference_mm3=err,matched=bool(ok)))
        if not ok:result['differences'].append(label)
        save();print(label,ok,flush=True)
    selected=[('initial',raw,raw['states'][0])]
    if refined['states']:
        selected.append(('refined_first',refined,refined['states'][0]))
        if len(refined['states'])>1:selected.append(('refined_last',refined,refined['states'][-1]))
    if continuous['states']:
        selected.append(('continuous_first',continuous,continuous['states'][0]))
        if len(continuous['states'])>1:selected.append(('continuous_last',continuous,continuous['states'][-1]))
    failed_saved=None;failed_source=None;failed_obs=None
    for label,report,state in selected:
        saved=n.c.load(n.p.read(ROOT/state['scene'])['parts']);groups=a.pose(ctx,hw,state['case'])
        for key,obs in groups:compare(label+'_'+key,n.c.material(saved,obs),state['native'][key])
        compare(label+'_self',k.flat([n.c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),state['native']['self'])
        indices=[3,10] if label=='initial' else [5]
        chosen=[z for z in saved if z['kind']=='housing' and z['path_segment'] in indices]
        if label=='initial':chosen += [z for z in saved if z['kind']=='cord' and z['path_segment'] in [0,1]]+[next(z for z in saved if z['name']=='compact_housing_0_5')]
        for row in chosen:
            source=material_shape(row,state,report);metric=measure.match(measure.metrics(source),measure.metrics(row['shape']))
            result['geometry_comparisons'].append(dict(id=label+'_'+row['name'],**metric))
            if not metric['matched']:result['differences'].append('metric_'+label+'_'+row['name'])
            if label=='initial' and row['name']=='compact_housing_0_5':failed_saved=row['shape'];failed_source=source
            save()
        if label=='initial':failed_obs=next(z['shape'] for _,rows in groups for z in rows if z['name']=='middle_dip_actuator_motor')
    # Plan fixed before this replay: one actual failed housing/motor common.
    if failed_source is not None:
        ss=measure.metrics(failed_source.intersect(failed_obs,tol=1e-6))['global_values'];tt=measure.metrics(failed_saved.intersect(failed_obs,tol=1e-6))['global_values']
        err=float(np.max(abs(np.array(ss)-tt)));item=dict(source_adaptive_values=ss,STEP_adaptive_values=tt,maximum_difference=err,matched=err<1e-5)
        result['planned_common_pairs'].append(item)
        if not item['matched']:result['differences'].append('planned_actual_block_common')
        save()
    for report in [raw,refined,continuous]:
        weights=[[np.array(z) for z in line] for line in report['arc_length_weights']];budgets=np.array(report['free_contour_budgets_mm']);x0=np.array(report['neutral_parameters'])
        recs={z['id']:z for z in report['increments']}
        for z in report['increments']:
            case=z['case'];P,H=n.a.feed.wrist_frames(case['wrist_flex_deg'],case['wrist_yaw_deg']);F=n.a.thumb_frame(np.array(case['q']))
            err=max(np.max(abs(P-case['P'])),np.max(abs(H-case['H'])),np.max(abs(F-case['F'])))
            residual=float(np.max(abs(n.endpoints(np.array(z['parameters']),case,weights,budgets))))
            ok=err<1e-10 and abs(residual-z['endpoint_scaled_residual'])<1e-8
            result['increment_kinematics'].append(dict(id=z['id'],FK_error=err,endpoint_scaled_residual=residual,matched=bool(ok)))
            if not ok:result['differences'].append('FK_'+z['id'])
        for entry in report['profiles']:
            x=np.array(recs[entry['id']]['parameters']);case=recs[entry['id']]['case']
            for want in entry['materials']:
                got=check.profile(x,case,x0,weights,budgets,want['line'],want['kind'])
                errors=[abs(got[key]-want[key]) for key in ['material_length_mm','actual_length_mm','maximum_join_gap_mm','guide_rear_material_s_mm','guide_rear_material_shift_mm']]
                errors += [float(np.max(abs(np.array(p['world_mm'])-q['world_mm']))) for p,q in zip(got['material_points'],want['material_points'])]
                errors += [float(np.max(abs(np.array(got['bore_material_witness']['world_mm'])-want['bore_material_witness']['world_mm'])))]
                ok=max(errors)<1e-5 and got['bore_material_witness']['inside_actual_whole_bore']==want['bore_material_witness']['inside_actual_whole_bore']
                result['profiles'].append(dict(id=entry['id'],kind=want['kind'],line=want['line'],maximum_difference=max(errors),matched=bool(ok)))
                if not ok:result['differences'].append('profile_'+entry['id']+'_'+want['kind']+str(want['line']))
                save()
    for entry in basis['required_movement_chord_bounds']:
        for want in entry['lines']:
            p=a.ports(entry['case'],want['line']);value=np.linalg.norm(p['up_B']-p['up_A'])+np.linalg.norm(p['down_B']-p['down_A']);err=abs(value-want['necessary_free_contour_lower_bound_mm']);ok=err<1e-5
            result['chord_bounds'].append(dict(id=entry['id'],line=want['line'],value_mm=float(value),matched=bool(ok)))
            if not ok:result['differences'].append('chord_'+entry['id']+str(want['line']))
    chosen=hwraw['states'][:1]+(hwraw['states'][-1:] if len(hwraw['states'])>1 else []);names={z['name'] for z in hw}
    for state in chosen:
        groups=a.pose(ctx,hw,state['case']);moving=[z for _,rows in groups for z in rows if z['name'] in names]
        for key,obs in groups:compare('hardware_'+state['id']+'_'+key,n.c.material(moving,[z for z in obs if z['name'] not in names]),state['native'][key])
        compare('hardware_'+state['id']+'_internal',k.flat([n.c.material([z],moving[j+1:]) for j,z in enumerate(moving)]),state['native']['internal'])
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),manifest_files=len(manifest['files']),frozen_files=len(frozen),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        original_projection_or_local_probe_or_continuation_reexecuted=False,rendering_reexecuted=False,independent_audit=False,
        initial_failed_candidate_admissible=False,physical_or_whole_hand_or_exterior_approved=False)
    save()


if __name__=='__main__':main()

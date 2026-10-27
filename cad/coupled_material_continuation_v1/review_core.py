"""Scoped saved-state replay from an exact, clean commit checkout."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import continuation as n
k=n.m.drive.module('_continue046_native',n.HERE/'check.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*args):return subprocess.check_output(['git',*args],cwd=n.ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(n.ROOT)
    manifest=n.p.read(n.ROOT/'checkpoints/CONT046.manifest.json');frozen={}
    for tag in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(n.ROOT/'AUDIT_EVIDENCE_INDEX.json')[tag])
    def guard():return all(n.m.drive.sha(n.ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(n.m.drive.sha(n.ROOT/f)==h for f,h in frozen.items())
    assert guard();analysis=n.p.read(n.HERE/'reviews/analysis.json');raw=n.p.read(n.HERE/'reviews/check.json');ev=n.p.read(n.HERE/'reviews/evidence.json');ctx=k.context()
    weights=[[n.np.array(z) for z in row] for row in analysis['arc_length_weights']];budgets=n.np.array(analysis['free_contour_budgets_mm'])
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],profiles=[],kinematic_steps=[],geometry_comparisons=[],adaptive_common_pairs=[])
    def save():n.a.save(args.output,result)
    for j,rec in enumerate(analysis['steps']):
        x=n.np.array(rec['parameters']);got=float(max(abs(n.endpoints(x,rec['case'],weights,budgets))))
        ok=abs(got-rec['endpoint_scaled_residual'])<1e-10
        result['kinematic_steps'].append(dict(step=j,maximum_endpoint_scaled_residual=got,matched=bool(ok)))
        if not ok:result['differences'].append('endpoints_'+str(j))
    selected=[raw['states'][j] for j in sorted({0,len(raw['states'])-1})] if raw['states'] else [];saved_by_step={}
    for rec in selected:
        rows=n.c.load(n.p.read(n.ROOT/rec['scene'])['parts']);groups=k.pose(ctx,rec['case']);saved_by_step[rec['step']]=rows
        tests={key:n.c.material(rows,obs) for key,obs in groups};tests['self']=k.flat([n.c.material([z],rows[j+1:]) for j,z in enumerate(rows)])
        for key,got in tests.items():
            want=rec['native'][key];aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']}
            error=max([abs(aa[q]-bb[q]) for q in aa.keys()&bb.keys()]+[0.])
            ok=aa.keys()==bb.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            label=str(rec['step'])+'_'+key;result['native_samples'].append(dict(id=label,native=got,maximum_basic_volume_difference_mm3=error,matched=bool(ok)))
            if not ok:result['differences'].append(label)
            print(label,ok,flush=True);save()
    for item in raw['profiles']:
        rec=analysis['steps'][item['step']];x=n.np.array(rec['parameters'])
        for want in item['materials']:
            got=k.profile(x,rec['case'],weights,budgets,want['kind'],want['line']);errors=[abs(got[key]-want[key]) for key in ['actual_length_mm','maximum_join_gap_mm','PAIR_rear_material_s_mm','PAIR_rear_material_coordinate_change_mm']]
            errors+=[float(max(abs(n.np.array(got['piece_lengths_mm'])-want['piece_lengths_mm'])))]
            errors+=[float(max(abs(n.np.array(a['world_mm'])-b['world_mm']))) for a,b in zip(got['material_points'],want['material_points'])]
            errors+=[float(max(abs(n.np.array(got['axis_sliding_witness']['world_mm'])-want['axis_sliding_witness']['world_mm'])))]
            ok=max(errors)<1e-5;result['profiles'].append(dict(step=item['step'],kind=want['kind'],line=want['line'],maximum_difference=max(errors),matched=bool(ok)))
            if not ok:result['differences'].append('profile_'+str(item['step'])+'_'+want['kind']+'_'+str(want['line']))
            save()
    if selected:
        rec=selected[-1];ar=analysis['steps'][rec['step']];fresh={z['name']:z for z in k.build(n.np.array(ar['parameters']),ar['case'],weights,budgets)};saved={z['name']:z for z in saved_by_step[rec['step']]}
        # Two free circular sections in each zone/line, housing only, chosen
        # before replay. This is a measurement signature, not trim equivalence.
        names=['continued_housing_'+str(i)+'_'+str(j) for i in [0,1] for j in [3,3+n.NUP//2,3+n.NUP+4,3+n.NUP+4+n.NDOWN//2]]
        for name in names:
            got=dict(name=name,**n.s.e.measure.match(n.s.e.measure.metrics(fresh[name]['shape']),n.s.e.measure.metrics(saved[name]['shape'])));result['geometry_comparisons'].append(got)
            if not got['matched']:result['differences'].append('metric_'+name)
            save()
        for pairrec in ev['new_common_pairs']:
            pair=pairrec['pair'];lookup={z['name']:z for z in [v for _,zz in k.pose(ctx,rec['case']) for v in zz]+saved_by_step[rec['step']]}
            sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6);values=n.s.e.measure.metrics(sh)['global_values'];err=float(max(abs(n.np.array(values)-pairrec['adaptive_common_values'])));ok=err<1e-5
            result['adaptive_common_pairs'].append(dict(pair=pair,STEP_common_values=values,source_common_values=pairrec['adaptive_common_values'],maximum_difference=err,matched=bool(ok)))
            if not ok:result['differences'].append('common_'+pair['lead'])
            save()
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),manifest_files=len(manifest['files']),frozen_files=len(frozen),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        solver_reexecuted=False,independent_audit=False,physical_or_whole_parent_wrist_approval=False,inherited_SLIDE045_core_false_preserved=True)
    save()


if __name__=='__main__':main()

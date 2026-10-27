"""Replay both declared feed candidates from an exact fresh clean checkout."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import evidence_render as e
n,first,t,w,l,a,m,g,c,p,np,ROOT,HERE=e.n,e.first,e.t,e.w,e.l,e.a,e.m,e.g,e.c,e.p,e.np,e.ROOT,e.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=p.read(ROOT/'checkpoints/CORD043.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');alt=p.read(HERE/'reviews/anchor_check.json');proof=p.read(HERE/'reviews/evidence.json');case,data,groups,housing=e.inputs()
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],material_checks=[],adaptive_common_pairs=[],motor_join_checks=[])
    def save():a.save(args.output,result)
    def compare(label,got,want):
        aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']}
        error=max([abs(aa[q]-bb[q]) for q in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),max_basic_numeric_difference=error,overlap_locations_match=aa.keys()==bb.keys()))
        if not ok:result['differences'].append(label)
        print(label,ok,flush=True);save()
    saved={};full=[z for _,zz in groups for z in zz]+housing
    def earlier(record):
        if not record['retained_cord_names']:return []
        rec=next(z for z in reversed(alt['cases']) if z['line']==0);rows=c.load(p.read(ROOT/rec['scene'])['parts'])
        assert [z['name'] for z in rows]==record['retained_cord_names'];return rows
    for mode,report in [('original',raw),('anchor_first',alt)]:
        selected={}
        if mode=='original':selected={z['id']:z for z in report['cases'] if 'scene' in z}
        else:
            for i in [0,1]:
                records=[z for z in report['cases'] if z['line']==i]
                if records:
                    for z in [records[0],records[min(1,len(records)-1)],records[-1]]:selected[z['id']]=z
        for rec in report['cases']:
            if mode=='original':_,got=first.n.build(case,rec['line'],rec['tip_coordinate_mm'],rec['tail_fraction'])
            else:_,_,got=n.pieces(case,rec['line'],rec['tip_coordinate_mm'])
            error=max(abs(got['actual_length_mm']-rec['actual_length_mm']),abs(got['material_length_mm']-rec['material_length_mm']))
            for x,y in zip(got['material_points'],rec['material_points']):error=max(error,float(np.max(abs(np.asarray(x['world_mm'])-y['world_mm']))))
            ok=error<1e-5;result['material_checks'].append(dict(mode=mode,id=rec['id'],maximum_difference=error,matched=bool(ok)))
            if not ok:result['differences'].append(mode+'_material_'+rec['id'])
            if rec['id'] not in selected:continue
            rows=c.load(p.read(ROOT/rec['scene'])['parts']);saved[(mode,rec['id'])]=rows
            for key,zz in groups:compare(mode+'_'+rec['id']+'_'+key,c.material(rows,zz),rec['native'][key])
            compare(mode+'_'+rec['id']+'_whole_housings',c.material(rows,housing),rec['native']['whole_housings'])
            compare(mode+'_'+rec['id']+'_earlier_cord',c.material(rows,earlier(rec)),rec['native']['earlier_cord'])
            compare(mode+'_'+rec['id']+'_self',first.aggregate([c.material([z],rows[j+1:]) for j,z in enumerate(rows)]),rec['native']['self'])
    metric_choices=[];oldrec=raw['cases'][-1];metric_choices.append(('original',oldrec,saved[('original',oldrec['id'])][0]))
    for i in [0,1]:
        records=[z for z in alt['cases'] if z['line']==i]
        if not records:continue
        rec=records[-1];rows=saved[('anchor_first',rec['id'])];path,_,_=n.specification(case,i)
        choices=[z for z in rows if z['path_segment'] in [0,3,len(path)-2,len(path)-1]]
        if not choices:choices=rows[:1]
        metric_choices += [('anchor_first',rec,z) for z in choices]
    source_cache={}
    for mode,rec,z in metric_choices:
        key=(mode,rec['id'])
        if key not in source_cache:source_cache[key]={q['name']:q for q in e.moving(case,rec,mode=='anchor_first')}
        source=source_cache[key];q=dict(mode=mode,id=rec['id'],name=z['name'],**e.measure.match(e.measure.metrics(source[z['name']]['shape']),e.measure.metrics(z['shape'])))
        result['geometry_comparisons'].append(q)
        if not q['matched']:result['differences'].append(mode+'_metrics_'+z['name'])
        save()
    for want in proof['motor_join_geometry']:
        path,_,_=n.specification(case,want['line']);angle=float(np.degrees(np.arccos(np.clip(np.dot(np.array(path[0].tangentAt(1).toTuple()),np.array(path[1].tangentAt(0).toTuple())),-1,1))))
        err=max(abs(path[0].Length()-want['anchor_bore_model_length_mm']),abs(path[2].Length()-want['motor_free_tangent_mm']),abs(angle-want['tangent_kink_deg']))
        ok=err<1e-5 and abs(path[2].Length()-18)<1e-5;result['motor_join_checks'].append(dict(line=want['line'],maximum_difference=err,matched=bool(ok)))
        if not ok:result['differences'].append('motor_join_'+str(want['line']))
    initial=not result['differences'] and guard() and not git('status','--porcelain=v1')
    for want in proof['pairs']:
        mode=want['mode'];report=raw if mode=='original' else alt;rec=next(z for z in report['cases'] if z['id']==want['state']);rows=saved[(mode,rec['id'])]
        lookup={z['name']:z for z in full+earlier(rec)+rows};pair=want['pair'];common=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6)
        values=e.measure.metrics(common)['global_values'];err=float(np.max(abs(np.asarray(values)-want['adaptive_source_common_values'])));ok=err<1e-5
        result['adaptive_common_pairs'].append(dict(mode=mode,state=rec['id'],pair=pair,STEP_STEP_values=values,source_source_values=want['adaptive_source_common_values'],maximum_difference=err,matched=bool(ok)))
        save()
    precision=all(z['matched'] for z in result['adaptive_common_pairs'])
    result.update(completed=True,initial_core_passed=bool(initial),precision_pairs_passed=bool(precision),passed=bool(initial and precision),manifest_files=len(manifest['files']),frozen_files=len(frozen),
                  clone_clean=not git('status','--porcelain=v1'),input_hash_guard=guard(),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),same_agent_not_independent_audit=True,
                  original_candidate_failed=True,alternative_raw_native_clear=alt['raw_native_all_clear'],alternative_sequence_other_overlaps_clear=alt['bounded_sequence_clear_of_other_overlaps'],physical_installation_or_fixation_approved=False)
    save()


if __name__=='__main__':main()

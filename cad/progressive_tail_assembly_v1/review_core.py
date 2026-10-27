"""Exact commit, fresh clean clone replay of a bounded new assembly candidate."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import evidence_render as e
k,n,s,t,a,m,g,c,p,np,ROOT,HERE=e.k,e.n,e.s,e.t,e.a,e.m,e.g,e.c,e.p,e.np,e.ROOT,e.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=p.read(ROOT/'checkpoints/FORM042.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');proof=p.read(HERE/'reviews/evidence.json');case,data,full,groups,hw=e.inputs()
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],material_checks=[],geometry_comparisons=[],adaptive_common_pairs=[],boundaries=[])
    def save():a.save(args.output,result)
    def compare(label,got,want):
        aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']}
        error=max([abs(aa[q]-bb[q]) for q in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),max_basic_numeric_difference=error))
        if not ok:result['differences'].append(label)
        print(label,ok,flush=True);save()
    selected=[]
    for i in [0,1]:
        records=[z for z in raw['cases'] if z['active_line']==i]
        if records:selected += [records[0],records[-1]] if len(records)>1 else records
    unique=[]
    for record in selected:
        if not any(np.max(abs(np.asarray(record['formed_mm'])-v['formed_mm']))<1e-7 for v in unique):unique.append(record)
    selected={z['id']:z for z in unique};saved={}
    for record in raw['cases']:
        for i,want in enumerate(record['lines']):
            _,T,got=n.edges(case,i,record['formed_mm'][i]);err=max(abs(got['actual_length_mm']-want['actual_length_mm']),float(np.max(abs(T-np.asarray(want['guide_transform'])))),abs(got['PAIR_rear_material_s_mm']-want['PAIR_rear_material_s_mm']))
            for x,y in zip(got['material_points'],want['material_points']):err=max(err,float(np.max(abs(np.asarray(x['world_mm'])-y['world_mm']))))
            ok=err<1e-5;result['material_checks'].append(dict(id=record['id'],line=i,maximum_difference=err,matched=bool(ok)))
            if not ok:result['differences'].append('material_'+record['id']+'_'+str(i))
        if record['id'] not in selected:continue
        rows=c.load(p.read(ROOT/record['scene'])['parts']);saved[record['id']]=rows
        for key,zz in groups:compare(record['id']+'_'+key,c.material(rows,zz),record['native'][key])
        compare(record['id']+'_internal',k.old.k.combine([c.material([z],rows[q+1:]) for q,z in enumerate(rows)]),record['native']['internal'])
    for i in [0,1]:
        total=case['lines'][i]['up']['length_mm'];_,T0,z0=n.edges(case,i,0.);_,T1,z1=n.edges(case,i,total);_,oldT,oldz=s.edges(case,i,'feed',s.definitions(case,i)[4])
        err=max(float(np.max(abs(T0-oldT))),float(np.max(abs(T1-np.eye(4)))),max(float(np.max(abs(np.asarray(x['world_mm'])-y['world_mm']))) for x,y in zip(z0['material_points'],oldz['material_points'])))
        ok=err<1e-5;result['boundaries'].append(dict(line=i,maximum_difference=err,matched=bool(ok),target_is_1D_only=True))
        if not ok:result['differences'].append('boundary_'+str(i))
    last=raw['cases'][-1];source={z['name']:z for z in e.fresh(case,last,hw)};rows=saved[last['id']]
    restore=p.read(HERE/'reviews/restoration.json') if (HERE/'reviews/restoration.json').exists() else None
    final_rows=None
    if restore:
        fullmap={z['name']:z for _,zz in full for z in zz};savedmap={z['name']:z for z in rows}
        housing=[z for z in rows if z.get('kind')=='housing']
        for path in restore['paths']:
            installed=[savedmap[v] if v in savedmap else fullmap[v] for v in path['installed_before']]
            for j,want in enumerate(path['samples']):
                if 'scene' not in want:continue
                moving=c.load(p.read(ROOT/want['scene'])['parts'])
                got=c.material(moving,[z for _,zz in groups for z in zz]+housing+installed)
                compare('restore_'+path['moving']+'_'+str(j),got,k.old.k.combine(list(want['native'].values())))
        if restore['final']:
            final_rows=c.load(p.read(ROOT/restore['final']['scene'])['parts'])
            for key,zz in full:compare('final_'+key,c.material(final_rows,zz),restore['final']['native'][key])
            compare('final_internal',k.old.k.combine([c.material([z],final_rows[j+1:]) for j,z in enumerate(final_rows)]),restore['final']['native']['internal'])
            result['final_packaging_names_match']=restore['final']['restored_packaging_names']==sorted(z['name'] for z in dict(full)['packaging'])
            if not result['final_packaging_names_match']:result['differences'].append('final_packaging_names')
    for i in [0,1]:
        mid=next((z for z in raw['cases'] if z['active_line']==i and z['fraction']==.5),last)
        sample_rows=c.load(p.read(ROOT/mid['scene'])['parts']);sample_source={z['name']:z for z in e.fresh(case,mid,hw)}
        choices=[z for z in sample_rows if z['name']=='motor_'+str(i)+'_jaw_with_fixed_bend']
        candidates=[z for z in sample_rows if z.get('kind')=='housing' and z['physical_line']==i]
        es,_,_=n.edges(case,i,mid['formed_mm'][i]);free=[z for z in candidates if z['segment']>=3 and z['segment']<len(es)-len(s.definitions(case,i)[0])]
        choices+=free[:2]
        for z in choices:
            q=dict(id=mid['id'],name=z['name'],**e.measure.match(e.measure.metrics(sample_source[z['name']]['shape']),e.measure.metrics(z['shape'])));result['geometry_comparisons'].append(q)
            if not q['matched']:result['differences'].append('metrics_'+mid['id']+'_'+z['name'])
            save()
    initial=not result['differences'] and guard() and not git('status','--porcelain=v1')
    lookup={z['name']:z for _,zz in full for z in zz};lookup.update({z['name']:z for z in rows})
    if restore and proof['failed_state']:
        failed=[sample for path in restore['paths'] for sample in path['samples'] if not sample['clear']]
        if failed:lookup.update({z['name']:z for z in c.load(p.read(ROOT/failed[0]['scene'])['parts'])})
        if proof['failed_state']=='final':lookup.update({z['name']:z for z in final_rows})
    for want in proof['pairs']:
        pair=want['pair'];sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6);got=e.measure.metrics(sh)['global_values'];err=float(np.max(abs(np.asarray(got)-want['adaptive_source_common_values'])));ok=err<1e-5
        result['adaptive_common_pairs'].append(dict(pair=pair,STEP_STEP_values=got,source_source_values=want['adaptive_source_common_values'],maximum_difference=err,matched=bool(ok)))
        save()
    precision=all(z['matched'] for z in result['adaptive_common_pairs'])
    result.update(completed=True,initial_core_passed=bool(initial),adaptive_common_pairs_passed=bool(precision),passed=bool(initial and precision),
                  manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),clone_clean=not git('status','--porcelain=v1'),
                  OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),candidate_clear=restore['conditional_CAD_sequence_clear'] if restore else raw['forming_sequence_clear'],physical_installation_approved=False,same_agent_not_independent_audit=True)
    save()


if __name__=='__main__':main()

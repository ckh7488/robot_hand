"""Exact-SHA scoped replay of saved full-material assembly states."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_thread as k
import feed_envelopes as e
t,a,m,c,p,ROOT,HERE=k.t,k.a,k.m,k.c,k.p,k.ROOT,k.HERE
measure=t.prior.measure
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=p.read(ROOT/'checkpoints/THREAD040.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');case=t.neutral();data,groups=t.prior.inputs(case);result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],material_checks=[])
    def save():a.save(args.output,result)
    def compare(label,got,want):
        aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']};err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];result['native_samples'].append(dict(id=label,native=got,max_basic_numeric_difference=err,matched=bool(ok)))
        if not ok:result['differences'].append(label)
        print(label,ok,flush=True);save()
    saved_lookup={};retained=[]
    for plan in raw['plans']:
        records=[z for z in raw['cases'] if z['kind']==plan['kind'] and z['line']==plan['line']]
        if not records:break
        for record in records:
            edges,length=t.specification(case,record['line'],record['kind']);pieces=t.window_edges(edges,record['tip_coordinate_mm'],length);err=abs(sum(z.Length() for z in pieces)-record['actual_centerline_length_mm'])
            for point in record['material_points']:err=max(err,float(t.np.max(abs(t.np.asarray(t.point_at(edges,record['tip_coordinate_mm']-length+point['material_s_mm']))-point['world_mm']))))
            ok=err<1e-5;result['material_checks'].append(dict(id=record['id'],maximum_difference_mm=err,matched=bool(ok)))
            if not ok:result['differences'].append('material_'+record['id'])
            if 'scene' not in record:continue
            saved=c.load(p.read(ROOT/record['scene'])['parts']);saved_lookup[record['id']]=saved
            assert record['retained_material_names']==[z['name'] for z in retained]
            for key,zz in groups:compare(record['id']+'_'+key,c.material(saved,zz),record['native'][key])
            compare(record['id']+'_earlier_material',c.material(saved,retained),record['native']['earlier_material'])
            compare(record['id']+'_self',k.aggregate([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),record['native']['self'])
        last=records[-1]
        if last['clear'] and abs(last['tip_coordinate_mm']-plan['tips_mm'][-1])<1e-7:retained+=saved_lookup[last['id']]
        if not last['clear']:break
    for plan in raw['plans']:
        records=[z for z in raw['cases'] if z['kind']==plan['kind'] and z['line']==plan['line'] and 'scene' in z]
        if not records:continue
        record=records[-1];saved=saved_lookup[record['id']];fresh,_=t.state(case,record['line'],record['kind'],record['tip_coordinate_mm']);lookup={z['name']:z for z in fresh}
        for z in [saved[0],saved[-1]] if len(saved)>1 else saved:
            test=dict(id=record['id'],name=z['name'],**measure.match(measure.metrics(lookup[z['name']]['shape']),measure.metrics(z['shape'])));result['geometry_comparisons'].append(test)
            if not test['matched']:result['differences'].append('metrics_'+record['id']+'_'+z['name'])
            save()
    occupancy=p.read(HERE/'reviews/envelopes.json');retained=[]
    for record in occupancy['cases']:
        key='occupancy_'+record['kind']+'_'+str(record['line']);saved=c.load(p.read(ROOT/record['scene'])['parts']);assert record['retained_material_names']==[z['name'] for z in retained]
        for group,zz in groups:compare(key+'_'+group,c.material(saved,zz),record['native'][group])
        compare(key+'_earlier_material',c.material(saved,retained),record['native']['earlier_material'])
        compare(key+'_self',k.aggregate([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),record['native']['self'])
        fresh,info=e.build(case,record['line'],record['kind']);lookup={z['name']:z for z in fresh};assert abs(info['material_length_mm']-record['material_length_mm'])<1e-5 and info['occupancy_coordinate_interval_mm']==record['occupancy_coordinate_interval_mm']
        for z in [saved[0],saved[-1]]:
            test=dict(id=key,name=z['name'],**measure.match(measure.metrics(lookup[z['name']]['shape']),measure.metrics(z['shape'])));result['geometry_comparisons'].append(test)
            if not test['matched']:result['differences'].append('metrics_'+key+'_'+z['name'])
        last=next(z for z in reversed(raw['cases']) if z['kind']==record['kind'] and z['line']==record['line']);retained+=saved_lookup[last['id']];save()
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,actual_threading_motor_installation_material_motion_or_force_approved=False);save()
if __name__=='__main__':main()

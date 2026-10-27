"""Exact-SHA bounded replay of the new grounded motor guides, including failures."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check as chk
b,a,m,g,c,p,np,cq,ROOT,HERE=chk.b,chk.a,chk.m,chk.g,chk.c,chk.p,chk.np,chk.cq,chk.ROOT,chk.HERE
survey=m.drive.module('_support038_survey_review',HERE/'survey.py')
measure=m.drive.module('_support038_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=p.read(ROOT/'checkpoints/SUPPORT038.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');oldsurvey=p.read(HERE/'reviews/survey.json');analysis=p.read(ROOT/'cad/material_length_motion_v1/reviews/analysis.json');data,pack,guide,wrist=a.s.prior.inputs();fresh,design=b.build(pack);saved=c.load(p.read(ROOT/raw['scene'])['parts']);removed=[z['replaced'] for z in design];pack_old=[z for z in pack if z['name'] not in removed];hw=[z for z in saved if z['installed_part_candidate']];result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],survey_distances=[],entry_boundary_kinematics=[],old_jaw_material_preserved=[z['removed_old_material_mm3'] for z in design])
    def save():a.save(args.output,result)
    def compare(label,got,want):
        aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']};err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];result['native_samples'].append(dict(id=label,native=got,max_basic_numeric_difference=err,matched=bool(ok)))
        if not ok:result['differences'].append(label)
        print(label,ok,flush=True);save()
    selected=[raw['static'][0]]+([raw['static'][-1]] if len(raw['static'])>1 else [])
    for case in selected:
        for key,zz in chk.kobs(data,pack_old,guide,wrist,analysis['cases'][case['case_id']]):compare('static_'+str(case['case_id'])+'_'+key,c.material(saved,zz),case['native'][key])
        compare('static_'+str(case['case_id'])+'_internal',chk.flat([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),case['internal'])
    allobs=[z for _,zz in chk.kobs(data,pack_old,guide,wrist,analysis['cases'][0]) for z in zz]+hw;lookup={z['name']:z for z in allobs}
    for station in raw['assembly']:
        for path in station['paths']:
            moving=lookup[path['moving']];ob=[lookup[n] for n in path['obstacle_names']]
            for index in [0,len(path['samples'])-1]:
                sample=path['samples'][index];delta=tuple(sample['translation_world_mm']);compare('assembly_'+path['moving']+'_'+str(index),c.material([dict(moving,shape=moving['shape'].translate(delta))],ob),sample['native'])
    for station in raw['feed']:
        i=station['line'];other=[z for z in saved if not z['installed_part_candidate'] and z['physical_line']!=i]
        for index in [0,len(station['samples'])-1]:
            z=station['samples'][index];compare('feed_'+str(i)+'_'+str(index),c.material([b.feed(i,z['tip_mm'])],allobs+other),z['native'])
    for left,right in zip(fresh,saved):
        assert left['name']==right['name'];test=dict(name=left['name'],**measure.match(measure.metrics(left['shape']),measure.metrics(right['shape'])));result['geometry_comparisons'].append(test)
        if not test['matched']:result['differences'].append('metric_'+left['name'])
        save()
    common=c.load(p.read(ROOT/'cad/material_length_motion_v1/generated/actual_common/scene.json')['parts'][:1])[0];pack_lookup={z['name']:z for z in pack}
    # Replay the complete bbox certificate and only the original branch-bound native set.
    tested=[];best=float('inf');order=sorted(pack,key=lambda z:survey.bbox_distance(z['shape'],common['shape']))
    for z in order:
        if survey.bbox_distance(z['shape'],common['shape'])>best:break
        d=survey.distance(common['shape'],z['shape']);best=min(best,d['distance_mm']);tested.append(dict(name=z['name'],**d))
    want={z['name']:z for z in oldsurvey['nearest_packaging_tested']};assert set(want)=={z['name'] for z in tested}
    for z in tested:
        err=abs(z['distance_mm']-want[z['name']]['distance_mm']);result['survey_distances'].append(dict(**z,matched=err<1e-5))
        if err>=1e-5:result['differences'].append('survey_'+z['name'])
    result['all_385_packaging_bbox_bounds_rebuilt']=len(pack)==385;result['nearest_packaging_mm']=best;save()
    Q=a.a.frame();axis=np.array([-1.,0.,0.])
    for want in oldsurvey['upper_entry_boundary_kinematics']:
        case=analysis['cases'][want['id']];F=a.thumb_frame(case['angles_deg']);T=F@Q;C=np.asarray(case['solution']['controls'])[3];v=a.s.SPL.derivative()(1.)@C;acc=a.s.SPL.derivative(2)(1.)@C;normal=acc-v*(v@acc)/(v@v);speed=np.linalg.norm(v);rotation=np.degrees(np.arccos(np.clip((T[:3,:3]@axis)@(Q[:3,:3]@axis),-1,1)));vals=[rotation,speed,np.linalg.norm(normal),speed**2/np.linalg.norm(normal)];expected=[want[k] for k in ['entry_tangent_rotation_from_neutral_deg','stored_downstream_end_speed_mm','stored_end_normal_acceleration_mm','stored_end_radius_mm']];err=max(abs(np.asarray(vals)-expected));ok=err<1e-5 and np.max(abs(T-want['actual_entry_frame']))<1e-5;result['entry_boundary_kinematics'].append(dict(id=want['id'],values=vals,matched=bool(ok)))
        if not ok:result['differences'].append('entry_'+str(want['id']))
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,old_collision_groups_or_optimizer_repeated=0,whole_flexible_link_or_hand_approved=False);save()
if __name__=='__main__':main()

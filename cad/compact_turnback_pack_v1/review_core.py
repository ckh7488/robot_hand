"""Exact-SHA first-gate replay of this separate rigid packaging candidate."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import pack_model as a
import service_pack as service
n,k,np,ROOT,HERE=a.n,a.k,a.np,a.ROOT,a.HERE
measure=n.m.drive.module('_pack049_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*values):return subprocess.check_output(['git',*values],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/PACK049.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=n.p.read(HERE/'reviews/check.json');fresh,design,_=a.build()
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts']);ctx=k.context();removed=set(raw['replaced_hardware'])
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],port_comparisons=[])
    def save():n.a.save(args.output,result)
    for left,right in zip(fresh,saved):
        assert left['name']==right['name'];item=dict(name=left['name'],**measure.match(measure.metrics(left['shape']),measure.metrics(right['shape'])))
        result['geometry_comparisons'].append(item)
        if not item['matched']:result['differences'].append('geometry_'+left['name'])
        save()
    for got,want in zip(design,raw['design']):
        fields=['free_port_world_mm','free_tangent_toward_hand']+(['shoulder_world_mm','shoulder_to_free_port_length_mm'] if got['id'].startswith('motor') else ['front_port_world_mm','front_tangent_toward_hand','centerline_length_mm'])
        err=max(float(np.max(abs(np.array(got[key])-want[key]))) for key in fields);ok=err<1e-5
        result['port_comparisons'].append(dict(id=got['id'],maximum_difference=err,matched=ok))
        if not ok:result['differences'].append('port_'+got['id'])
    # Only first and final actually executed states, never unexecuted planned poses.
    states=[raw['states'][0]]+([raw['states'][-1]] if len(raw['states'])>1 else [])
    def compare(label,got,want):
        ga={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']}
        wa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']}
        err=max([abs(ga[key]-wa[key]) for key in ga.keys()&wa.keys()]+[0.])
        ok=ga.keys()==wa.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,maximum_basic_volume_difference_mm3=err,matched=bool(ok)))
        if not ok:result['differences'].append(label)
        print(label,ok,flush=True);save()
    for state in states:
        groups=[(key,[z for z in obs if z['name'] not in removed]) for key,obs in k.pose(ctx,state['case'])]
        moving=a.posed(saved,state['case'])
        for key,obs in groups:
            assert [z['name'] for z in obs]==state['tested_obstacle_names'][key]
            compare(state['id']+'_'+key,n.c.material(moving,obs),state['native'][key])
        compare(state['id']+'_internal',k.flat([n.c.material([z],moving[j+1:]) for j,z in enumerate(moving)]),state['native']['internal'])
    sr=n.p.read(HERE/'reviews/service.json');assert sr['completed']
    neutral_groups=[(key,[z for z in obs if z['name'] not in removed]) for key,obs in k.pose(ctx,raw['states'][0]['case'])]
    allrows=[z for _,obs in neutral_groups for z in obs]+saved;lookup={z['name']:z for z in allrows}
    for station in sr['stations']:
        for path in station['paths']:
            row=lookup[path['moving']];obstacles=[lookup[name] for name in path['obstacle_names']]
            indices=sorted({0,len(path['samples'])-1}|{j for j,sample in enumerate(path['samples']) if not n.m.drive.clear(sample['native'])})
            for index in indices:
                sample=path['samples'][index];moving=dict(row,shape=row['shape'].translate(tuple(sample['translation_world_mm'])))
                compare('assembly_'+station['id']+'_'+path['moving']+'_'+str(index),n.c.material([moving],obstacles),sample['native'])
    if sr['local_bore_fit']:
        fit=sr['local_bore_fit'];local=n.c.load(n.p.read(ROOT/fit['scene'])['parts']);original=service.local_housings()
        for left,right in zip(original,local):
            assert left['name']==right['name'];item=dict(name=left['name'],**measure.match(measure.metrics(left['shape']),measure.metrics(right['shape'])))
            result['geometry_comparisons'].append(item)
            if not item['matched']:result['differences'].append('geometry_'+left['name'])
            save()
        for key,obstacles in neutral_groups:compare('local_fit_'+key,n.c.material(local,obstacles),fit['native'][key])
        compare('local_fit_new_guides',n.c.material(local,saved),fit['native']['new_guides'])
        compare('local_fit_internal',k.flat([n.c.material([z],local[j+1:]) for j,z in enumerate(local)]),fit['native']['internal'])
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),source_geometry_rebuilt=len(result['geometry_comparisons']),old_local_checks_reexecuted=0,
        assembly_scope='Only first/last stored samples of each executed conditional path; not tools, force or continuous assembly.',
        whole_material_length_free_path_or_motion_verified=False,physical_or_exterior_approved=False,independent_audit=False)
    save()


if __name__=='__main__':main()

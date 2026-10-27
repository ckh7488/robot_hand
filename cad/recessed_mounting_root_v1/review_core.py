"""Bounded exact-SHA replay of first failures and corrected carrier relations."""
import argparse,importlib.util,json,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm063_review',Path(__file__).with_name('selected_model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
metrics=n.m.drive.module('_arm063_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
from scipy.optimize import linear_sum_assignment


def geometry_match(x,y):
    if len(x.Solids())==len(y.Solids())==1:return metrics.match(metrics.metrics(x),metrics.metrics(y))
    left,right=metrics.metrics(x),metrics.metrics(y);bad=[]
    if not left['valid'] or not right['valid'] or left['solids']!=right['solids']:bad.append('valid_or_solid_counts')
    maximum=float(max(abs(np.array(left['global_values'])-right['global_values'])))
    for key in ['faces','edges','vertices']:
        for kind in sorted({z['kind'] for z in left[key]+right[key]}):
            l=[z['values'] for z in left[key] if z['kind']==kind];r=[z['values'] for z in right[key] if z['kind']==kind]
            if len(l)!=len(r):bad.append(key+'_count');continue
            cost=np.array([[float(max(abs(np.array(i)-j))) for j in r] for i in l]);ii,jj=linear_sum_assignment(cost)
            maximum=max(maximum,float(max(cost[ii,jj])))
    if maximum>1e-5:bad.append('absolute_metric_limit')
    return dict(matched=not bad,differences=bad,max_numeric_difference=maximum,source_solids=left['solids'],STEP_solids=right['solids'])


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM063.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==v for f,v in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    first=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json');ev=n.p.read(HERE/'reviews/evidence.json')
    before,membership,source=a.context();first_source,entries,first_design=a.base.build(before);fresh,design=a.build(before)
    initial=n.c.load(n.p.read(ROOT/first['scene'])['parts']);new=n.c.load(n.p.read(ROOT/selected['scene'])['parts'])[0]
    entry=n.c.load(n.p.read(ROOT/first['entry_scene'])['parts']);other=[z for z in initial if z['name']!=a.CARRIER]
    fixed=[z for z in before if z['name'] not in a.OLD];bounds=n.c.load(n.p.read(ROOT/first['protection_input_scene'])['parts'])
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],front_pad_samples=[],
        manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    def geom(left,right):
        assert len(left)==len(right)
        for x,y in zip(left,right):
            assert x['name']==y['name'];got=dict(name=x['name'],**geometry_match(x['shape'],y['shape']))
            result['geometry_comparisons'].append(got)
            if not got['matched']:result['differences'].append('geometry_'+x['name'])
            save()
    geom(first_source,initial);geom(entries,entry);geom([fresh],[new])
    def compare(label,got,want):
        l,r=d.overlap_map(got),d.overlap_map(want)
        error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),max_volume_difference_mm3=error))
        if not ok:result['differences'].append(label)
        save();print(label,bool(ok),flush=True)
    compare('first_neutral6__retained865',n.c.material(initial,fixed),first['native']['new_whole_unit6__retained865_neutral'])
    compare('first_internal15',d.k.flat([n.c.material([z],initial[i+1:]) for i,z in enumerate(initial[:-1])]),first['native']['new_whole_unit6_internal15'])
    for label,l,r in [('selected_carrier__retained865',[new],fixed),('selected_carrier__unchanged_new_cap_fasteners5',[new],other),
        ('existing_rotation_bounds9__selected_carrier',bounds[:9],[new]),('existing_nut_approach__selected_carrier',[bounds[-1]],[new]),
        ('unchanged_new_entry4__selected_carrier',entry,[new])]:compare(label,n.c.material(l,r),selected['native'][label])
    transform_error=0.;arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in selected['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        transform_error=max(transform_error,max(float(np.max(abs(F[k]-state['transforms'][k]))) for k in F))
        moving=dict(new,shape=n.g.original.transform(new['shape'],F['forearm']))
        obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')]))
            for z in fixed if membership.get(z['name'],'forearm')!='forearm']
        label=state['id']+'__selected_carrier_relative_bodies';compare(label,n.c.material([moving],obstacles),selected['native'][label])
    for pad in a.front_pad_inputs(before):
        q=pad['shape'].cut(new['shape'],tol=1e-6);volume=q.Volume() if q.Solids() else 0.
        want=next(z for z in selected['front_pad_queries'] if z['id']==pad['id'])
        ok=q.isValid() and len(q.Solids())==want['outside_solids'] and abs(volume-want['outside_mm3'])<1e-5
        result['front_pad_samples'].append(dict(id=pad['id'],outside_mm3=volume,solids=len(q.Solids()),matched=bool(ok)))
        if not ok:result['differences'].append(pad['id'])
    lookup={z['name']:z for z in before};initial_lookup={z['name']:z for z in initial};common=[];queries=[]
    carrier=initial_lookup[a.CARRIER]['shape']
    for label,right in [('first_rotor',lookup[ev['initial_rotor_record']['part']]['shape']),('first_cap',initial_lookup[a.CAP]['shape'])]:
        q=carrier.intersect(right,tol=1e-6);want=next(z for z in ev['first_common_queries'] if z['id']==label)
        ok=q.isValid() and len(q.Solids())==want['solids'] and abs(q.Volume()-want['volume_mm3'])<1e-5
        queries.append(dict(id=label,volume_mm3=q.Volume(),solids=len(q.Solids()),matched=bool(ok)))
        if not ok:result['differences'].append(label)
        common.append(dict(name='arm063_'+label+'_common',shape=q))
    result['first_common_samples']=queries;geom(common,n.c.load(n.p.read(ROOT/ev['first_common_scene'])['parts']))
    beam=lookup['Forearm_box_beam']['shape'];distance=new['shape'].distance(beam)
    result['carrier_beam_distance_mm']=distance
    distances=[dict(name=z['name'],distance_mm=new['shape'].distance(z['shape'])) for z in other[1:]]
    result['carrier_fastener_distances_mm']=distances
    if abs(distance-selected['new_carrier_beam_distance_mm'])>=1e-5 or any(x['name']!=y['name'] or abs(x['distance_mm']-y['distance_mm'])>=1e-5 for x,y in zip(distances,selected['new_carrier_fastener_distances_mm'])):result['differences'].append('selected_distances')
    plain=lambda v:json.loads(json.dumps(v,default=lambda q:q.tolist() if hasattr(q,'tolist') else q))
    result['initial_design_contract_matched']=plain(first_design)==first['design']
    result['selected_design_contract_matched']=plain(design)==selected['design'] and transform_error<1e-10
    result['unchanged5_and_entry4_source_hashes_matched']=all(n.m.drive.sha(ROOT/f)==h for f,h in selected['source_sha256'].items())
    for k in ['initial_design_contract_matched','selected_design_contract_matched','unchanged5_and_entry4_source_hashes_matched']:
        if not result[k]:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        front_pad_matches=sum(z['matched'] for z in result['front_pad_samples']),first_common_matches=sum(z['matched'] for z in queries),
        arm_transform_max_error=transform_error,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='First2groups preserve initial6rotor/cap obstruction evidence;selected17 changed-carrier groups;13sourceSTEP signatures(initial6+entry4+selected1+common2,actual multi-solid counts retained);16changed frontpad preservation queries and16input commons;2focused initialcommon queries;5selecteddistances;design/body/4explicitfastener transforms. Other16initial groups not replayed;no oldARM062/ARM061 tests/vendorrepair/+-90/failedpad/seams/alltrimBoolean or pixels/armvisualunits/full assembly/tool/strength/torque/longmaterials/electrical/wholearm/physical/exterior approval.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean']
    save();print('core',result['initial_core_passed'],'differences',result['differences'],flush=True)


if __name__=='__main__':main()

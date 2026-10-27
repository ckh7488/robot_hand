"""Exact-SHA replay of changed rear-module entries and restoration lists."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm069_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm069_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def samples(path):
    indexes={0,len(path['samples'])-1}
    if path['stage']=='world' and not path['id'].endswith('four_M2_base_nuts_last'):
        count=4 if '__rack_M3_' in path['id'] else 3
        indexes.update(range(max(0,len(path['samples'])-count),len(path['samples'])))
    return [path['samples'][i] for i in sorted(indexes)]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
    man=n.p.read(ROOT/'checkpoints/ARM069.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source=m.context();lookup={z['name']:z for z in rows};raw=n.p.read(HERE/'reviews/check.json');ev=n.p.read(HERE/'reviews/evidence.json')
    prior=n.p.read(ROOT/'cad/captive_module_assembly_v1/reviews/selected_check.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],contracts={},
        manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    for path in raw['paths']:
        for s in samples(path):
            got=n.c.material(m.moved([lookup[k] for k in path['moving_names']],np.array(s['transform'])),[lookup[k] for k in path['present_names']]);want=raw['native'][s['id']]
            l,r=d.overlap_map(got),d.overlap_map(want);error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
            ok=l.keys()==r.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            result['native_samples'].append(dict(id=s['id'],native=got,matched=bool(ok),max_volume_difference_mm3=error))
            if not ok:result['differences'].append(s['id'])
            save();print(s['id'],bool(ok),flush=True)
    packets=m.packets(rows);contracts=result['contracts'];contracts['original16packet408_final886_and_initial502']=packets==raw['original16packets'] and sum(len(p['names']) for p in packets)==408 and sorted(lookup)==raw['original_final886_names'] and raw['initial_world_present_names']==prior['packet_final_present_names']
    placements={p['module']:p for p in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements']}
    world=set(raw['initial_world_present_names']);seq=True;matrix_error=0.;identity_error=0.;outside=True
    for c in raw['module_contracts']:
        P=np.array(c['placement']);r=-P[:3,2];normal=P[:3,0]
        seq=seq and np.max(abs(P-np.array(placements[c['module']]['proposed_installation_transform'])))<1e-10
        seq=seq and np.max(abs(r-np.array(c['radial'])))<1e-10 and np.max(abs(normal-np.array(c['normal'])))<1e-10 and set(c['world_before_names'])==world
        body=set(c['body19_names']);unit=set(c['motor_and_own_electrical10_names']);carrier=set(c['carrier9_names'])
        seq=seq and unit|carrier==body and not unit&carrier and body|set(k for k in c['original24_names'] if k not in body)==set(c['original24_names']) and c['temporarily_later_base_nuts'] not in body
        bench={c['module']+'_cradle'};carrier_state=set(carrier)
        for p in [p for p in raw['paths'] if p['module']==c['module']]:
            present=world if p['stage']=='world' else bench if p['stage']=='bench' else carrier_state
            seq=seq and present==set(p['present_names']) and not present&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-present
            for s in p['samples']:matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
            if p['stage']=='world':
                moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));fixed=[lookup[k] for k in present]
                mb=m.project_bounds(moving,r);fb=m.project_bounds(fixed,r);gap=max(fb[0]-mb[1],mb[0]-fb[1])
                outside=outside and gap>0 and abs(gap-p['initial_radial_projection_separation_mm'])<1e-5
            if p['completed']:
                present.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        if c.get('completed'):seq=seq and world==set(c['world_after_names']) and set(c['original24_names'])<=world
    contracts['actual_bases_partitions_temporary_omissions_sequence_and_final_original_poses']=bool(seq and matrix_error<1e-10 and identity_error<1e-10)
    contracts['whole_obstacle_outside_start_for_executed_world_paths']=bool(outside)
    contracts['final_present_pending_partition_and_restored_count']=world==set(raw['packet_final_present_names']) and world|set(raw['packet_final_pending_names'])==set(lookup) and not world&set(raw['packet_final_pending_names']) and [p['module'] for p in packets if set(p['names'])<=world]==raw['restored_modules']
    contracts['planned7rear_and_explicit_later_front8']=raw['planned_rear_modules']==[p['module'] for p in packets if p['row']==0 and p['module'] not in prior['restored_modules']] and raw['explicitly_later_front8_modules']==[p['module'] for p in packets if p['row']==1]
    axes_ok=ev['actual_M2_M3_axes_match_declared_bases'] and len(ev['actual_axis_reads'])==2*len(raw['module_contracts'])
    for z in ev['actual_axis_reads']:
        cyl=[]
        for face in lookup[z['name']]['shape'].Faces():
            if face.geomType()=='CYLINDER':
                cy=BRepAdaptor_Surface(face.wrapped).Cylinder();axis=np.array(cy.Axis().Direction().Coord())
                cyl.append(dict(radius_mm=cy.Radius(),axis=axis,point=list(cy.Axis().Location().Coord()),axis_parallel=bool(abs(axis@np.array(z['expected_axis']))>1.-1e-10)))
        axes_ok=axes_ok and len(cyl)==len(z['cylinders']) and all(c['axis_parallel'] for c in cyl)
        for c,w in zip(cyl,z['cylinders']):axes_ok=axes_ok and abs(c['radius_mm']-w['radius_mm'])<1e-5 and np.max(abs(c['axis']-np.array(w['axis'])))<1e-5 and np.max(abs(np.array(c['point'])-np.array(w['point'])))<1e-5
    contracts['actual_M2_M3_cylinder_axis_reads_match_original_module_bases']=bool(axes_ok)
    saved=n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []
    for q,z in zip(ev['common_queries'],saved):
        sh=n.g.original.transform(lookup[q['lead']]['shape'],np.array(q['lead_transform'])).intersect(lookup[q['part']]['shape'],tol=1e-6)
        ok=sh.isValid() and len(sh.Solids())==q['solids'] and abs(sh.Volume()-q['volume_mm3'])<1e-5
        result['common_samples'].append(dict(name=q['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())))
        g=dict(name=q['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+q['name'])
        if not g['matched']:result['differences'].append('geometry_'+q['name'])
    for k,v in contracts.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),
        all_translation_matrix_max_error=matrix_error,completed_final_identity_max_error=identity_error,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New rear-module bench first/last;world body/nuts first and radial-to-axial last3;M3first and last4;M2nuts first/last,including first failure. Original16packet408/final886/initial502/actual module bases,body19-unit10-carrier9/omissions/lists/final original transforms/outside AABBstarts;new common/signature only if saved. Not all new middle samples,ARM06864or17proofs,oldfront/PCD/615faces/holds,alltrimBoolean/pixels/armvisualunits,continuousfactory/tools/captive retention/supplier/electrical/output internalassembly,physical/20drive/fullarm/exterior qualification.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()

"""Scoped exact-commit replay; no physical or complete hand-drive approval."""
import argparse,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import candidate2 as g
from OCP.OSD import OSD_ThreadPool
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/DRIVE018.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(g.sha(ROOT/z)==r['sha256'] for z,r in manifest['files'].items()) and all(g.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard()
    expected=p.read(HERE/'selected/reviews/check.json');assert expected['completed'] and expected['input_hash_guard'] and expected['sampled_and_envelope_clear']
    saved=p.read(HERE/'selected/generated/scene.json');new=c.load(saved['parts']);fresh,design=g.build();freshmap={z['name']:z for z in fresh}
    signature=g.module('_drive_geometry_signature',ROOT/'cad/palm_motor_layout_v1/review_core.py').signature
    comparisons=[];differences=[];probes=[]
    for row in new:
        a,z=signature(freshmap[row['name']]['shape'],m.pack._bbox),signature(row['shape'],m.pack._bbox)
        bad=[];numbers=[]
        if not a['valid'] or not z['valid'] or a['solids']!=z['solids']:bad.append('valid/solids')
        for key in ['global_values','vertices']:
            aa,zz=np.asarray(a[key]),np.asarray(z[key])
            if aa.shape!=zz.shape or not np.allclose(aa,zz,rtol=0,atol=1e-5):bad.append(key)
            else:numbers.extend(abs(aa-zz).ravel().tolist())
        for key in ['faces','edges']:
            if len(a[key])!=len(z[key]):bad.append(key+' count');continue
            for aa,zz in zip(a[key],z[key]):
                if aa[0]!=zz[0] or not np.allclose(aa[1],zz[1],rtol=0,atol=1e-5):bad.append(key)
                else:numbers.extend(abs(np.asarray(aa[1])-np.asarray(zz[1])).tolist())
        comparisons.append(dict(name=row['name'],matched=not bad,mismatch=bad,max_numeric_difference=max(numbers or [0.])))
        if bad:differences.append('signature_'+row['name'])
    def compare(label,got,want):
        def vals(r):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in r['overlaps']}
        aa,zz=vals(got),vals(want)
        ok=aa.keys()==zz.keys() and all(abs(aa[k]-zz[k])<1e-5 for k in aa.keys()&zz.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        probes.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    fixed,wires,arm,retained,replaced=g.fixed_inputs()
    for angle in [0.,140.,270.]:
        # Use reimported saved shapes and the independent world-axis transform.
        layout=p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
        T=np.asarray(next(z['transform_world_mm'] for z in layout['modules'] if z['name']==g.NAME))
        start=T[:3,3];end=start+T[:3,0]
        customnew=g.custom(new)
        moving=[dict(z,shape=z['shape'].rotate(tuple(start),tuple(end),angle)) for z in customnew]
        stockrotated=[dict(z,shape=z['shape'].rotate(tuple(start),tuple(end),angle) if z['moving'] else z['shape']) for z in new if z not in customnew]
        want=next(z for z in expected['rotations'] if z['angle_deg']==angle)
        for key,obstacles in [('packaging',fixed),('wires',wires),('arm',arm),('stock_reference',stockrotated)]:compare('rotation_'+str(angle)+'_'+key,c.material(moving,obstacles),want[key])
    compare('internal45pairs',g.pairs(new),expected['internal_pairs'])
    threads=g.thread_contacts(new)
    for aa,zz in zip(threads,expected['thread_contacts']):
        if any(abs(aa[k]-zz[k])>1e-5 for k in ['common_volume_mm3','outside_declared_region_mm3']) or aa['acceptable_nominal_thread_envelope']!=zz['acceptable_nominal_thread_envelope']:differences.append('thread_'+str(aa['screw']))
    custom=g.custom(fresh);stock=[z for z in fresh if z not in custom]
    screws=[z for z in custom if z['role']=='nominal_tapping_screw_not_vendor_cad'];spoolgroup=[z for z in custom if z not in screws]
    for shift in [20.,0.]:
        for label,rows,obstacles in [('spool_install',spoolgroup,fixed+wires+arm+stock),('screw_install',screws,fixed+wires+arm+stock+spoolgroup)]:
            want=next(z['native'] for z in expected[label] if z['shift_x_mm']==shift)
            compare(label+'_'+str(shift),c.material(g.moved(rows,0.,shift),obstacles),want)
    tools=[dict(name='provisional_driver_stem_'+str(i),shape=g.pose(cq.Solid.makeCylinder(1.05,18.,cq.Vector(5.1,y,z),cq.Vector(1,0,0)))) for i,(y,z) in enumerate([(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)],1)]
    compare('provisional_driver_stems',c.material(tools,fixed+wires+arm+stock+spoolgroup),expected['driver_stem_space'])
    swept=[]
    for i,(y,z) in enumerate([(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)],1):
        shaft=cq.Solid.makeCylinder(1.,26.,cq.Vector(-2.5,y,z),cq.Vector(1,0,0))
        head=cq.Solid.makeCylinder(2.1,21.6,cq.Vector(3.5,y,z),cq.Vector(1,0,0))
        swept.append(dict(name='screw_insertion_sweep_'+str(i),shape=g.pose(shaft.fuse(head))))
    compare('continuous_screw_insertion_sweep',c.material(swept,fixed+wires+arm+stock),expected['screw_insertion_swept_against_fixed'])
    envelope,description=g.custom_envelopes(fresh)
    for key,obstacles in [('packaging',fixed),('wires',wires),('arm',arm)]:compare('full_rotation_envelope_'+key,c.material(envelope,obstacles),expected['full_rotation_'+key])
    firstscene=p.read(HERE/'generated/scene.json');first=c.load(firstscene['parts'])
    firstspool=next(z for z in first if z['name']=='thumb_ip_two_tail_spool')
    failed=dict(firstspool,shape=firstspool['shape'].rotate(tuple(start),tuple(end),140.))
    adapter=next(z for z in fixed if z['name']==g.NAME+'_adapter')
    got=c.material([failed],[adapter]);old=expected['original_failed_pair']
    compare('original_boss_adapter_140deg',got,dict(pairs_in_scope=1,exact_pairs=1,overlaps=[dict(lead=old['lead'],part=old['part'],material_intersection_mm3=old['common_volume_mm3'])],operation_failures=[]))
    checker=g.module('_drive_replay_hand_bounds',ROOT/'cad/remote_bus_v1/bbox_check.py')
    bounds=checker.hand_boxes(envelope)
    for aa,zz in zip(bounds['cases'],expected['five_hand_pose_bounds']['cases']):
        if aa['id']!=zz['id'] or aa['native_candidates']!=zz['native_candidates'] or aa['clear']!=zz['clear'] or abs(aa['minimum_separating_axis_gap_mm']-zz['minimum_separating_axis_gap_mm'])>1e-5:differences.append('hand_boxes_'+aa['id'])
    clean=not git('status','--porcelain=v1')
    result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,
        key_samples=probes,geometry_signature_comparisons=comparisons,thread_contacts=threads,five_hand_pose_bounds=bounds,full_rotation_envelope_description=description,
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,
        OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,
        scope='10source/STEP geometry signatures(types/metrics/vertices,not all surface equations);23native probes:3output angles x4groups,internal pairs,2assembly endpoints x2groups,driver stems,screw insertion sweep,3full rotation envelope groups,original boss/adapter failure.Five hand endpoint saved-box bounds versus3full-rotation envelopes;no full364hand native import when disjoint.No tendon-to-finger path,force/strength/supplied screw/driver/physical approval;other exact output angles/assembly middle positions and render not rerun.')
    p.write(args.output,result)
    assert result['passed']


if __name__=='__main__':main()

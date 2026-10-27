"""Exact-commit replay of the two changed solids and retained-jaw classification."""
import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm055_review_model',Path(__file__).with_name('model.py'))
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE = a.n,a.np,a.d,a.ROOT,a.HERE
measure = n.m.drive.module('_arm055_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
partition = n.m.drive.module('_arm055_partition',HERE/'partition.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',type=Path,required=True)
    args = ap.parse_args()
    def git(*xs):
        return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD') == args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest = n.p.read(ROOT/'checkpoints/ARM055.manifest.json')
    frozen = {}
    for key in ['geometry_sha256','code_sha256','report_sha256']:
        frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():
        return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw = n.p.read(HERE/'reviews/check.json')
    evidence = n.p.read(HERE/'reviews/evidence.json')
    expected_partition = n.p.read(HERE/'reviews/partition.json')
    _,_,retained,T,old,source = a.context()
    fresh,design = a.build(T,source)
    saved = n.c.load(n.p.read(ROOT/raw['scene'])['parts'])
    result = dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],
        geometry_comparisons=[],ports_lengths_seats=[],partition_records=[],
        manifest_files=len(manifest['files']),frozen_files=len(frozen),OCC_pool_threads=1,
        current862_installation_replaced=False,proposal_adopted=False,
        inherited33_native_records_retested=False,ARM054_156_groups_reexecuted=False,
        same_agent_not_independent_audit=True,whole_drive_or_exterior_approved=False)
    def save():
        n.a.save(args.output,result)
    save()
    for left,right in zip(fresh,saved):
        assert left['name']==right['name']
        item = dict(name=left['name'],**measure.match(measure.metrics(left['shape']),measure.metrics(right['shape'])))
        result['geometry_comparisons'].append(item)
        if not item['matched']:
            result['differences'].append('geometry_'+left['name'])
    for got,want in zip(design,raw['design']):
        error = max(float(np.max(abs(np.array(got[k])-want[k]))) for k in
                    ['real_jaw_frame','free_port_world_mm','free_tangent_world','shoulder_to_free_port_length_mm','original_jaw_removed_mm3'])
        ok = error < 1e-10
        result['ports_lengths_seats'].append(dict(line=got['line'],maximum_difference=error,matched=bool(ok)))
        if not ok:
            result['differences'].append('port_length_seat_'+str(got['line']))
    def compare(label,got,want):
        l,r = d.overlap_map(got),d.overlap_map(want)
        error = max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok = l.keys()==r.keys() and error<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,maximum_volume_difference_mm3=error,matched=bool(ok)))
        if not ok:
            result['differences'].append(label)
        save()
        print(label,bool(ok),flush=True)
    all_retained = [z for rows in retained.values() for z in rows]
    compare('new2__retained870_neutral',n.c.material(saved,all_retained),raw['native']['new2__retained870_neutral'])
    compare('new2_internal',n.c.material(saved[:1],saved[1:]),raw['native']['new2_internal'])
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in raw['motion']:
        F = d.transforms(state['angles_deg'],arm)
        assert max(np.max(abs(F[key]-state['transforms'][key])) for key in F)<1e-10
        moving = [dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in saved]
        for target in state['compared_frames']:
            label = state['id']+'__new2__'+target
            obstacle = [dict(z,shape=n.g.original.transform(z['shape'],F[target])) for z in retained[target]]
            compare(label,n.c.material(moving,obstacle),raw['native'][label])
    got_partition,_ = partition.classify(raw,evidence)
    for got,want in zip(got_partition,expected_partition['records']):
        error = max(abs(got[k]-want[k]) for k in ['reconstructed_common_volume_mm3','common_outside_original_unbent_jaw_mm3'])
        ok = got['record']==want['record'] and got['outside_original_jaw_solids']==want['outside_original_jaw_solids'] and error<1e-5
        result['partition_records'].append(dict(record=got['record'],actual=got,maximum_difference_mm3=error,matched=bool(ok)))
        if not ok:
            result['differences'].append('partition_'+got['record']['lead']+'__'+got['record']['part'])
        save()
    inherited = raw['inherited_unchanged_overlap_records']
    assert len(inherited)==33 and all(z['record']['lead'] not in a.OLD_NAMES and z['record']['part'] not in a.OLD_NAMES for z in inherited)
    result.update(completed=True,initial_core_passed=not result['differences'],
        native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        port_length_seat_matches=sum(z['matched'] for z in result['ports_lengths_seats']),
        partition_matches=sum(z['matched'] for z in result['partition_records']),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='All26new-guide native groups7129pairs with8neutral records;2newsourceSTEPmetrics epsilon1e-10/absolute1e-5;2ports/lengths/jaw-seat frames/original-jaw removed0;8common-minus-jaw partitions with7newcommon constructions and1savedwitness reuse.33inheritedunchanged records are metadata/identity carry,not native replay. NoARM054156/oldarm groups,assembly/renderpixels/alltrimBoolean/fullmaterials/physical/whole20drive/exterior approval.')
    save()


if __name__=='__main__':
    main()

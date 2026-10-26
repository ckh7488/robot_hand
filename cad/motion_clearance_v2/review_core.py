"""Exact-commit core replay. Preserved collisions are required to reproduce.

All parts and the committed closure are hashed. Full 180-case and all adaptive
distance calculations are not rerun: selected full-hand poses and five saved
distance samples per finger are recomputed, and every stored interval bound is
checked arithmetically against the hash-identical native geometry.
"""
import argparse, hashlib, importlib.util, json, math, subprocess, time
from pathlib import Path
import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();start=time.monotonic()
    if a.output.exists() or a.output.resolve().is_relative_to(ROOT):raise ValueError('Use a new output outside the clean checkout')
    def git(*args):
        return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=a.commit or git('status','--porcelain=v1'):
        raise ValueError('Wrong/dirty commit')
    manifest=read(ROOT/'checkpoints/MOTION002.manifest.json')
    def guard():
        return all(sha(ROOT/k)==h['sha256'] for k,h in manifest['files'].items())
    if not guard():raise ValueError('Committed input closure changed')
    spec=importlib.util.spec_from_file_location('_core_native',ROOT/'cad/gesture_hand_v3/audit_gestures.py')
    native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native);v=native.load_v2(ROOT)
    scene=read(HERE/'generated/scene.json');design=read(HERE/'generated/design.json');parts={}
    for r in scene['hand']:
        path=(ROOT/r['step']).resolve()
        if not path.is_relative_to(ROOT) or sha(path)!=r['sha256_step']:raise ValueError('Changed STEP')
        parts[r['name']]=cq.importers.importStep(str(path)).val()
    source=read(HERE/'reviews/audit.json')
    if not source['completed'] or not source['input_hash_guard']['passed']:raise ValueError('Incomplete committed audit')
    for k,h in source['source_sha256'].items():
        if sha(ROOT/k)!=h:raise ValueError('Stale committed audit source '+k)
    original_palm=cq.importers.importStep(str(ROOT/'cad/gesture_hand_v3/generated/parts/Palm_tapered_chassis.step')).val()
    candidate_palm=parts['Palm_tapered_chassis']
    masks={'floor':cq.Solid.makeBox(400,500,20,cq.Vector(-200,-100,-60.5))}
    for f in design['fingers']:
        x,y,z=f['root'];masks[f['name']+'_rear_mount']=cq.Solid.makeBox(32,14,40,cq.Vector(x-16,y-15,-50))
    protected={k:original_palm.intersect(mask).cut(candidate_palm).Volume() for k,mask in masks.items()}
    if max(protected.values())>1e-6 or candidate_palm.cut(original_palm).Volume()>1e-6:
        raise ValueError('Saved candidate lost a protected support or added material')
    engine=native.PairScreen(v,parts,scene['hand'],design,0.,True)
    wanted={'gesture_open','gesture_fist','gesture_thumb_up','four_fingers_extension','four_fingers_curl','fan_1',
        'index_pip_+135','thumb_mcp_+90',*[f+'_ordinary_extension_mcp' for f in ['index','middle','ring','little']],
        'gesture_open__gesture_fist@0.75','gesture_open__gesture_middle_finger@0.75'}
    selected=[r for r in source['cases'] if r['id'] in wanted]
    probes=[]
    for row in selected:
        probes.append(dict(id=row['id'],angles_deg=row['angles_deg'],expected=row['changed_palm_collisions']+row['inherited_invariant_collisions']))
    for row in source['extra_extension_endpoints']:
        probes.append(dict(id=row['finger']+'_mcp_-25',angles_deg=row['result']['angles_deg'],expected=row['result']['collision_details']))
    results=[];differences=[]
    def signature(rows):return {tuple(sorted([r['a'],r['b']])):r['material_intersection_mm3'] for r in rows}
    for row in probes:
        actual=engine.evaluate(row['angles_deg']);old=signature(row['expected']);new=signature(actual['collision_details'])
        if old.keys()!=new.keys() or actual['operation_failures']:differences.append(dict(id=row['id'],reason='Changed pair set/calculation failure'))
        for k in old.keys() & new.keys():
            if abs(old[k]-new[k])>1e-5:differences.append(dict(id=row['id'],pair=k,before=old[k],after=new[k]))
        actual.pop('exact_candidate_results');actual.pop('near_pairs');results.append(dict(id=row['id'],result=actual))
        print(json.dumps(dict(probe=row['id'],collisions=actual['collision_count'])),flush=True)
    palm=parts['Palm_tapered_chassis'];neutral=v.frames(design,np.zeros(20));gap_checks=[]
    for fi,path in enumerate(source['local_path_bounds']):
        finger=path['finger'];body=finger+'_mcp';name=finger+'_segment1_segment_shell'
        available={r['angle_deg']:r['gap_mm'] for r in path['samples']}
        angles=sorted({min(available,key=lambda k:abs(k-target)) for target in [-25,-18.75,-12.5,-6.25,0]})
        for angle in angles:
            q=np.zeros(20);q[fi*4+1]=angle;frames=v.frames(design,q)
            gap=float(palm.distance(v.relocate(parts[name],frames[body],neutral[body])))
            if not math.isfinite(gap) or abs(gap-available[angle])>1e-5:
                differences.append(dict(id=finger,angle=angle,expected_gap=available[angle],actual_gap=gap))
            gap_checks.append(dict(finger=finger,angle_deg=angle,actual_gap_mm=gap,expected_gap_mm=available[angle]))
        # Every interval uses the nearest one of its endpoint samples. Validate
        # coverage and the declared speed/error debit without resampling them all.
        intervals=sorted(path['bounded_intervals'],key=lambda r:r['angle_interval_deg'][0])
        if not intervals or path['failed_intervals'] or intervals[0]['angle_interval_deg'][0]!=-25 or intervals[-1]['angle_interval_deg'][1]!=0:
            raise ValueError('Incomplete bounded path')
        last=-25.
        for interval in intervals:
            lo,hi=interval['angle_interval_deg']
            if lo!=last:raise ValueError('Path coverage gap')
            lower=min(available[lo],available[hi])-path['distance_speed_bound_mm_per_degree']*(hi-lo)/2-1e-5
            if abs(lower-interval['gap_lower_bound_mm'])>1e-10 or lower<.5:raise ValueError('Invalid stored bound')
            last=hi
    final_guard=guard();status=git('status','--porcelain=v1')
    result=dict(target_commit=a.commit,passed=not differences and final_guard and not status,
        independent_audit=False,manifest_files_checked=len(manifest['files']),input_hash_guard=final_guard,
        remeasured_protected_support_removed_mm3=protected,
        selected_full_hand_poses=len(results),all_pair_scope_per_pose=364*363//2,cases=results,
        recomputed_native_gap_samples=len(gap_checks),gap_checks=gap_checks,
        all_recorded_interval_bound_arithmetic_rechecked=True,
        full180_cases_replayed=False,all_adaptive_native_distances_recomputed=False,renderer_replayed=False,
        clean_git_status=status,differences=differences,elapsed_s=time.monotonic()-start,
        meaning='Selected exact-SHA findings, including remaining collisions, reproduced; same-agent core replay.',
        manufacturing_release=False,whole_hand_motion_approved=False)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

"""Prepare a fresh exact-SHA input copy and replay representative thumb samples.
CLI: --sha FULL_COMMIT --output NEW_DIRECTORY_UNDER_WORK_REVIEWS
The parent runs this later. No Git mutation, CAD rebuild, renderer or full-path rerun.
"""
from pathlib import Path
import argparse, hashlib, json, math, os, platform, re, subprocess, sys, time, traceback

ROOT=Path('D:/robot_hand').resolve(); WORK=Path(__file__).resolve().parent
REVIEWS=WORK.parent/'reviews'; CAD='cad/thumb_wrist_v1'; MANIFEST=CAD+'/manifest.json'
BASE='fae9514622079e5349b4b429553202e32cb60c2b'; GIT='C:/Program Files/Git/cmd/git.exe'
BATCH=WORK.parent/'gesture002/git_blob_batch.py'; BATCH_SHA='28e37b501bac04630ffb953bfe6ccc1cb81674830e475834c539b0c8abd5f386'
OWN={CAD+'/'+p for p in ('wrist_profile.py','profile.json','audit.py','render.py','README.md','reports/motion_audit.json',
     'generated/comparison.png','generated/candidate.png','generated/motion.gif','generated/render_report.json')}
def require(ok,message):
    if not ok:raise ValueError(message)
def h(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):value.update(block)
    return value.hexdigest()
def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))
def angles(values):
    require(isinstance(values,list) and len(values)==20 and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values),'Expected twenty finite angles')
    return tuple(float(v).hex() for v in values)
def grid(rows):
    require(isinstance(rows,list) and rows and len({r['id'] for r in rows})==len(rows),'Empty/duplicate observation IDs')
    return [dict(metadata={k:v for k,v in r.items() if k not in ('angles_deg','case_id')},angles_hex=angles(r['angles_deg'])) for r in rows]
def pair_map(case):
    result={}
    for r in case['collision_details']:
        key=tuple(sorted((r['a'],r['b'])));v=r['material_intersection_mm3']
        require(key not in result and math.isfinite(v) and v>=0,'Invalid collision pair/volume');result[key]=v
    require(case['collision_count']==len(result),'Collision count differs from detail rows');return result
def compare(a,b):
    aa,bb=pair_map(a),pair_map(b);delta=max((abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()),default=0.)
    ae,be=a['operation_failures'],b['operation_failures']
    return dict(angles_match=angles(a['angles_deg'])==angles(b['angles_deg']),missing_pairs=sorted(aa.keys()-bb.keys()),
        extra_pairs=sorted(bb.keys()-aa.keys()),maximum_volume_difference_mm3=delta,original_operation_failures=ae,
        recomputed_operation_failures=be,passed=angles(a['angles_deg'])==angles(b['angles_deg']) and aa.keys()==bb.keys() and delta<=.02 and not ae and not be)
def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--sha',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    require(re.fullmatch('[0-9a-f]{40}',a.sha),'Full commit SHA required');dest=a.output.resolve()
    require(dest.is_relative_to(REVIEWS.resolve()) and dest!=REVIEWS.resolve() and not dest.exists(),'Use a new review directory on D:')
    for key in ('GIT_DIR','GIT_WORK_TREE','GIT_INDEX_FILE'):require(not os.environ.get(key),'Refuse inherited '+key)
    require(h(BATCH)==BATCH_SHA,'Batch helper changed');sys.dont_write_bytecode=True;sys.path.insert(0,str(BATCH.parent))
    from git_blob_batch import ImmutableGitBlobs, repository_path
    dest.mkdir(parents=True);repo=dest/'repository';repo.mkdir();tmp=dest/'tmp';tmp.mkdir()
    env=dict(os.environ,GIT_NO_REPLACE_OBJECTS='1',GIT_OPTIONAL_LOCKS='0',GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never',
             OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',TEMP=str(tmp),TMP=str(tmp))
    state=dict(target_sha=a.sha,completed=False,reproduction_passed=False,commands=[],errors=[],reviewer_sha256=h(__file__),
        batch_helper_sha256=BATCH_SHA,environment=dict(python=sys.executable,version=sys.version,platform=platform.platform(),pythonpath=env.get('PYTHONPATH'),blas_threads=1),
        scope='Fresh exact-commit manifest input closure; no rebuild. Reconcile all original sample angles and rerun only representative recorded samples against all349 saved STEP parts. No full-path replay, distance, renderer, dynamics, hardware or manufacturing approval.')
    def save():(dest/'review.json').write_text(json.dumps(state,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    def run(label,argv,cwd):
        row=dict(label=label,argv=list(map(str,argv)),cwd=str(cwd),completed=False);state['commands'].append(row);save();start=time.monotonic()
        with (dest/(label+'.log')).open('wb') as out,(dest/(label+'.err')).open('wb') as err:
            p=subprocess.run(row['argv'],cwd=cwd,env=env,stdin=subprocess.DEVNULL,stdout=out,stderr=err,shell=False,creationflags=subprocess.CREATE_NO_WINDOW)
        row.update(completed=True,exit_code=p.returncode,elapsed_s=time.monotonic()-start);save();require(p.returncode==0,'Failed command: '+label)
    def inside(name):
        p=repo/repository_path(name);require(p.resolve()==p and p.is_file(),'Missing/aliased committed input: '+name);return p
    def report_guard(value,origin):
        g=value.get('input_hash_guard',{});require(value.get('completed') is True and not value.get('errors') and g.get('passed') is True and bool(g.get('start')) and g['start']==g.get('end'),'Incomplete audit/input guard')
        normalized={}
        for name,expected in g['start'].items():
            p=Path(name)
            if p.is_absolute():require(p.is_relative_to(origin),'Input outside recorded checkout');name=p.relative_to(origin).as_posix()
            require(name not in normalized and name in manifest['files'] and h(inside(name))==expected==manifest['files'][name]['sha256'],'Audit input differs from immutable manifest: '+name)
            normalized[name]=expected
        return normalized
    def coverage(value):
        rows=value['observations'];cases=value['cases'];require(len({r['id'] for r in rows})==len(rows) and rows,'Duplicate/missing observations')
        require({r['case_id'] for r in rows}==set(cases),'Missing or unreferenced cases')
        for r in rows:require(angles(r['angles_deg'])==angles(cases[r['case_id']]['angles_deg']),'Observation/case angle mismatch')
        require(len(value['parts'])==349 and len({r['name'] for r in value['parts']})==349 and all(r['valid'] and r['solid_count']==1 for r in value['parts']),'Invalid/incomplete native parts')
        require(value.get('assembly_roundtrip',{}).get('passed') is True,'Assembly/individual STEP mismatch')
        require(all(c['total_pairs_in_scope']==349*348//2 for c in cases.values()),'Incomplete all-part pair scope')
        s=value['summary'];require(s['parts']==349 and s['unique_pose_count']==len(cases) and s['observation_count']==len(rows),'Summary/actual coverage differs')
        require(s['cases_with_material_collision']==sum(bool(c['collision_details']) for c in cases.values()) and s['operation_failure_count']==sum(len(c['operation_failures']) for c in cases.values()),'Summary/failure details differ')
    blobs=ImmutableGitBlobs(GIT,ROOT,env,tmp)
    try:
        run('resolve_commit',[GIT,'rev-parse',a.sha+'^{commit}'],ROOT);require((dest/'resolve_commit.log').read_text().strip()==a.sha,'SHA is not an exact commit')
        raw=blobs.read(a.sha,MANIFEST);manifest=json.loads(raw)
        require(manifest.get('id')=='thumb_wrist_v1' and manifest.get('baseline_commit')==BASE and set(manifest['candidate_files'])==OWN and OWN<=set(manifest['files']),'Manifest identity/candidate coverage differs')
        for name,item in manifest['files'].items():
            repository_path(name);signature=blobs.signature(a.sha,name)
            require(signature['bytes']==item['bytes'] and signature['sha256']==item['sha256'],'Committed manifest bytes differ: '+name)
            if name not in OWN:require(blobs.signature(BASE,name)['sha256']==item['sha256'],'Base geometry/dependency changed: '+name)
            p=repo/name;p.parent.mkdir(parents=True,exist_ok=True)
            with p.open('xb') as f:f.write(blobs.read(a.sha,name))
            require(h(p)==item['sha256'],'Extracted byte mismatch: '+name)
        (repo/MANIFEST).write_bytes(raw);state.update(manifest_sha256=h(repo/MANIFEST),manifest_files_checked=len(manifest['files']),immutable_blob_reads=blobs.stats());save()
        frozen={n:h(inside(n)) for n in set(manifest['files'])|{MANIFEST}}
        old=read(inside(CAD+'/reports/motion_audit.json'));oldguard=report_guard(old,ROOT);coverage(old)
        require(old.get('mode')=='full' and old['summary']['full_sampled_path_executed'] is True and old['summary']['cases_with_material_collision']==0 and old['summary']['operation_failure_count']==0,'Original full path did not pass')
        require(grid(old['full_sampling_observations'])==grid(old['observations']),'Original full sampling grid differs from executed observations')
        output=dest/'representative_audit.json'
        run('representative_audit',[sys.executable,'-B',str(repo/CAD/'audit.py'),'--repo',str(repo),'--mode','representative','--output',str(output)],repo)
        new=read(output);newguard=report_guard(new,repo);coverage(new)
        require(newguard==oldguard,'Representative execution changed source/input coverage')
        require(new.get('mode')=='representative' and new['summary']['full_sampled_path_executed'] is False,'Unexpected replay scope')
        require(old['definition']==new['definition'] and old['full_path_definition']==new['full_path_definition'] and old['thresholds']==new['thresholds'] and old['direction']==new['direction'],'Path/profile/threshold definition changed')
        require(grid(old['full_sampling_observations'])==grid(new['full_sampling_observations']),'Recomputed full angle grid differs')
        path=[r for r in old['full_sampling_observations'] if r['kind']=='transition'];spec=read(inside(CAD+'/profile.json'))
        fractions=[0.,.25,.5,.75,spec['finger_closing_phase_end'],1.]
        ids={min(path,key=lambda r:(abs(r['path_fraction']-f),r['path_fraction']))['id'] for f in fractions}
        expected=[r for r in old['full_sampling_observations'] if r['kind']=='endpoint' or r['id'] in ids]
        require(grid(expected)==grid(new['observations']),'Representative selection differs from recorded sample grid')
        original={r['id']:r for r in old['observations']};comparisons=[]
        for r in new['observations']:
            comparisons.append(dict(observation=r['id'],angles_hex=angles(r['angles_deg']),**compare(old['cases'][original[r['id']]['case_id']],new['cases'][r['case_id']])))
        after={n:h(inside(n)) for n in frozen};unchanged=frozen==after
        state.update(completed=True,full_sampling_angles_reconciled=True,original_full_sample_count=len(path),selected_observation_count=len(expected),
            representative_unique_cases=len(new['cases']),comparisons=comparisons,source_hash_guard=dict(start=frozen,end=after,passed=unchanged),
            reviewer_source_unchanged=h(__file__)==state['reviewer_sha256'] and h(BATCH)==BATCH_SHA,
            representative_output_sha256=h(output),full_path_rerun=False)
        state['reproduction_passed']=unchanged and state['reviewer_source_unchanged'] and all(c['passed'] for c in comparisons)
    except Exception as e:state['errors'].append(dict(error=repr(e),traceback=traceback.format_exc()))
    finally:blobs.close(abort=True);save()
    print(json.dumps({k:state[k] for k in ('completed','reproduction_passed','errors')},ensure_ascii=False));return 0 if state['reproduction_passed'] else 1
if __name__=='__main__':raise SystemExit(main())

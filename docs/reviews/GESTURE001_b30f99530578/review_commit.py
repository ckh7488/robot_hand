"""Rebuild and rerun a bounded core audit from one committed snapshot on D:."""
from pathlib import Path
import hashlib,json,os,re,subprocess,sys,tarfile,time,traceback
import numpy as np
ORIGINAL=Path('D:/robot_hand'); WORK=Path(__file__).parent
GIT='C:/Program Files/Git/cmd/git.exe'; PY=sys.executable
sha=sys.argv[1]; dest=Path(sys.argv[2]).resolve()
if not re.fullmatch('[0-9a-f]{40}',sha):raise ValueError('Full commit SHA required')
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',
    TEMP='D:/Codex-work/robot-hand-workspace/work/tmp',TMP='D:/Codex-work/robot-hand-workspace/work/tmp',
    PYTHONDONTWRITEBYTECODE='1',GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never')
if not dest.is_relative_to(Path('D:/Codex-work/robot-hand-workspace/work/reviews').resolve()):raise ValueError('Unexpected review destination')
if dest.exists():raise FileExistsError('Preserve prior review')
dest.mkdir(parents=True);repo=dest/'repository';repo.mkdir()
report=dict(target_sha=sha,completed=False,reproduction_passed=False,manufacturing_approved=False,commands=[],errors=[])
def h(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
report['reviewer_sha256']=h(__file__)
def save():(dest/'review.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
def run(label,argv,cwd):
    started=time.monotonic()
    with (dest/(label+'.log')).open('wb') as out,(dest/(label+'.err')).open('wb') as err:
        p=subprocess.run(argv,cwd=cwd,stdout=out,stderr=err,stdin=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW,shell=False)
    report['commands'].append(dict(label=label,argv=argv,cwd=str(cwd),exit_code=p.returncode,seconds=time.monotonic()-started));save()
    if p.returncode:raise RuntimeError('Command failed: '+label)
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def inside(relative):
    path=(repo/relative).resolve()
    if not path.is_relative_to(repo.resolve()) or not path.is_file():raise ValueError('Missing/escaping committed input: '+relative)
    return path
def interface_outcomes(value,path=()):
    result={}
    if isinstance(value,dict):
        if 'passed' in value:
            identity={k:value[k] for k in ('id','part','a','b','parts','definition','label','against_part') if k in value}
            result['/'.join(path)]=dict(identity=identity,passed=value['passed'],error=value.get('error'))
        for k,v in value.items():
            if k not in ('input_hash_guard','source_sha256','runtime'):result.update(interface_outcomes(v,(*path,k)))
    elif isinstance(value,list):
        for i,v in enumerate(value):result.update(interface_outcomes(v,(*path,str(i))))
    return result
try:
    p=subprocess.run([GIT,'rev-parse',sha+'^{commit}'],cwd=ORIGINAL,capture_output=True,text=True,stdin=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW)
    if p.returncode or p.stdout.strip()!=sha:raise ValueError('Commit identity did not resolve')
    report['resolved_commit_sha']=p.stdout.strip()
    archive=dest/'committed.tar'
    run('archive',[GIT,'archive','--format=tar','--output='+str(archive),sha],ORIGINAL)
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            target=(repo/member.name).resolve()
            if not target.is_relative_to(repo.resolve()) or member.issym() or member.islnk():raise ValueError('Unsafe archive entry')
        tar.extractall(repo,filter='data')
    state=read(repo/'CHECKPOINT.json');manifest=read(inside(state['manifest']))
    checked=[]
    for relative,item in manifest['files'].items():
        path=inside(relative)
        if h(path)!=item['sha256'] or path.stat().st_size!=item['bytes']:raise ValueError('Manifest mismatch: '+relative)
        checked.append(relative)
    report['manifest_files_checked']=len(checked)
    frozen=read(repo/'AUDIT_EVIDENCE_INDEX.json');frozen_paths=set()
    for group in ('geometry_sha256','code_sha256','report_sha256'):
        for relative,digest in frozen[group].items():
            if h(inside(relative))!=digest:raise ValueError('Frozen evidence mismatch: '+relative)
            frozen_paths.add(relative)
    if len(frozen_paths)!=222:raise ValueError('Frozen evidence count changed')
    report['frozen_files_checked']=len(frozen_paths)
    cad=repo/'cad/gesture_hand_v3';expected_scene=read(cad/'generated/scene.json');expected_mass=read(cad/'generated/mass_inertia.json')
    expected_audit=read(cad/'reviews/gesture_endpoints.json')
    if not expected_audit['completed'] or not expected_audit['input_hash_guard']['passed']:raise ValueError('Baseline endpoint audit incomplete')
    source_start={p.relative_to(repo).as_posix():h(p) for p in repo.rglob('*.py')}
    run('rebuild',[PY,'-B','cad/gesture_hand_v3/build.py','--stage','fabrication'],repo)
    rebuilt_scene=read(cad/'generated/scene.json');rebuilt_mass=read(cad/'generated/mass_inertia.json')
    old={r['name']:r for r in expected_scene['hand']};new={r['name']:r for r in rebuilt_scene['hand']}
    if old.keys()!=new.keys():raise ValueError('Rebuilt part identities differ')
    geometry=[]
    for name,a in old.items():
        b=new[name];dv=abs(a['volume_mm3']-b['volume_mm3']);db=float(np.max(np.abs(np.array(a['bbox_mm'])-b['bbox_mm'])))
        geometry.append(dict(name=name,volume_error_mm3=dv,bounds_error_mm=db,passed=dv<=.05 and db<=.002 and a['body']==b['body']))
    oldm={r['name']:r for r in expected_mass['parts']};newm={r['name']:r for r in rebuilt_mass['parts']};masses=[]
    if len(oldm)!=len(expected_mass['parts']) or len(newm)!=len(rebuilt_mass['parts']) or oldm.keys()!=old.keys() or newm.keys()!=new.keys():raise ValueError('Mass names duplicate or differ from geometry')
    for name,a in oldm.items():
        b=newm[name];dm=abs(a['mass_kg']-b['mass_kg']);dc=float(np.max(np.abs(np.array(a['com_world_mm'])-b['com_world_mm'])));di=float(np.max(np.abs(np.array(a['inertia_com_world_kg_m2'])-b['inertia_com_world_kg_m2'])))
        masses.append(dict(name=name,mass_error_kg=dm,com_error_mm=dc,inertia_error_kg_m2=di,passed=dm<=1e-8 and dc<=1e-4 and di<=1e-9))
    report['geometry_reproduction']=geometry;report['mass_tensor_reproduction']=masses;save()
    poses=read(cad/'poses.json');ids=['open','fist','thumb_up'];poses['gestures']=[p for p in poses['gestures'] if p['id'] in ids]
    if len(poses['gestures'])!=3:raise ValueError('Core pose selection missing')
    poses['transitions']=[];core=dest/'core_poses.json';core.write_text(json.dumps(poses,indent=2))
    run('core_collisions',[PY,'-B','cad/gesture_hand_v3/audit_gestures.py','--cad-root',str(cad),'--poses',str(core),'--mode','endpoints','--skip-proximity','--output',str(dest/'core_collisions.json')],repo)
    got=read(dest/'core_collisions.json');comparisons=[]
    for name in ids:
        first=expected_audit['cases'][expected_audit['gesture_results'][name]['case_id']]
        second=got['cases'][got['gesture_results'][name]['case_id']]
        aa={(c['a'],c['b']):c['material_intersection_mm3'] for c in first['collision_details']}
        bb={(c['a'],c['b']):c['material_intersection_mm3'] for c in second['collision_details']}
        delta=max((abs(aa[k]-bb[k]) for k in aa.keys() & bb.keys()),default=0.)
        comparisons.append(dict(gesture=name,original_pairs=len(aa),recomputed_pairs=len(bb),missing_pairs=sorted(aa.keys()-bb.keys()),extra_pairs=sorted(bb.keys()-aa.keys()),maximum_overlap_difference_mm3=delta,passed=aa.keys()==bb.keys() and delta<=.02 and not second['operation_failures']))
    report['core_collision_reproduction']=comparisons
    run('interfaces',[PY,'-B','cad/gesture_hand_v3/audit_fabrication.py','--output',str(dest/'interfaces.json')],repo)
    old_i=read(cad/'reviews/fabrication_interfaces.json');new_i=read(dest/'interfaces.json')
    if not old_i['completed'] or not old_i['input_hash_guard']['passed'] or not new_i['completed'] or not new_i['input_hash_guard']['passed']:raise ValueError('Incomplete interface audit')
    old_outcomes=interface_outcomes(old_i);new_outcomes=interface_outcomes(new_i)
    report['interface_individual_checks_compared']=len(old_outcomes)
    report['interface_changed_checks']=[k for k in sorted(old_outcomes.keys()|new_outcomes.keys()) if old_outcomes.get(k)!=new_outcomes.get(k)]
    report['interface_summary_reproduced']=old_i.get('summary')==new_i.get('summary') and not report['interface_changed_checks']
    report['interface_original_summary']=old_i.get('summary');report['interface_recomputed_summary']=new_i.get('summary')
    report['source_hash_guard']=dict(start=source_start,end={p.relative_to(repo).as_posix():h(p) for p in repo.rglob('*.py')})
    report['source_hash_guard']['passed']=report['source_hash_guard']['start']==report['source_hash_guard']['end']
    report['reviewer_source_unchanged']=h(__file__)==report['reviewer_sha256']
    report['completed']=True
    report['reproduction_passed']=bool(all(x['passed'] for x in geometry+masses+comparisons) and report['interface_summary_reproduced'] and report['source_hash_guard']['passed'] and report['reviewer_source_unchanged'])
    report['scope']='Exact committed archive and manifest/frozen evidence; rebuilt all CAD/mass tensors; reran open/fist/thumb-up exact collision and nominal interface audits. Other six endpoints, transitions and renderer were not rerun. Existing failure outcomes must reproduce; this is not a manufacturing release or external independent audit.'
except Exception as exc:report['errors'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
save();print(json.dumps(dict(completed=report['completed'],reproduction_passed=report['reproduction_passed'],errors=report['errors'])),flush=True)
raise SystemExit(0 if report['reproduction_passed'] else 1)

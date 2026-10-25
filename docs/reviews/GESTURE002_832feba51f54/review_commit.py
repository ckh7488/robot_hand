"""GESTURE002 same-task exact-SHA reproduction, executed only by the parent.

Rebuild native CAD/mass, replay three static gestures and selected exact samples
from three committed forward paths, then replay nominal fabrication interfaces.
The selected samples do not rerun or certify the complete 2.5-degree paths.
Existing collision outcomes may reproduce without granting manufacturing approval.
CLI: python review_commit.py FULL_SHA NEW_DIRECTORY_UNDER_WORK_REVIEWS
"""
from pathlib import Path, PurePosixPath, PureWindowsPath
import copy,hashlib,json,math,os,re,subprocess,sys,tarfile,time,traceback
ORIGINAL=Path('D:/robot_hand'); WORK=Path(__file__).parent
GIT='C:/Program Files/Git/cmd/git.exe'; PY=sys.executable
CAD_REL='cad/gesture_hand_v3'
CORE_CATALOG_REL=CAD_REL+'/core_paths.json'
CORE_REPORT_REL=CAD_REL+'/reviews/core_transitions.json'
FORWARD_IDS=['open__fist','open__thumb_up','fist__thumb_up']
CORE_IDS=['open','fist','thumb_up']
STEP_DEG=2.5
PATH_FRACTIONS=[0.,.25,.5,.75,1.]
STAGED_PATHS=['open__thumb_up','fist__thumb_up']
SELECTION_POLICY='path_quarters_plus_staged_thumb_waypoints_v1'
sha=sys.argv[1]; dest=Path(sys.argv[2]).resolve()
if not re.fullmatch('[0-9a-f]{40}',sha):raise ValueError('Full commit SHA required')
for key in ('GIT_DIR','GIT_WORK_TREE','GIT_INDEX_FILE'):
    if os.environ.get(key):raise ValueError('Refuse inherited '+key)
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',
    TEMP='D:/Codex-work/robot-hand-workspace/work/tmp',TMP='D:/Codex-work/robot-hand-workspace/work/tmp',
    PYTHONDONTWRITEBYTECODE='1',GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never',GIT_NO_REPLACE_OBJECTS='1')
import numpy as np  # Configure BLAS before the first numerical-library import.
review_root=Path('D:/Codex-work/robot-hand-workspace/work/reviews').resolve()
if dest==review_root or not dest.is_relative_to(review_root):raise ValueError('Unexpected review destination')
if dest.exists():raise FileExistsError('Preserve prior review')
dest.mkdir(parents=True);repo=dest/'repository';repo.mkdir()
report=dict(target_sha=sha,completed=False,reproduction_passed=False,manufacturing_approved=False,commands=[],errors=[])
def h(p):
    digest=hashlib.sha256()
    with Path(p).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()
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
    if not isinstance(relative,str) or '\\' in relative or ':' in relative or any(ord(c)<32 for c in relative):raise ValueError('Unsafe relative input')
    lexical=PurePosixPath(relative)
    if not relative or lexical.is_absolute() or str(lexical)!=relative or any(v in ('..','.git') for v in lexical.parts):raise ValueError('Unsafe relative input: '+relative)
    path=(repo/relative).resolve()
    if not path.is_relative_to(repo.resolve()) or not path.is_file():raise ValueError('Missing/escaping committed input: '+relative)
    return path
def source_relative(value):
    """Translate a recorded original D: source to the same committed path."""
    if not isinstance(value,str):raise ValueError('Source path must be text')
    original=PureWindowsPath(str(ORIGINAL));path=PureWindowsPath(value)
    if path.is_absolute():
        try:relative=path.relative_to(original).as_posix()
        except ValueError as exc:raise ValueError('Recorded source lies outside original repository: '+value) from exc
    else:relative=value.replace('\\','/')
    inside(relative)
    return relative
def completed_guard(value,label):
    guard=value.get('input_hash_guard',{})
    if value.get('completed') is not True or value.get('errors') or guard.get('passed') is not True or not guard.get('start') or guard.get('start')!=guard.get('end'):
        raise ValueError(label+' is incomplete or has an inconsistent input guard')
def original_input_guard(value,label):
    completed_guard(value,label);seen={}
    for name,digest in value['input_hash_guard']['start'].items():
        relative=source_relative(name)
        if relative in seen or not isinstance(digest,str) or not re.fullmatch('[0-9a-f]{64}',digest) or h(inside(relative))!=digest:
            raise ValueError(label+' input differs from exact committed snapshot: '+name)
        seen[relative]=digest
    return seen
def angle_key(values):
    if not isinstance(values,list) or len(values)!=20 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
        raise ValueError('Expected exactly twenty finite joint angles')
    return tuple(float(v).hex() for v in values)
def unique_rows(rows,label):
    result={row['id']:row for row in rows}
    if len(result)!=len(rows):raise ValueError('Duplicate '+label+' IDs')
    return result
def compact_case(value):
    return {key:copy.deepcopy(value[key]) for key in ('angles_deg','collision_details','operation_failures')}
def collision_map(value):
    result={}
    for row in value['collision_details']:
        pair=tuple(sorted((row['a'],row['b'])));volume=row['material_intersection_mm3']
        if pair in result or not math.isfinite(volume) or volume<0:raise ValueError('Invalid collision pair/volume')
        result[pair]=volume
    return result
def operation_errors(value):
    # Compare numerical operation failures, not cache/timing metadata.
    rows=[dict(parts=sorted((r['a'],r['b'])),errors=r['errors']) for r in value['operation_failures']]
    return sorted(rows,key=lambda r:json.dumps(r,sort_keys=True))
def compare_case(first,second):
    aa=collision_map(first);bb=collision_map(second)
    delta=max((abs(aa[k]-bb[k]) for k in aa.keys() & bb.keys()),default=0.)
    old_errors=operation_errors(first);new_errors=operation_errors(second)
    angles_match=angle_key(first['angles_deg'])==angle_key(second['angles_deg'])
    return dict(angles_match=angles_match,original_pairs=len(aa),recomputed_pairs=len(bb),
        missing_pairs=sorted(aa.keys()-bb.keys()),extra_pairs=sorted(bb.keys()-aa.keys()),
        maximum_overlap_difference_mm3=delta,original_operation_errors=old_errors,
        recomputed_operation_errors=new_errors,operation_errors_reproduced=old_errors==new_errors,
        # A repeated Boolean failure is recorded but cannot support a successful replay.
        passed=angles_match and aa.keys()==bb.keys() and delta<=.02 and old_errors==new_errors and not new_errors)
def representative_picks(path,all_rows):
    """Select existing original samples only; never invent/interpolate a replay."""
    if not all_rows:raise ValueError('No original path samples')
    picks=[]
    for fraction in PATH_FRACTIONS:
        row=min(all_rows,key=lambda r:(abs(r['path_fraction']-fraction),r['path_fraction']))
        picks.append((dict(selection_role='path_fraction',requested_path_fraction=fraction),row))
    if path['id'] in STAGED_PATHS:
        if len(path['waypoints'])!=4:raise ValueError('Expected four authored waypoints on each staged thumb path')
        for index,waypoint in enumerate(path['waypoints']):
            boundary=[r for r in all_rows if r['segment']==max(0,index-1) and r['segment_fraction']==(0. if index==0 else 1.)]
            if len(boundary)!=1:raise ValueError('Authored thumb waypoint has no original boundary sample')
            # Full original arithmetic is reconciled before selection. Preserve
            # the recorded boundary angles, even if authored arithmetic differs
            # at its last bit; never re-round or substitute the authored array.
            picks.append((dict(selection_role='authored_waypoint',waypoint_index=index,
                authored_waypoint_fraction=waypoint['fraction']),boundary[0]))
    return picks
def prepare_transition_samples():
    """Validate the full original sampling record, retaining only selected cases."""
    original=read(inside(CORE_REPORT_REL));catalog=read(inside(CORE_CATALOG_REL));canonical=read(inside(CAD_REL+'/poses.json'))
    guarded=original_input_guard(original,'Original transition audit')
    if source_relative(original['poses_source'])!=CORE_CATALOG_REL or CORE_CATALOG_REL not in guarded:
        raise ValueError('Original transition audit did not read committed core_paths.json')
    if original.get('mode')!='full' or original['thresholds'].get('maximum_joint_step_deg')!=STEP_DEG:
        raise ValueError('Original audit must contain the declared full 2.5-degree forward sample set')
    for key,expected in [('material_overlap_mm3',.02),('boolean_tolerance_mm',1e-6)]:
        if original['thresholds'].get(key)!=expected:raise ValueError('Original collision threshold differs: '+key)
    if catalog['joint_names']!=canonical['joint_names'] or catalog['kinematic_design']!=canonical['kinematic_design'] or catalog['reference']!=canonical['reference']:
        raise ValueError('Core path kinematics/reference differs from canonical catalog')
    endpoints=unique_rows(catalog['gestures'],'core gesture');canon=unique_rows(canonical['gestures'],'canonical gesture')
    if set(endpoints)!=set(CORE_IDS):raise ValueError('Core path catalog must contain exactly three canonical endpoints')
    for name,row in endpoints.items():
        if angle_key(row['angles_deg'])!=angle_key(canon[name]['angles_deg']):raise ValueError('Core path endpoint differs from canonical: '+name)
    paths=unique_rows(catalog['transitions'],'core transition')
    original_paths=unique_rows(original['transitions'],'reported transition')
    if set(paths)!=set(FORWARD_IDS) or set(original_paths)!=set(FORWARD_IDS):raise ValueError('Expected exactly three forward paths')
    observations=unique_rows(original['observations'],'observation');cases=original['cases'];case_angles={}
    for cid,case in cases.items():
        key=angle_key(case['angles_deg'])
        if key in case_angles:raise ValueError('Duplicate original case angle key')
        case_angles[key]=cid
    visited=set();selected={};segment_counts={};reconstructed_counts={}
    def validate_observation(expected):
        row=observations.get(expected['id'])
        if row is None:raise ValueError('Missing reported sample: '+expected['id'])
        for name,want in expected.items():
            if name=='angles_deg':same=angle_key(row[name])==angle_key(want)
            elif name in ('segment_fraction','path_fraction'):same=float(row[name]).hex()==float(want).hex()
            else:same=row.get(name)==want
            if not same:raise ValueError('Reported segment/sample differs from committed catalog: '+expected['id']+' '+name)
        cid=row['case_id']
        if cid not in cases or angle_key(cases[cid]['angles_deg'])!=angle_key(row['angles_deg']):raise ValueError('Observation-to-case angle mismatch')
        visited.add(row['id']);return row
    for name,row in endpoints.items():
        recorded=validate_observation(dict(id='gesture:'+name,kind='endpoint',gesture=name,angles_deg=row['angles_deg']))
        if original['gesture_results'][name]['case_id']!=recorded['case_id']:raise ValueError('Endpoint case pointer differs')
    for path_id in FORWARD_IDS:
        path=paths[path_id];source,target=path_id.split('__')
        if path['source']!=source or path['target']!=target:raise ValueError('Forward path identity mismatch')
        waypoints=path['waypoints'];fractions=np.asarray([w['fraction'] for w in waypoints],float)
        for row in waypoints:angle_key(row['angles_deg'])
        q=np.asarray([w['angles_deg'] for w in waypoints],float)
        if len(waypoints)<2 or not np.all(np.isfinite(fractions)) or fractions[0]!=0 or fractions[-1]!=1 or np.any(np.diff(fractions)<=0):raise ValueError('Invalid segment fractions')
        if angle_key(q[0].tolist())!=angle_key(endpoints[source]['angles_deg']) or angle_key(q[-1].tolist())!=angle_key(endpoints[target]['angles_deg']):raise ValueError('Path endpoint does not equal canonical pose')
        all_rows=[];maximum=0.;groups=[]
        for segment in range(len(waypoints)-1):
            steps=max(1,int(math.ceil(float(np.max(np.abs(q[segment+1]-q[segment])))/STEP_DEG)));rows=[]
            for k in range(steps+1):
                if segment and not k:continue
                alpha=k/steps
                angles=q[segment].copy() if not k else q[segment+1].copy() if k==steps else q[segment]+alpha*(q[segment+1]-q[segment])
                expected=dict(id=f'transition:{path_id}:{len(all_rows):04d}',kind='transition',transition=path_id,
                    source=source,target=target,segment=segment,segment_fraction=alpha,
                    path_fraction=float(fractions[segment]+alpha*(fractions[segment+1]-fractions[segment])),
                    phase=waypoints[segment+1].get('phase'),angles_deg=angles.tolist())
                row=validate_observation(expected)
                if all_rows:maximum=max(maximum,float(np.max(np.abs(angles-np.asarray(all_rows[-1]['angles_deg'])))))
                rows.append(row);all_rows.append(row)
            groups.append(rows)
        declared=original_paths[path_id]
        expected_metadata=dict(source=source,target=target,waypoint_count=len(waypoints),sample_count=len(all_rows),observations=[r['id'] for r in all_rows])
        if any(declared.get(k)!=v for k,v in expected_metadata.items()) or abs(declared['maximum_actual_joint_step_deg']-maximum)>1e-12 or maximum>STEP_DEG+1e-9:
            raise ValueError('Transition sample summary differs from independently reconstructed catalog')
        segment_counts[path_id]=len(groups);reconstructed_counts[path_id]=len(all_rows)
        for reason,row in representative_picks(path,all_rows):
            key=angle_key(row['angles_deg'])
            if key not in selected:selected[key]=dict(id=f'transition_sample_{len(selected):04d}',angles_deg=copy.deepcopy(row['angles_deg']),angles_hex=list(key),references=[],expected=compact_case(cases[row['case_id']]))
            selected[key]['references'].append(dict(transition=path_id,segment=row['segment'],**reason,
                source_observation_id=row['id'],source_case_id=row['case_id'],segment_fraction=row['segment_fraction'],path_fraction=row['path_fraction']))
    if visited!=set(observations) or {row['case_id'] for row in observations.values()}!=set(cases):raise ValueError('Unaccounted original observations/cases')
    summary=original['summary']
    if summary.get('gesture_count')!=3 or summary.get('transition_count')!=3 or summary.get('observation_count')!=len(observations) or summary.get('unique_pose_count')!=len(cases) or summary.get('full_transition_scope_executed') is not True:
        raise ValueError('Original transition summary coverage differs')
    rows=list(selected.values());document=copy.deepcopy(catalog)
    document['gestures']=[dict(id=r['id'],label_ko='Selected transition sample '+str(i),angles_deg=r['angles_deg']) for i,r in enumerate(rows)]
    document['transitions']=[]
    document['study_scope']=dict(meaning='Selected transition samples exact replay, not a full 2.5-degree path rerun.',
        forward_paths=FORWARD_IDS,selection_policy=SELECTION_POLICY,full_transition_rerun=False,reverse_paths_executed=False)
    provenance=dict(catalog=CORE_CATALOG_REL,catalog_sha256=h(inside(CORE_CATALOG_REL)),original_report=CORE_REPORT_REL,
        original_report_sha256=h(inside(CORE_REPORT_REL)),canonical_catalog=CAD_REL+'/poses.json',canonical_catalog_sha256=h(inside(CAD_REL+'/poses.json')),
        forward_paths=FORWARD_IDS,segment_counts=segment_counts,original_sample_counts=reconstructed_counts,
        original_maximum_joint_step_deg=STEP_DEG,original_guarded_inputs_checked=len(guarded),
        selection_policy=SELECTION_POLICY,requested_path_fractions=PATH_FRACTIONS,
        authored_waypoint_paths=STAGED_PATHS,authored_waypoint_counts={name:len(paths[name]['waypoints']) for name in STAGED_PATHS},
        full_original_arithmetic_reconciled=True,original_total_transition_samples=sum(reconstructed_counts.values()),
        original_total_observations=len(observations),original_unique_cases=len(cases),
        selection='Nearest recorded path_fraction 0/.25/.5/.75/1 on each of three forward paths; ties choose the earlier fraction. Additionally every authored waypoint on the two staged thumb paths. Exact float.hex angle deduplication; no requirement to CAD-replay all 33 fist waypoints.',
        selected_transition_samples=len(rows),full_transition_rerun=False,reverse_paths_executed=False,
        scope='Selected transition samples exact replay; no continuous-clearance, full-path replay, dynamics or manufacturing approval.')
    return document,rows,provenance
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
    if state.get('id')!='GESTURE002' or state.get('candidate')!=CAD_REL or state.get('manifest')!='checkpoints/GESTURE002.manifest.json' or manifest.get('id')!='GESTURE002':raise ValueError('Checkpoint identity differs')
    for required in (CORE_CATALOG_REL,CORE_REPORT_REL,CAD_REL+'/poses.json'):
        if required not in manifest['files']:raise ValueError('Transition source/evidence omitted from checkpoint manifest: '+required)
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
    cad=repo/CAD_REL;expected_scene=read(cad/'generated/scene.json');expected_mass=read(cad/'generated/mass_inertia.json')
    expected_audit=read(cad/'reviews/gesture_endpoints.json')
    original_input_guard(expected_audit,'Baseline endpoint audit')
    selected_document,selected_samples,selection_provenance=prepare_transition_samples()
    report['transition_sample_selection']=selection_provenance
    report['selected_transition_samples']=len(selected_samples)
    selected_path=dest/'selected_transition_poses.json'
    selected_path.write_text(json.dumps(selected_document,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    report['selected_transition_document_sha256']=h(selected_path)
    del selected_document
    save()
    source_start={p.relative_to(repo).as_posix():h(p) for p in repo.rglob('*.py')}
    run('rebuild',[PY,'-B','cad/gesture_hand_v3/build.py','--stage','fabrication'],repo)
    rebuilt_scene=read(cad/'generated/scene.json');rebuilt_mass=read(cad/'generated/mass_inertia.json')
    old={r['name']:r for r in expected_scene['hand']};new={r['name']:r for r in rebuilt_scene['hand']}
    if len(old)!=len(expected_scene['hand']) or len(new)!=len(rebuilt_scene['hand']) or old.keys()!=new.keys():raise ValueError('Rebuilt part identities duplicate or differ')
    if rebuilt_scene.get('inputs_unchanged') is not True or read(cad/'generated/build_report.json').get('inputs_unchanged') is not True:raise ValueError('Rebuilt CAD input guard failed')
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
    poses=read(cad/'poses.json');ids=CORE_IDS;poses['gestures']=[p for p in poses['gestures'] if p['id'] in ids]
    if len(poses['gestures'])!=3:raise ValueError('Core pose selection missing')
    poses['transitions']=[];core=dest/'core_poses.json';core.write_text(json.dumps(poses,indent=2))
    run('core_collisions',[PY,'-B','cad/gesture_hand_v3/audit_gestures.py','--cad-root',str(cad),'--poses',str(core),'--mode','endpoints','--skip-proximity','--output',str(dest/'core_collisions.json')],repo)
    got=read(dest/'core_collisions.json');completed_guard(got,'Recomputed core endpoint audit');comparisons=[]
    for name in ids:
        first=expected_audit['cases'][expected_audit['gesture_results'][name]['case_id']]
        second=got['cases'][got['gesture_results'][name]['case_id']]
        comparisons.append(dict(gesture=name,**compare_case(first,second)))
    report['core_collision_reproduction']=comparisons
    del expected_audit,got
    run('core_transition_samples',[PY,'-B',CAD_REL+'/audit_gestures.py','--cad-root',str(cad),
        '--poses',str(selected_path),'--mode','endpoints','--skip-proximity','--output',str(dest/'core_transition_samples.json')],repo)
    got=read(dest/'core_transition_samples.json');completed_guard(got,'Recomputed selected transition samples')
    if got.get('mode')!='endpoints' or got.get('transitions') or got['summary'].get('full_transition_scope_executed') is not False:raise ValueError('Unexpected selected replay scope')
    if set(got['gesture_results'])!={r['id'] for r in selected_samples} or len(got['cases'])!=len(selected_samples):raise ValueError('Selected replay coverage differs')
    transition_comparisons=[]
    for sample in selected_samples:
        second=got['cases'][got['gesture_results'][sample['id']]['case_id']]
        comparison=dict(sample_id=sample['id'],angles_deg=sample['angles_deg'],angles_hex=sample['angles_hex'],
            source_references=sample['references'],**compare_case(sample['expected'],second))
        transition_comparisons.append(comparison)
    report['transition_sample_reproduction']=transition_comparisons
    report['transition_sample_reproduction_passed']=bool(transition_comparisons) and all(r['passed'] for r in transition_comparisons)
    if h(selected_path)!=report['selected_transition_document_sha256']:raise ValueError('Selected pose document changed')
    save();del got,selected_samples
    run('interfaces',[PY,'-B','cad/gesture_hand_v3/audit_fabrication.py','--output',str(dest/'interfaces.json')],repo)
    old_i=read(cad/'reviews/fabrication_interfaces.json');new_i=read(dest/'interfaces.json')
    completed_guard(old_i,'Original fabrication interface audit');completed_guard(new_i,'Recomputed fabrication interface audit')
    old_outcomes=interface_outcomes(old_i);new_outcomes=interface_outcomes(new_i)
    report['interface_individual_checks_compared']=len(old_outcomes)
    report['interface_changed_checks']=[k for k in sorted(old_outcomes.keys()|new_outcomes.keys()) if old_outcomes.get(k)!=new_outcomes.get(k)]
    report['interface_summary_reproduced']=old_i.get('summary')==new_i.get('summary') and not report['interface_changed_checks']
    report['interface_original_summary']=old_i.get('summary');report['interface_recomputed_summary']=new_i.get('summary')
    report['source_hash_guard']=dict(start=source_start,end={p.relative_to(repo).as_posix():h(p) for p in repo.rglob('*.py')})
    report['source_hash_guard']['passed']=report['source_hash_guard']['start']==report['source_hash_guard']['end']
    report['reviewer_source_unchanged']=h(__file__)==report['reviewer_sha256']
    report['completed']=True
    report['reproduction_passed']=bool(all(x['passed'] for x in geometry+masses+comparisons) and report['transition_sample_reproduction_passed'] and report['interface_summary_reproduced'] and report['source_hash_guard']['passed'] and report['reviewer_source_unchanged'])
    report['scope']='Exact committed archive and manifest/frozen evidence; rebuilt all CAD/mass tensors; reran open/fist/thumb-up exact collisions, representative transition samples exact replay, and nominal fabrication interfaces. Original full 2.5-degree sampling arithmetic and angle correspondence were reconciled in full. CAD selection is nearest path fractions 0/.25/.5/.75/1 on each of three forward paths, plus every authored waypoint on the two staged thumb paths, with exact-angle deduplication. All 33 fist waypoints and the complete 2.5-degree paths are not CAD-replayed. Reverse directions, other six endpoints, proximity distances, renderer and dynamics were not rerun. Existing geometric failures must reproduce; a Boolean operation failure cannot pass reproduction. This is same-task reproduction, not a manufacturing release or external independent audit.'
except Exception as exc:report['errors'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
save();print(json.dumps(dict(completed=report['completed'],reproduction_passed=report['reproduction_passed'],errors=report['errors'])),flush=True)
raise SystemExit(0 if report['reproduction_passed'] else 1)

"""Shared read-only JSON/byte contracts for the manufacturing checkpoint helpers."""
from pathlib import Path
import hashlib, json, math

BASE = 'acda2839ea8e32441b78f2ec66d511c2ab505c76'
CAD = 'cad/manufacturing_v2'
MANIFEST = CAD + '/manifest.json'
AUDIT = CAD + '/reviews/assembly_audit.json'
RENDER = CAD + '/generated/render_report.json'
DOCS = {'README.md', 'AGENTS.md', 'docs/REMOTE_DRIVE_PLAN.md'}
GIT = 'C:/Program Files/Git/cmd/git.exe'
BATCH_SHA = '28e37b501bac04630ffb953bfe6ccc1cb81674830e475834c539b0c8abd5f386'
REMOTE = {'https://github.com/ckh7488/robot_hand', 'https://github.com/ckh7488/robot_hand.git', 'git@github.com:ckh7488/robot_hand.git'}
SCOPE = ('Exact committed input closure; reread all 359 saved native STEP parts and 10 pilot-only machining blanks. '
         'Replay 10 retained-pin sets, five access scopes per set, 20 oversized-tool positive controls, '
         '10 static poses and all 48 forward-path observations at declared maximum joint step 5 degrees. '
         'Collision scope is 309 other parts plus 10 five-part freedom envelopes (319 entities covering 359 parts). '
         'No CAD rebuild, render replay, continuous hand-motion, proximity, true helical installation, '
         'tolerance, strength, preload, loosening, drive, latency or physical manufacturing approval. '
         'Whole-hand access obstructions remain blockers; reproduction is not release.')
ACCESS_KEYS = ('pin_insertion', 'keeper_insertion', 'screw_outer_approach', 'hex_driver_approach', 'flat_driver_approach')
DIGITS = ('index', 'middle', 'ring', 'little', 'thumb')

def require(value, message):
    if not value: raise ValueError(message)

def read(path): return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''): value.update(block)
    return value.hexdigest()

def finite(value): return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

def angles(values):
    require(isinstance(values, list) and len(values) == 20 and all(finite(v) for v in values), 'Twenty finite angles required')
    return tuple(float(v).hex() for v in values)

def safe_file(repo, name, path_check):
    name = path_check(name); path = repo / name
    require(path.resolve() == path and path.is_file(), 'Missing or aliased input: ' + name)
    return path

def manifest_contract(manifest, path_check):
    require(manifest.get('id') == 'manufacturing_v2' and manifest.get('baseline_commit') == BASE, 'Manifest identity/base differs')
    require(manifest.get('audit_report') == AUDIT and manifest.get('render_report') == RENDER, 'Unexpected report paths')
    own = manifest.get('candidate_files', [])
    require(isinstance(own, list) and len(own) == len(set(own)) and own, 'Missing/duplicate candidate files')
    for name in own: require(path_check(name).startswith(CAD + '/'), 'Non-candidate own file: ' + name)
    files = manifest.get('files', {})
    require(isinstance(files, dict) and bool(files), 'Empty manifest files')
    for name, row in files.items():
        path_check(name)
        require(name not in {MANIFEST, 'CHECKPOINT.json', 'PROGRESS.md'}, 'Excluded manifest entry: ' + name)
        require(isinstance(row, dict) and isinstance(row.get('bytes'), int) and row['bytes'] >= 0 and
                isinstance(row.get('sha256'), str) and len(row['sha256']) == 64 and all(c in '0123456789abcdef' for c in row['sha256']), 'Invalid file signature: ' + name)
    require(set(own) == {n for n in files if n.startswith(CAD + '/')}, 'Candidate list/closure differs')
    required = {CAD + '/' + n for n in ('build.py', 'design_patch.json', 'audit_assembly.py', 'render.py', 'README.md',
        'generated/design.json', 'generated/scene.json', 'generated/build_report.json', 'generated/mass_inertia.json',
        'generated/hand.step', 'generated/whole_hand.png', 'generated/retention_assembly.png', 'tool_gauges.json',
        'RETENTION_DECISION.md', 'source_docs/required_sources.json')} | {AUDIT, RENDER}
    require(required <= set(own) and DOCS <= set(files), 'Required candidate/doc files omitted')
    blockers = manifest.get('remaining_blockers')
    require(isinstance(blockers, list) and blockers and all(isinstance(x, str) and x.strip() for x in blockers), 'Explicit remaining manufacturing limitations required')
    return set(own)

def report_guard(report, origin, manifest, repo, path_check):
    guard = report.get('input_hash_guard', {})
    require(report.get('completed') is True and not report.get('errors') and guard.get('passed') is True and
            bool(guard.get('start')) and guard['start'] == guard.get('end') == report.get('source_sha256'), 'Incomplete audit or input hash guard')
    result = {}
    for name, expected in guard['start'].items():
        path = Path(name)
        if path.is_absolute():
            require(path.is_relative_to(origin), 'Report input outside recorded checkout')
            name = path.relative_to(origin).as_posix()
        path_check(name)
        require(name not in result and name in manifest['files'], 'Input omitted/duplicated in closure: ' + name)
        require(manifest['files'][name]['sha256'] == expected == digest(safe_file(repo, name, path_check)), 'Input hash differs: ' + name)
        result[name] = expected
    return result

def insertion_map(scope):
    result = {}
    for row in scope['checked']:
        name, volume = row['part'], row['overlap_mm3']
        require(name not in result and finite(volume) and volume >= 0, 'Invalid insertion pair')
        require(row['blocked'] == (volume > 1e-4), 'Incorrect insertion classification')
        result[name] = volume
    require(scope['broadphase_candidates'] == len(result), 'Insertion candidate count differs')
    require(scope['blocked_parts'] == [r for r in scope['checked'] if r['blocked']], 'Insertion blockers differ')
    require(scope['passed'] == (not scope['blocked_parts']), 'Insertion passed flag differs')
    return result

def collision_map(case):
    result = {}
    for row in case['collision_details']:
        key = tuple(sorted((row['a'], row['b']))); volume = row['material_intersection_mm3']
        require(key not in result and finite(volume) and volume > .02, 'Invalid pose collision row')
        result[key] = volume
    require(case['collision_count'] == len(result), 'Pose collision count differs')
    return result

def near_zero(value, tolerance=1e-5):
    return finite(value) and abs(value) <= tolerance

def native_part_comparisons(old, new):
    previous = {p['name']: p for p in old['parts']}
    require(set(previous) == {p['name'] for p in new['parts']}, 'Replayed native part set differs')
    rows = []
    for part in new['parts']:
        before = previous[part['name']]; delta = abs(before['volume_mm3']-part['volume_mm3'])
        rows.append(dict(name=part['name'], maximum_volume_difference_mm3=delta,
            passed=finite(delta) and delta <= 1e-5 and all(before[k] == part[k] for k in ('body','role','valid','solid_count','mapping_passed'))))
    return rows

def audit_contract(report):
    require(report.get('schema') == 'manufacturing_assembly_audit.v2' and report.get('completed') is True and not report.get('errors'), 'Incomplete/wrong audit schema')
    parts=report['parts']; retained=report['retention']; gestures=report['gestures']; observations=report['observations']; cases=report['cases']
    names={p['name'] for p in parts}
    byname={p['name']:p for p in parts}
    require(len(parts)==359 and len(names)==359 and all(p['valid'] is True and p['solid_count']==1 and p['mapping_passed'] is True and finite(p['volume_mm3']) and p['volume_mm3']>0 for p in parts), 'Incomplete/invalid native parts')
    require(report.get('assembly_roundtrip',{}).get('passed') is True, 'Assembly STEP mismatch')
    modified={f'{d}_joint_pin_{j}' for d in DIGITS for j in (1,2)}
    ids={f'{d}_distal{j}' for d in DIGITS for j in (1,2)}
    removed={n+'_retaining_clip' for n in ids}; added={n+'_retaining_'+k for n in ids for k in ('washer','screw')}
    contract=report['part_contract']; unchanged=set(contract['unchanged'])
    require(contract['baseline_count']==349 and contract['candidate_count']==359 and set(contract['modified'])==modified and
            set(contract['removed'])==removed and set(contract['added'])==added and len(contract['unchanged'])==len(unchanged)==329 and
            names==unchanged|modified|added and not unchanged&(modified|added) and not names&removed, 'Wrong part change partition')
    require(len(retained)==10 and {p['id'] for p in retained}==ids, 'Incomplete retained-pin coverage')
    require(report['thresholds']==dict(boolean_tolerance_mm=1e-6,access_overlap_mm3=1e-4,pose_overlap_mm3=.02,maximum_joint_step_deg=5.), 'Changed audit thresholds')
    require(report['tool_gauge_definition']['approach_travel_mm']==30 and report['tool_gauge_definition']['nominal_axial_float_mm']==.15, 'Changed declared approach/endplay')
    all_members=set(); detected=0
    for row in retained:
        ident=row['id']; digit,j=ident.split('_distal'); pin=f'{digit}_joint_pin_{j}'; washer=ident+'_retaining_washer'; screw=ident+'_retaining_screw'
        members={pin,washer,screw,ident+'_left_pin_washer',ident+'_right_pin_washer'}
        require(len(row['members'])==5 and set(row['members'])==members and not all_members&members, 'Incomplete/duplicate five-part cluster')
        require(all(byname[n]['body']==row['parent_body_placement'] for n in members), 'Cluster body placement mismatch')
        all_members|=members
        require(set(row['reconstruction_difference_mm3'])==members and all(len(a)==2 and all(near_zero(x) for x in a) for a in row['reconstruction_difference_mm3'].values()), 'Cluster reconstruction failed')
        blank=row['machining_blank']
        require(blank['path']==CAD+'/generated/machining_blanks/'+pin+'_pilot_blank.step' and blank['finished_part'] is False and
                len(blank['reconstruction_difference_mm3'])==2 and all(near_zero(x) for x in blank['reconstruction_difference_mm3']) and
                near_zero(blank['assembly_material_outside_blank_mm3']) and finite(blank['material_removed_by_simplified_thread_envelope_mm3']) and blank['material_removed_by_simplified_thread_envelope_mm3']>0, 'Invalid pilot-only blank comparison')
        expected_pairs={tuple(sorted((a,b))) for a in members for b in members if a<b}
        internal=row['nominal_internal_intersections']
        require(len(internal)==10 and {tuple(sorted((r['a'],r['b']))) for r in internal}==expected_pairs and
                all(finite(r['overlap_mm3']) and 0<=r['overlap_mm3']<=1e-4 and r['passed'] is True for r in internal), 'Nominal cluster internal collision/missing pair')
        proxy=row['cluster_proxy']; contains=proxy['contains_original_outside_volume_mm3']
        require(proxy['name']==ident+'_retention_freedom_envelope' and set(contains)==members and all(near_zero(x) for x in contains.values()) and
                proxy['rotation_deg']==[0,360] and proxy['axial_float_mm']==[-.15,0] and proxy['internal_intersections_checked_separately'] is True, 'Invalid freedom envelope containment')
        # The correlated washer-float schema is checked explicitly below, never inferred from a completed flag.
        validate_correlated_freedom(row, members)
        require(set(row['access'])==set(ACCESS_KEYS), 'Missing/extra access case')
        exclusions={'pin_insertion':{pin,washer,screw},'keeper_insertion':{washer,screw},'screw_outer_approach':{screw},'hex_driver_approach':set(),'flat_driver_approach':set()}
        for key,scope in row['access'].items():
            require(set(scope['excluded_parts'])==exclusions[key] and len(scope['excluded_parts'])==len(exclusions[key]), 'Access excluded wrong material')
            local=insertion_map(scope['finger_module']); whole=insertion_map(scope['whole_hand_neutral'])
            require(set(local)=={n for n in whole if n.startswith(digit+'_')} and
                    all(abs(local[n]-whole[n])<=1e-5 and (local[n]>1e-4)==(whole[n]>1e-4) for n in local), 'Finger access differs from whole-hand subset')
            require(not set(whole)&exclusions[key] and set(whole)<=names, 'Access obstacle scope invalid')
        for key,target in (('hex_oversize_tool_control',screw),('flat_oversize_tool_control',pin)):
            control=row[key]; measured=insertion_map(control)
            require(set(measured)=={target} and measured[target]>1e-4 and control['passed'] is False, 'Missing oversized-tool positive control')
            detected+=1
        require(near_zero(row['simultaneous_installed_tools_overlap_mm3'],1e-4), 'Installed tools conflict')
    partition=report['proxy_partition']
    require(partition['physical_part_count']==359 and partition['retention_clusters']==10 and partition['parts_per_cluster']==5 and
            partition['unchanged_part_files']==329 and partition['noncluster_physical_parts']==309 and
            partition['collision_entity_count']==319 and len(names-all_members)==309 and partition['no_material_excluded'] is True and
            len(partition['loose_washer_names'])==20 and set(partition['loose_washer_names'])=={n+'_'+side+'_pin_washer' for n in ids for side in ('left','right')} and
            len(partition['covered_names'])==359 and set(partition['covered_names'])==names, 'Wrong full-material proxy partition')
    expected_gestures={'open','fist','v_sign','scissors','thumb_up','middle_finger','mountain','rock_thumb_folded','rock_thumb_extended','fingers_folded'}
    require(len(gestures)==10 and {g['id'] for g in gestures}==expected_gestures and len(observations)==58 and len({o['id'] for o in observations})==58, 'Incomplete static/path observations')
    endpoints=[o for o in observations if o['kind']=='endpoint']; path=[o for o in observations if o['kind']=='transition']
    require(len(endpoints)==10 and len(path)==48 and {o['gesture_id'] for o in endpoints}=={g['id'] for g in gestures}, 'Wrong static/forward-path count')
    gesture_angles={g['id']:angles(g['angles_deg']) for g in gestures}; keys={}; used=set()
    for obs in observations:
        key=angles(obs['angles_deg']); case_id=obs['case_id']; require(case_id in cases, 'Missing observed case')
        require(angles(cases[case_id]['angles_deg'])==key, 'Observed/case angle mismatch')
        if key in keys: require(keys[key]==case_id, 'Duplicate angle has different case')
        else: require(case_id not in used, 'Different angles share case'); keys[key]=case_id; used.add(case_id)
        if obs['kind']=='endpoint': require(key==gesture_angles[obs['gesture_id']], 'Static gesture differs')
    require(used==set(cases), 'Unreferenced collision cases')
    for case in cases.values():
        require(case['total_pairs_in_scope']==319*318//2 and case.get('clearance_reported') is False, 'Wrong all-material/proximity scope')
        collision_map(case)
    definitions=report['full_path_definition']
    require(len(definitions)==1 and definitions[0]['observations']==[o['id'] for o in path] and definitions[0]['sample_count']==48, 'Forward-path definition coverage differs')
    maximum=max(max(abs(float(a)-float(b)) for a,b in zip(left['angles_deg'],right['angles_deg'])) for left,right in zip(path,path[1:]))
    require(finite(maximum) and maximum<=5.+1e-9 and abs(maximum-definitions[0]['maximum_actual_joint_step_deg'])<=1e-12, 'Path spacing exceeds declared scope')
    derived=dict(physical_parts=359,collision_entities=319,modified_parts=10,removed_parts=10,added_parts=20,unchanged_parts=329,retention_sets=10,
        finger_module_blocked={k:sum(not r['access'][k]['finger_module']['passed'] for r in retained) for k in ACCESS_KEYS},
        whole_hand_neutral_blocked={k:sum(not r['access'][k]['whole_hand_neutral']['passed'] for r in retained) for k in ACCESS_KEYS},
        oversize_tool_controls_detected=detected,static_pose_count=10,observation_count=58,unique_pose_count=len(cases),sampled_hand_path_count=48,
        maximum_actual_joint_step_deg=maximum,pose_collision_count=sum(c['collision_count'] for c in cases.values()),operation_failure_count=sum(len(c['operation_failures']) for c in cases.values()))
    require(report['summary']==derived, 'Summary differs from raw results')
    require(not any(derived['finger_module_blocked'].values()) and derived['pose_collision_count']==0 and derived['operation_failure_count']==0, 'Own-finger access or static/path collisions/errors remain')
    return derived

def validate_correlated_freedom(row, members):
    freedom=row['axial_freedom']; proxy=row['cluster_proxy']; stack=row['nominal_stack']
    require(freedom['coaxial_only'] is True and freedom['domain']=='-d<=t<=0; t<=s_left<=0; 0<=s_right<=d+t; d=0.15mm' and
            freedom['not_modelled']==['radial washer eccentricity','washer tilt','dimensional tolerances'] and
            isinstance(freedom['continuous_internal_separation'],str) and freedom['continuous_internal_separation'].strip(), 'Axial freedom assumptions omitted/changed')
    require(proxy['left_washer_shift_mm']==[-.15,0] and proxy['right_washer_shift_mm']==[0,.15] and stack['axial_float_mm']==[-.15,0] and
            abs(stack['left_face_x_mm']+13)<=1e-5 and abs(stack['right_face_x_mm']-13)<=1e-5, 'Washer endplay/stack differs')
    expected={(-.15,-.15,0.),(-.15,0.,0.),(0.,0.,0.),(0.,0.,.15)}
    vertices=freedom['vertex_checks']; pairs={tuple(sorted((a,b))) for a in members for b in members if a<b}
    require(len(vertices)==4 and {(r['pin_shift_mm'],r['left_washer_shift_mm'],r['right_washer_shift_mm']) for r in vertices}==expected, 'Missing correlated freedom vertex')
    for vertex in vertices:
        checks=vertex['pairs']
        require(len(checks)==10 and {tuple(sorted((r['a'],r['b']))) for r in checks}==pairs and
                all(finite(r['overlap_mm3']) and 0<=r['overlap_mm3']<=1e-4 for r in checks), 'Correlated freedom internal collision/missing pair')

def package_contract(repo, manifest, path_check, origin):
    """Every declared calculation input and output must belong to the file closure."""
    def bind(name, expected):
        path = Path(name)
        if path.is_absolute():
            require(path.is_relative_to(origin), 'Absolute source outside recorded repo')
            name = path.relative_to(origin).as_posix()
        require(name in manifest['files'] and manifest['files'][name]['sha256'] == expected == digest(safe_file(repo, name, path_check)), 'Unbound generated evidence: ' + name)
    def sources(value):
        if isinstance(value, dict):
            for key, entry in value.items():
                if key == 'source_sha256':
                    require(isinstance(entry, dict) and entry, 'Empty source hash declaration')
                    for n, h in entry.items(): bind(n, h)
                elif key == 'input_hash_guard':
                    require(entry.get('passed') is True and entry.get('start') and entry['start'] == entry.get('end'), 'Nested input guard failed')
                    for n, h in entry['start'].items(): bind(n, h)
                else: sources(entry)
        elif isinstance(value, list):
            for entry in value: sources(entry)
    build = read(repo / CAD / 'generated/build_report.json'); scene = read(repo / CAD / 'generated/scene.json')
    mass = read(repo / CAD / 'generated/mass_inertia.json'); audit = read(repo / AUDIT); render = read(repo / RENDER)
    interfaces=build['retention_interfaces']
    require(len(interfaces)==10 and all(r['female_full_thread_span_x_mm']==[8.75,12.75] and r['nominal_full_thread_overlap_mm']==2.8 and
            r['thread_engagement_strength_approved'] is False for r in interfaces), 'Thread callout/strength scope differs from corrected 4.0 mm nominal full form and 2.8 mm overlap')
    require(build.get('completed') is True and build.get('inputs_unchanged') is True and not build.get('errors'), 'Incomplete build')
    require(scene.get('inputs_unchanged') is True and mass.get('inputs_unchanged') is True and scene['source_sha256'] == mass['source_sha256'] == build['source_sha256'], 'Build/scene/mass guard differs')
    for value in (build, scene, mass, audit, render): sources(value)
    bind(CAD + '/' + build['assembly_step'], build['assembly_step_sha256'])
    bind(CAD + '/' + build['generated_design'], build['generated_design_sha256'])
    require(len(scene['hand']) == 359 and build['part_count']==359 and build['unchanged_part_count']==329 and
            build['modified_part_count']==10 and build['removed_part_count']==10 and build['added_part_count']==20, 'Wrong scene/build count')
    for row in scene['hand']:
        for path_key, hash_key in (('step', 'sha256_step'), ('mesh', 'sha256_stl')):
            resolved = (repo / CAD / row[path_key]).resolve()
            require(resolved.is_relative_to(repo), 'Scene escapes repo')
            bind(resolved.relative_to(repo).as_posix(), row[hash_key])
    require(len(build['machining_blanks'])==10 and {r['finished_part'] for r in build['machining_blanks']}=={f'{d}_joint_pin_{j}' for d in DIGITS for j in (1,2)}, 'Missing pilot-only blanks')
    for blank in build['machining_blanks']:
        require(blank['assembly_part'] is False and blank['included_in_mass'] is False and blank['finished_thread_geometry'] is False, 'Pilot blank misclassified')
        bind(CAD+'/'+blank['step'],blank['sha256_step'])
    require(render.get('completed') is True and render.get('status') == 'completed' and render.get('inputs_unchanged') is True and not render.get('errors') and not render.get('error'), 'Incomplete renderer')
    require(set(render['output_sha256']) == {'whole_hand.png', 'retention_assembly.png'}, 'Unexpected rendered outputs')
    for name, sha in render['output_sha256'].items(): bind(CAD + '/generated/' + name, sha)
    source = report_guard(audit, origin, manifest, repo, path_check)
    require({CAD + '/audit_assembly.py', CAD + '/build.py', CAD + '/design_patch.json', CAD+'/tool_gauges.json'} <= set(source), 'Audit omitted candidate sources')
    audit_contract(audit)
    return audit, source

def frozen_evidence(index):
    result = {}
    for key in ('geometry_sha256', 'code_sha256', 'report_sha256'):
        for name, sha in index[key].items():
            require(name not in result or result[name] == sha, 'Conflicting frozen evidence')
            result[name] = sha
    require(len(result) == 222, 'Expected frozen 222 evidence files')
    return result

def compare_audits(old, new):
    """Ignore timing/cache scheduling; compare exact scope and classified raw outcomes."""
    for key in ('summary','gestures','thresholds','part_contract','proxy_partition','observations','full_path_definition','tool_gauge_definition'):
        require(old[key]==new[key], 'Replayed scope/observations differ: '+key)
    rows=[]; previous={r['id']:r for r in old['retention']}
    def numeric_equal(a,b,path=''):
        if isinstance(a,dict): return isinstance(b,dict) and a.keys()==b.keys() and all(numeric_equal(a[k],b[k],path+'/'+k) for k in a)
        if isinstance(a,list): return isinstance(b,list) and len(a)==len(b) and all(numeric_equal(x,y,path) for x,y in zip(a,b))
        if finite(a) and finite(b) and ('mm3' in path): return abs(a-b)<=1e-5
        return type(a)==type(b) and a==b or finite(a) and finite(b) and a==b
    for cluster in new['retention']:
        before=previous[cluster['id']]
        rows.append(dict(kind='retention_geometry_and_freedom',id=cluster['id'],maximum_volume_difference_limit_mm3=1e-5,
            passed=numeric_equal({k:v for k,v in before.items() if k!='access'},{k:v for k,v in cluster.items() if k!='access'})))
        for key in ACCESS_KEYS:
            for scope in ('finger_module','whole_hand_neutral'):
                aa,bb=insertion_map(before['access'][key][scope]),insertion_map(cluster['access'][key][scope])
                delta=max((abs(aa[n]-bb[n]) for n in aa.keys()&bb.keys()),default=0.)
                rows.append(dict(kind='access',id=cluster['id'],approach=key,scope=scope,maximum_volume_difference_mm3=delta,
                    missing_parts=sorted(aa.keys()-bb.keys()),extra_parts=sorted(bb.keys()-aa.keys()),
                    passed=aa.keys()==bb.keys() and delta<=1e-5 and before['access'][key]['excluded_parts']==cluster['access'][key]['excluded_parts'] and
                    before['access'][key][scope]['passed']==cluster['access'][key][scope]['passed'] and
                    {n for n,v in aa.items() if v>1e-4}=={n for n,v in bb.items() if v>1e-4}))
    require(old['cases'].keys()==new['cases'].keys(), 'Replayed case identity differs')
    for name, case in new['cases'].items():
        previous = old['cases'][name]; aa, bb = collision_map(previous), collision_map(case)
        delta = max((abs(aa[n]-bb[n]) for n in aa.keys() & bb.keys()), default=0.)
        rows.append(dict(kind='observed_pose', id=name, angles_hex=angles(case['angles_deg']), maximum_volume_difference_mm3=delta,
            missing_pairs=sorted(aa.keys()-bb.keys()), extra_pairs=sorted(bb.keys()-aa.keys()),
            passed=angles(previous['angles_deg']) == angles(case['angles_deg']) and aa.keys() == bb.keys() and delta <= 1e-5 and
                not previous['operation_failures'] and not case['operation_failures']))
    return rows

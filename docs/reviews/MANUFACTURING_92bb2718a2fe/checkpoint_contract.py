"""Shared read-only JSON/byte contracts for the manufacturing checkpoint helpers."""
from pathlib import Path
import hashlib, json, math

BASE = 'dcdd20ca5ce7acf069d3b5108d215f262588f80a'
CAD = 'cad/manufacturing_v1'
MANIFEST = CAD + '/manifest.json'
AUDIT = CAD + '/reviews/assembly_audit.json'
RENDER = CAD + '/generated/render_report.json'
DOCS = {'README.md', 'AGENTS.md', 'docs/REMOTE_DRIVE_PLAN.md'}
GIT = 'C:/Program Files/Git/cmd/git.exe'
BATCH_SHA = '28e37b501bac04630ffb953bfe6ccc1cb81674830e475834c539b0c8abd5f386'
REMOTE = {'https://github.com/ckh7488/robot_hand', 'https://github.com/ckh7488/robot_hand.git', 'git@github.com:ckh7488/robot_hand.git'}
SCOPE = ('Exact committed input closure; reread all 349 saved native STEP parts and replay the complete '
         '10-pin nominal 30 mm insertion/clip-envelope audit and 10 static endpoint poses. '
         'No CAD rebuild, render replay, continuous hand-motion, proximity, elastic clip installation, '
         'tolerance, strength, drive, latency or physical manufacturing approval. '
         'A reproduced whole-hand insertion obstruction remains an obstruction; reproduction is not release.')

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
    require(manifest.get('id') == 'manufacturing_v1' and manifest.get('baseline_commit') == BASE, 'Manifest identity/base differs')
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
        'generated/hand.step', 'generated/whole_hand.png', 'generated/pin_assembly.png')} | {AUDIT, RENDER}
    require(required <= set(own) and DOCS <= set(files), 'Required candidate/doc files omitted')
    blockers = manifest.get('remaining_blockers')
    require(isinstance(blockers, list) and blockers and all(isinstance(x, str) and x.strip() for x in blockers), 'Explicit remaining manufacturing limitations required')
    return set(own)

def report_guard(report, origin, manifest, repo, path_check):
    guard = report.get('input_hash_guard', {})
    require(report.get('completed') is True and not report.get('errors') and guard.get('passed') is True and
            bool(guard.get('start')) and guard['start'] == guard.get('end'), 'Incomplete audit or input hash guard')
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

def audit_contract(report):
    parts = report['parts']; pins = report['pin_insertion']; clips = report['clip_envelopes']; poses = report['pose_cases']; gestures = report['gestures']
    require(len(parts) == 349 and len({p['name'] for p in parts}) == 349 and all(p['valid'] and p['solid_count'] == 1 and p['mapping_passed'] for p in parts), 'Incomplete/invalid native parts')
    require(report.get('assembly_roundtrip', {}).get('passed') is True, 'Assembly STEP mismatch')
    expected = {f'{digit}_distal{j}' for digit in ('index', 'middle', 'ring', 'little', 'thumb') for j in (1, 2)}
    require(len(pins) == 10 and {p['id'] for p in pins} == expected and len({p['moving_pin'] for p in pins}) == 10, 'Incomplete pin coverage')
    require(len(clips) == 10 and {p['id'] for p in clips} == expected and len({p['part'] for p in clips}) == 10, 'Incomplete clip coverage')
    require(len(gestures) == 10 and len(poses) == 10 and {g['id'] for g in gestures} == set(poses), 'Incomplete static pose coverage')
    require(report['thresholds'] == dict(boolean_tolerance_mm=1e-6, insertion_overlap_mm3=1e-4, pose_overlap_mm3=.02), 'Changed audit thresholds')
    for pin in pins:
        require(pin['travel_mm'] == 30 and finite(pin['original_pin_through_original_left_bush_overlap_mm3']) and pin['original_pin_through_original_left_bush_overlap_mm3'] > 1e-4, 'Missing original obstruction control')
        require(len(pin['pin_reconstruction_difference_mm3']) == 2 and all(finite(x) and 0 <= x <= 1e-5 for x in pin['pin_reconstruction_difference_mm3']), 'Saved pin reconstruction failed')
        for key in ('new_pin_through_new_left_bush', 'joint_subassembly', 'whole_hand_neutral'): insertion_map(pin[key])
    for clip in clips:
        require(clip['outer_radius_mm'] == 3.5 and clip['inner_radius_mm'] == 1.65 and clip['axial_span_mm'] == [13.15, 13.85] and finite(clip['original_outside_envelope_mm3']) and 0 <= clip['original_outside_envelope_mm3'] <= 1e-5, 'Invalid clip envelope')
    for g in gestures:
        case = poses[g['id']]; require(angles(g['angles_deg']) == angles(case['angles_deg']), 'Pose/case angle mismatch')
        require(case['total_pairs_in_scope'] == 349 * 348 // 2 and case.get('clearance_reported') is False, 'Wrong all-part/proximity scope')
        collision_map(case)
    derived = dict(parts=349, modified_parts=20, pin_insertions=10,
        left_bushing_paths_blocked=sum(not x['new_pin_through_new_left_bush']['passed'] for x in pins),
        joint_subassembly_paths_blocked=sum(not x['joint_subassembly']['passed'] for x in pins),
        whole_hand_neutral_paths_blocked=sum(not x['whole_hand_neutral']['passed'] for x in pins),
        original_obstructions_reproduced=10, clip_rotation_envelopes=10, static_pose_count=10,
        pose_collision_count=sum(x['collision_count'] for x in poses.values()),
        operation_failure_count=sum(len(x['operation_failures']) for x in poses.values()))
    require(report['summary'] == derived, 'Summary differs from raw results')
    require(derived['operation_failure_count'] == 0 and derived['pose_collision_count'] == 0, 'Static collision/error remains; this checkpoint contract expects zero static collisions')
    return derived

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
    require(build.get('completed') is True and build.get('inputs_unchanged') is True and not build.get('errors'), 'Incomplete build')
    require(scene.get('inputs_unchanged') is True and mass.get('inputs_unchanged') is True and scene['source_sha256'] == mass['source_sha256'] == build['source_sha256'], 'Build/scene/mass guard differs')
    for value in (build, scene, mass, audit, render): sources(value)
    bind(CAD + '/' + build['assembly_step'], build['assembly_step_sha256'])
    bind(CAD + '/' + build['generated_design'], build['generated_design_sha256'])
    require(len(scene['hand']) == 349, 'Wrong scene count')
    for row in scene['hand']:
        for path_key, hash_key in (('step', 'sha256_step'), ('mesh', 'sha256_stl')):
            resolved = (repo / CAD / row[path_key]).resolve()
            require(resolved.is_relative_to(repo), 'Scene escapes repo')
            bind(resolved.relative_to(repo).as_posix(), row[hash_key])
    require(render.get('completed') is True and render.get('status') == 'completed' and render.get('inputs_unchanged') is True and not render.get('errors') and not render.get('error'), 'Incomplete renderer')
    require(set(render['output_sha256']) == {'whole_hand.png', 'pin_assembly.png'}, 'Unexpected rendered outputs')
    for name, sha in render['output_sha256'].items(): bind(CAD + '/generated/' + name, sha)
    source = report_guard(audit, origin, manifest, repo, path_check)
    require({CAD + '/audit_assembly.py', CAD + '/build.py', CAD + '/design_patch.json'} <= set(source), 'Audit omitted candidate sources')
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
    require(old['summary'] == new['summary'] and old['gestures'] == new['gestures'] and old['thresholds'] == new['thresholds'], 'Summary/pose definitions differ')
    rows = []; oldpins = {p['id']: p for p in old['pin_insertion']}
    for pin in new['pin_insertion']:
        previous = oldpins[pin['id']]
        for key in ('new_pin_through_new_left_bush', 'joint_subassembly', 'whole_hand_neutral'):
            aa, bb = insertion_map(previous[key]), insertion_map(pin[key])
            delta = max((abs(aa[n]-bb[n]) for n in aa.keys() & bb.keys()), default=0.)
            rows.append(dict(kind='pin_insertion', id=pin['id'], scope=key, maximum_volume_difference_mm3=delta,
                missing_parts=sorted(aa.keys()-bb.keys()), extra_parts=sorted(bb.keys()-aa.keys()),
                passed=aa.keys() == bb.keys() and delta <= 1e-5 and previous[key]['passed'] == pin[key]['passed'] and
                    {r['part'] for r in previous[key]['blocked_parts']} == {r['part'] for r in pin[key]['blocked_parts']}))
        control_delta = abs(previous['original_pin_through_original_left_bush_overlap_mm3']-pin['original_pin_through_original_left_bush_overlap_mm3'])
        rows.append(dict(kind='original_obstruction_control', id=pin['id'], maximum_volume_difference_mm3=control_delta, passed=control_delta <= 1e-5))
    for name, case in new['pose_cases'].items():
        previous = old['pose_cases'][name]; aa, bb = collision_map(previous), collision_map(case)
        delta = max((abs(aa[n]-bb[n]) for n in aa.keys() & bb.keys()), default=0.)
        rows.append(dict(kind='static_pose', id=name, angles_hex=angles(case['angles_deg']), maximum_volume_difference_mm3=delta,
            missing_pairs=sorted(aa.keys()-bb.keys()), extra_pairs=sorted(bb.keys()-aa.keys()),
            passed=angles(previous['angles_deg']) == angles(case['angles_deg']) and aa.keys() == bb.keys() and delta <= 1e-5 and
                not previous['operation_failures'] and not case['operation_failures']))
    return rows

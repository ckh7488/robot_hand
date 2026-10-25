"""Eight local native-STEP gaps for the current canonical thumb-up endpoint.

Place in cad/gesture_hand_v3, then use --output NEW_REPORT.json. --cad-root can
select another candidate copy. The current completed endpoint report must bind
the exact CAD and pose inputs. No geometry is rebuilt or repaired. Completion
and the 0.5 mm reference target are separate; neither is fabrication approval.
"""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import math
import sys
import time
import traceback

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
VERIFIER_REL = 'cad/articulation_drive_v2/verify.py'
VERIFIER_SHA = '79a0b66b8c5ede499e870e954b32ac0d2e679c4f96a9e3e0bf4d0fdca44680f3'
PAIRS = [(a, b, 'Selected exterior pair')
    for a in ['index_segment3_rounded_tip_cap', 'index_segment3_tip_core']
    for b in ['thumb_segment1_thenar_shell', 'thumb_segment2_segment_shell',
              'thumb_segment2_dark_faceplate']] + [
    ('index_segment3_rounded_tip_cap', 'thumb_segment2_dark_faceplate_clamp1_bolt',
     'Actual bolt involved in the earlier rejected candidate'),
    ('index_segment3_rounded_tip_cap', 'thumb_segment2_dark_faceplate_clamp1_washer',
     'Actual washer involved in the earlier rejected candidate')]


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def write_json(path, report):
    temporary = path.with_suffix(path.suffix+'.partial')
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)


def angle_key(values):
    if not isinstance(values, list) or len(values) != 20 or any(
        isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
        raise ValueError('Expected exactly twenty finite angles')
    return tuple(float(x).hex() for x in values)


def recorded_digest(hashes, relative):
    # Audit input paths may refer to the original checkout. Matching the whole
    # unique repository-relative suffix permits an unchanged committed copy.
    suffix = '/'+relative.replace('\\', '/').lower()
    matches = [value for name, value in hashes.items()
        if ('/'+name.replace('\\', '/').lstrip('/').lower()).endswith(suffix)]
    if len(matches) != 1:
        raise ValueError('Expected one endpoint-provenance entry: '+relative)
    return matches[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cad-root', type=Path, default=HERE)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cad = args.cad_root.resolve()
    root = cad.parents[1]
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError('Preserve the existing result; choose a new output')
    output.parent.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    report = dict(schema_version='gesture_hand_v3.thumb_local_clearance.v1', completed=False,
        parts=[], distances=[], source_identity_checks=[], errors=[], reference_target_mm=.5,
        all_selected_distances_positive=None, all_selected_pairs_meet_reference_target=None,
        scope='Eight named native-part distances at current canonical thumb_up only; not a global minimum or transition check.',
        inherited_endpoint_scope_parts=349, current_native_parts_reimported=0,
        all_native_files_rehashed=False,
        tolerance_stack_verified=False, dynamics_verified=False, manufacture_approved=False)
    before = {}
    try:
        def register(path):
            path = path.resolve()
            if path == output:
                raise ValueError('Output cannot overwrite an input')
            value = digest(path)
            if str(path) in before and before[str(path)] != value:
                raise ValueError('Input changed during loading: '+str(path))
            before[str(path)] = value
            return value

        register(Path(__file__))
        files = dict(design=cad/'design.json', poses=cad/'poses.json',
            scene=cad/'generated/scene.json', build=cad/'generated/build_report.json',
            mass=cad/'generated/mass_inertia.json', endpoint=cad/'reviews/gesture_endpoints.json')
        for path in files.values():
            register(path)
        docs = {name: read_json(path) for name, path in files.items()}
        design, catalog, scene, build, mass, endpoint = [docs[k] for k in
            ('design', 'poses', 'scene', 'build', 'mass', 'endpoint')]
        guard = endpoint.get('input_hash_guard', {})
        if endpoint.get('completed') is not True or endpoint.get('errors') or guard.get('passed') is not True or not guard.get('start') or guard.get('start') != guard.get('end'):
            raise ValueError('Canonical endpoint audit is incomplete or its input guard failed')
        for name, expected in [('material_overlap_mm3', .02), ('boolean_tolerance_mm', 1e-6)]:
            if endpoint.get('thresholds', {}).get(name) != expected:
                raise ValueError('Canonical endpoint collision threshold differs: '+name)

        def identify(path):
            path = path.resolve()
            relative = path.relative_to(root).as_posix()
            actual, expected = register(path), recorded_digest(guard['start'], relative)
            row = dict(path=relative, sha256=actual, endpoint_sha256=expected, passed=actual == expected)
            report['source_identity_checks'].append(row)
            if not row['passed']:
                raise ValueError('Input changed since canonical endpoint audit: '+relative)
            return actual

        for key in ('design', 'poses', 'scene', 'build'):
            identify(files[key])
        helper = root/VERIFIER_REL
        if identify(helper) != VERIFIER_SHA:
            raise ValueError('Frozen native geometry/FK helper changed')
        spec = importlib.util.spec_from_file_location('_thumb_gap_frozen_v2', helper)
        v = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(v)
        import cadquery as cq
        import numpy as np
        report['runtime'] = dict(cadquery=cq.__version__, numpy=np.__version__, python=sys.version)
        if scene.get('inputs_unchanged') is not True or build.get('inputs_unchanged') is not True:
            raise ValueError('Native build start/end guard is missing')
        if not scene.get('source_sha256') or scene['source_sha256'] != build.get('source_sha256') or scene['source_sha256'] != mass.get('source_sha256'):
            raise ValueError('Scene/build/mass input provenance differs')
        for relative, expected in scene['source_sha256'].items():
            source = v.safe_relative(root, relative)
            if register(source) != expected:
                raise ValueError('Current CAD/mass source is stale: '+relative)

        rows = {r['name']: r for r in scene['hand']}
        audited = {r['name']: r for r in endpoint['parts']}
        masses = {r['name']: r for r in mass['parts']}
        if len(rows) != 349 or len(rows) != len(scene['hand']) or len(audited) != len(endpoint['parts']) or len(masses) != len(mass['parts']) or rows.keys() != audited.keys() or rows.keys() != masses.keys():
            raise ValueError('Expected the same 349 distinct native/endpoint/mass components')
        if any(r.get('valid') is not True or r.get('solid_count') != 1 or r.get('mapping_passed') is not True for r in audited.values()):
            raise ValueError('Canonical endpoint audit contains an invalid/unidentified saved component')
        if build.get('part_count') != 349 or build.get('all_parts_valid_single_solid') is not True:
            raise ValueError('Canonical native build is not the declared valid349 assembly')
        gestures = {g['id']: g for g in catalog['gestures']}
        if len(gestures) != len(catalog['gestures']) or catalog['joint_names'] != v.joint_names():
            raise ValueError('Duplicate gesture or unexpected joint order')
        q = gestures['thumb_up']['angles_deg']
        key = angle_key(q)
        result = endpoint['gesture_results']['thumb_up']
        case = endpoint['cases'][result['case_id']]
        observations = [r for r in endpoint['observations'] if r.get('kind') == 'endpoint' and r.get('gesture') == 'thumb_up']
        if len(observations) != 1 or observations[0]['case_id'] != result['case_id'] or angle_key(observations[0]['angles_deg']) != key or angle_key(case['angles_deg']) != key:
            raise ValueError('Canonical thumb_up angles do not match its audited observation/case')
        if case.get('total_pairs_in_scope') != 349*348//2 or case.get('collision_count') != 0 or case.get('collision_details') or case.get('operation_failures') or case.get('no_reportable_material_collision_and_no_operation_failure') is not True or result.get('collision_count') != 0 or result.get('operation_failure_count') != 0:
            raise ValueError('Canonical thumb_up lacks the complete349-part collision/operation pass')
        for name, angle in zip(v.joint_names(), q):
            range_key = name if name.startswith('thumb_') else name.split('_', 1)[1]
            lo, hi = design['joint_ranges_deg'][range_key]
            if not lo <= angle <= hi:
                raise ValueError('Current angle outside its declared range: '+name)
        report['selected_pose'] = dict(id='thumb_up', angles_deg=q, angles_hex=list(key),
            endpoint_case_id=result['case_id'], source_part_count=349,
            endpoint_report_sha256=before[str(files['endpoint'].resolve())],
            mass_file_sha256=before[str(files['mass'].resolve())])
        neutral, current = v.frames(design, [0.]*20), v.frames(design, q)
        shapes = {}
        for name in sorted({name for pair in PAIRS for name in pair[:2]}):
            row = rows[name]
            if row['mesh_units'] != 'mm' or row['body'] not in neutral or row['body'] != audited[name]['body'] or row['body'] != masses[name]['body']:
                raise ValueError('Unknown/inconsistent native body or units: '+name)
            step = v.safe_relative(cad, row['step'])
            if identify(step) != row['sha256_step']:
                raise ValueError('Scene/native STEP hash mismatch: '+name)
            shape = cq.importers.importStep(str(step)).val()
            volume = float(shape.Volume())
            volume_error = abs(volume-row['volume_mm3'])
            mass_volume_error = abs(volume-masses[name]['volume_mm3'])
            bbox_error = float(np.max(np.abs(v.bounds(shape)-row['bbox_mm'])))
            threshold = max(.05, abs(row['volume_mm3'])*1e-5)
            good = bool(shape.isValid() and len(shape.Solids()) == 1 and
                volume_error <= threshold and mass_volume_error <= threshold and bbox_error <= .002)
            report['parts'].append(dict(name=name, body=row['body'], native_step=row['step'],
                sha256_step=row['sha256_step'], valid=bool(shape.isValid()), solid_count=len(shape.Solids()),
                scene_volume_error_mm3=volume_error, mass_volume_error_mm3=mass_volume_error,
                scene_bbox_error_mm=bbox_error, identity_passed=good))
            report['current_native_parts_reimported'] = len(report['parts'])
            if not good:
                raise ValueError('Saved STEP validity/identity mismatch: '+name)
            shapes[name] = v.relocate(shape, current[row['body']], neutral[row['body']])
        for a, b, reason in PAIRS:
            distance = float(shapes[a].distance(shapes[b]))
            if not math.isfinite(distance) or distance < 0:
                raise ValueError('Nonfinite or negative native distance: '+a+' / '+b)
            row = dict(a=a, b=b, body_a=rows[a]['body'], body_b=rows[b]['body'], reason=reason,
                native_distance_mm=distance, finite_positive=distance > 0,
                meets_0p5mm_reference=distance >= .5)
            report['distances'].append(row)
            print(json.dumps(row, allow_nan=False), flush=True)
        report['minimum_among_selected_pairs_mm'] = min(r['native_distance_mm'] for r in report['distances'])
        report['all_selected_distances_positive'] = all(r['finite_positive'] for r in report['distances'])
        report['all_selected_pairs_meet_reference_target'] = all(r['meets_0p5mm_reference'] for r in report['distances'])
        report['completed'] = True
    except Exception as exc:
        report['errors'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
    after = {name: digest(name) if Path(name).is_file() else None for name in before}
    report['input_hash_guard'] = dict(start=before, end=after, passed=bool(before) and before == after)
    report['completed'] = bool(report['completed'] and report['input_hash_guard']['passed'])
    report['elapsed_s'] = time.perf_counter()-started
    write_json(output, report)
    print(json.dumps(dict(completed=report['completed'], pairs=len(report['distances']),
        minimum_among_selected_pairs_mm=report.get('minimum_among_selected_pairs_mm'),
        meets_reference=report['all_selected_pairs_meet_reference_target'],
        guard=report['input_hash_guard']['passed'], errors=report['errors'])), flush=True)
    return 0 if report['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

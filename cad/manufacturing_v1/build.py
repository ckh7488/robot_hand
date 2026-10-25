"""Create a new 349-part candidate, changing only 10 pins and 10 left bushes.

No CAD operation or subprocess starts on import. Public construction API:
  make_pin(cq, kind='round') -> local-X CadQuery Shape
  swept_pin(cq, kind='round', travel=30.0) -> exact translational union

The sweep is insertion from local translation (-travel, 0, 0) to (0, 0, 0).
Every constant cross-section interval is extruded across its whole travel and
the resulting prisms are united. This is not a sampled-pose approximation.
All dimensions are millimetres. The old model, a full-length D alternative and
the selected round pin can be reconstructed without importing the old builder.

Run only in the project's hidden, single-thread CAD worker. No subprocess is
launched here. The fixed HERE/generated destination must not already exist.
Unchanged native/mesh files remain references to the read-only base candidate;
all 349 solids are also exported as one new native assembly.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import traceback

HERE = Path(__file__).resolve().parent
PIN_DIMENSIONS = {
    'shaft_radius': 2.0, 'shaft_span_x': [-13.0, 14.4],
    'head_radius': 3.5, 'head_span_x': [-15.0, -13.0],
    'groove_radius': 1.6, 'groove_span_x': [13.1, 13.9],
    'old_flat_y': 1.7, 'old_flat_span_x': [-13.0, -9.0],
    'left_parent_journal_radius': 2.015,
}
KINDS = ('old', 'full_D', 'round')


def _cylinder(cq, radius, lo, hi):
    return cq.Solid.makeCylinder(radius, hi-lo, cq.Vector(lo, 0, 0), cq.Vector(1, 0, 0))


def _flat_cut(cq, lo, hi):
    return cq.Workplane('XY').box(hi-lo, 4, 10).translate(((lo+hi)/2, 3.7, 0)).val()


def make_pin(cq, kind='round'):
    """Original headed/grooved pin, full-D alternative, or round selected pin."""
    if kind not in KINDS:
        raise ValueError('Unknown pin kind: '+str(kind))
    pin = _cylinder(cq, 2, -13, 14.4).fuse(_cylinder(cq, 3.5, -15, -13))
    if kind == 'old':
        pin = pin.cut(_flat_cut(cq, -13, -9))
    elif kind == 'full_D':
        pin = pin.cut(_flat_cut(cq, -13, 14.4))
    groove = _cylinder(cq, 2.4, 13.1, 13.9).cut(_cylinder(cq, 1.6, 12.1, 14.9))
    return pin.cut(groove).clean()


def swept_pin(cq, kind='round', travel=30.0):
    """Exact union of the pin over continuous local-X translation -travel..0.

    The head remains unflattened in all kinds. The groove section is circular:
    its R1.6 is already below the D flat at Y=1.7. Zero travel is permitted.
    """
    if kind not in KINDS:
        raise ValueError('Unknown pin kind: '+str(kind))
    travel = float(travel)
    if not math.isfinite(travel) or travel < 0:
        raise ValueError('Insertion travel must be finite and nonnegative')
    sections = [(-15., -13., 3.5, False),
                (-13., -9., 2., kind in ('old', 'full_D')),
                (-9., 13.1, 2., kind == 'full_D'),
                (13.1, 13.9, 1.6, False),
                (13.9, 14.4, 2., kind == 'full_D')]
    swept = None
    for lo, hi, radius, flat in sections:
        solid = _cylinder(cq, radius, lo-travel, hi)
        if flat:
            solid = solid.cut(_flat_cut(cq, lo-travel, hi))
        swept = solid if swept is None else swept.fuse(solid)
    return swept.clean()


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


class Inputs:
    """Snapshot the bytes actually read, and rehash every input at completion."""
    def __init__(self, repo):
        self.repo = Path(repo).resolve()
        self.start = {}

    def key(self, path):
        path = Path(path).resolve()
        if not path.is_relative_to(self.repo) or not path.is_file():
            raise ValueError('Input must be an existing file inside the repository: '+str(path))
        return path.relative_to(self.repo).as_posix()

    def record(self, path, sha, expected=None):
        key = self.key(path)
        if key in self.start and self.start[key] != sha:
            raise RuntimeError('Input changed while reading: '+key)
        if expected is not None and sha != expected:
            raise ValueError('Recorded input hash mismatch: '+key)
        self.start[key] = sha
        return sha

    def track(self, path, expected=None):
        return self.record(path, digest(path), expected)

    def read_json(self, path):
        raw = Path(path).read_bytes()
        self.record(path, hashlib.sha256(raw).hexdigest())
        return json.loads(raw.decode('utf-8-sig'))

    def final_guard(self):
        after = {key: digest(self.repo/key) if (self.repo/key).is_file() else None
                 for key in self.start}
        return dict(passed=self.start == after, start=dict(self.start), end=after)


def inside(root, relative):
    relative = str(relative).replace('\\', '/')
    if Path(relative).is_absolute() or ':' in relative or '..' in Path(relative).parts:
        raise ValueError('Expected a contained relative source path: '+relative)
    path = (Path(root)/relative).resolve()
    if not path.is_relative_to(Path(root).resolve()) or not path.is_file():
        raise ValueError('Missing or escaping source: '+relative)
    return path


def load_module(name, path, inputs):
    """Hash-bound unique-name import; never use an ambient same-name module."""
    path = Path(path).resolve()
    sha = inputs.track(path)
    unique_name = '_manufacturing_v1_'+name+'_'+sha[:16]
    spec = importlib.util.spec_from_file_location(unique_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[unique_name] = module
    spec.loader.exec_module(module)
    if Path(module.__file__).resolve() != path or digest(path) != sha:
        raise RuntimeError('Imported module identity changed: '+str(path))
    return module


def shape_check(shape, label):
    volume = float(shape.Volume())
    result = dict(valid=bool(shape.isValid()), solid_count=len(shape.Solids()), volume_mm3=volume)
    if not result['valid'] or result['solid_count'] != 1 or not math.isfinite(volume) or volume <= 0:
        raise ValueError('Expected one valid positive-volume solid: '+label)
    return result


def difference(a, b, tolerance, limit):
    ab = float(a.cut(b, tol=tolerance).Volume())
    ba = float(b.cut(a, tol=tolerance).Volume())
    return dict(a_minus_b_mm3=ab, b_minus_a_mm3=ba, limit_mm3=limit,
                passed=math.isfinite(ab) and math.isfinite(ba) and abs(ab) <= limit and abs(ba) <= limit)


def _write(path, document):
    Path(path).write_text(json.dumps(document, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=HERE.parents[1])
    args = parser.parse_args()
    repo = args.repo.resolve()
    if not HERE.is_relative_to(repo) or HERE != repo/'cad/manufacturing_v1':
        raise ValueError('Run this candidate from the selected repository copy')
    out = HERE/'generated'
    if out.exists():
        raise FileExistsError('Use a fresh candidate copy; generated already exists: '+str(out))
    inputs = Inputs(repo)
    inputs.track(Path(__file__))
    patch = inputs.read_json(HERE/'design_patch.json')
    if patch['pin_kind'] != 'round' or patch['pin_dimensions_mm'] != PIN_DIMENSIONS:
        raise ValueError('This source implements only the explicit round-pin dimension contract')
    if (patch['candidate_thumb_ip_range_deg'] != [-20, 85]
            or patch['original_thumb_ip_range_deg'] != [0, 85]
            or patch['reference_insertion_travel_mm'] != 30):
        raise ValueError('Unexpected range or insertion-domain change')
    base = repo/patch['base_candidate']
    if base.resolve() != repo/'cad/gesture_hand_v3':
        raise ValueError('Unexpected baseline candidate')
    scene = inputs.read_json(base/'generated/scene.json')
    base_build = inputs.read_json(base/'generated/build_report.json')
    mass = inputs.read_json(base/'generated/mass_inertia.json')
    design = inputs.read_json(base/'design.json')
    if design['joint_ranges_deg']['thumb_ip'] != patch['original_thumb_ip_range_deg']:
        raise ValueError('Baseline thumb range changed')
    if len(scene['hand']) != patch['expected_base_part_count'] or len(scene['hand']) != 349:
        raise ValueError('Expected the complete 349-part hand')
    if scene.get('arm_reference'):
        raise ValueError('This construction does not change or include an arm')
    for doc in (scene, base_build):
        if doc.get('inputs_unchanged') is not True:
            raise ValueError('Baseline guarded-build declaration missing')
    for doc in (scene, base_build, mass):
        for relative, sha in doc['source_sha256'].items():
            inputs.track(inside(repo, relative), sha)
    inputs.track(base/'generated/hand.step')
    for row in scene['hand']:
        inputs.track(inside(base, row['step']), row['sha256_step'])
        inputs.track(inside(base, row['mesh']), row['sha256_stl'])
    names = [r['name'] for r in scene['hand']]
    if len(set(names)) != len(names):
        raise ValueError('Duplicate baseline part names')
    masses = {r['name']: r for r in mass['parts']}
    if set(masses) != set(names) or len(mass['parts']) != len(names):
        raise ValueError('Mass/scene part set differs')
    for row in scene['hand']:
        if row['body'] != masses[row['name']]['body']:
            raise ValueError('Mass/scene body differs: '+row['name'])

    # Imports occur only inside the explicitly invoked build, after the inputs
    # have been snapshotted. The two helpers do not run their own exporters.
    sys.dont_write_bytecode = True
    import cadquery as cq
    repair = load_module('frozen_fabrication_repairs', base/'fabrication_repairs.py', inputs)
    helper = load_module('exact_mass_helper', repo/'cad/form_drive_v1/exterior.py', inputs)
    tolerance = float(patch['boolean_tolerance_mm'])
    limit = float(patch['bidirectional_difference_limit_mm3'])
    bounds_limit = float(patch['preserved_part_bounds_limit_mm'])
    if not (0 < tolerance <= 1e-6 and 0 < limit <= 1e-5 and 0 < bounds_limit <= 1e-5):
        raise ValueError('Unexpectedly relaxed geometric comparison threshold')
    changes_by_name = {}
    for config in [*design['fingers'], dict(design['thumb'], name='thumb')]:
        finger = config['name']
        for joint in (1, 2):
            frame = repair._frame(config, joint)
            body = finger+('_opposition' if finger == 'thumb' and joint == 1 else
                           '_mcp' if joint == 1 or finger == 'thumb' else '_pip')
            for name, kind in [(f'{finger}_joint_pin_{joint}', 'pin'),
                               (f'{finger}_segment{joint}_distal_left_bushing', 'left_parent_bushing')]:
                if name not in names or next(r for r in scene['hand'] if r['name'] == name)['body'] != body:
                    raise ValueError('Missing or incorrectly assigned source part: '+name)
                changes_by_name[name] = dict(kind=kind, frame=frame, body=body,
                                            digit=finger, distal_joint=joint)
    if len(changes_by_name) != patch['expected_modified_part_count'] or len(changes_by_name) != 20:
        raise ValueError('Expected exactly 10 pins and 10 left parent bushes')

    report = dict(schema='manufacturing_v1.round_pin_build.v1', completed=False,
                  base_candidate=patch['base_candidate'], part_count=349,
                  expected_modified_names=list(changes_by_name), changed_parts=[],
                  unchanged_part_count=329, part_checks=[], original_pin_reconstruction=[],
                  original_bushing_reconstruction=[], round_pin_rotational_symmetry=[], errors=[],
                  insertion_sweep_available=True, insertion_sweep_executed=False,
                  original_geometry_mutated=False, geometry_modified=True,
                  manufacturing_approved=False, purchase_specification_complete=False,
                  interpretation=patch['interpretation'], design_patch=patch,
                  limitations=['No insertion, whole-hand motion or contact/fit acceptance is performed by this builder.',
                               'Pin spin and friction are not fixed to the parent body or dynamically solved.',
                               'Original clip rotation, elastic installation and axial retention require separate review.'])
    out.mkdir()
    (out/'parts').mkdir()
    (out/'meshes').mkdir()
    try:
        result_rows = []
        result_masses = []
        assembly = cq.Assembly(name='manufacturing_v1_round_floating_distal_pins')
        old_pin_local = make_pin(cq, 'old')
        round_pin_local = make_pin(cq, 'round')
        shape_check(old_pin_local, 'reconstructed old pin')
        shape_check(round_pin_local, 'selected round pin')
        symmetry = difference(round_pin_local,
                              round_pin_local.rotate((0, 0, 0), (1, 0, 0), 90), tolerance, limit)
        if not symmetry['passed']:
            raise ValueError('Selected round pin failed the 90-degree symmetry check')
        for index, row in enumerate(scene['hand']):
            name = row['name']
            old_shape = cq.importers.importStep(str(inside(base, row['step']))).val()
            old_check = shape_check(old_shape, name+' source')
            if abs(old_check['volume_mm3']-row['volume_mm3']) > max(1e-5, 1e-8*row['volume_mm3']):
                raise ValueError('Imported source volume mismatch: '+name)
            shape = old_shape
            new_row = copy.deepcopy(row)
            new_mass = copy.deepcopy(masses[name])
            if name in changes_by_name:
                change = changes_by_name[name]
                frame = change['frame']
                if change['kind'] == 'pin':
                    rebuilt = repair._world(old_pin_local, frame)
                    match = difference(rebuilt, old_shape, tolerance, limit)
                    report['original_pin_reconstruction'].append(dict(name=name, **match))
                    if not match['passed']:
                        raise ValueError('Original pin reconstruction differs from saved STEP: '+name)
                    shape = repair._world(round_pin_local, frame)
                    role = 'free_rotating_round_clevis_pin'
                    report['round_pin_rotational_symmetry'].append(dict(name=name, local_axis=[1, 0, 0],
                        rotation_deg=90, method='common local native pin, placed by the preserved rigid frame', **symmetry))
                else:
                    rebuilt = repair._world(repair._bushing(cq, -12, 3, -12.4, .4, keyed_shaft=True), frame)
                    match = difference(rebuilt, old_shape, tolerance, limit)
                    report['original_bushing_reconstruction'].append(dict(name=name, **match))
                    if not match['passed']:
                        raise ValueError('Original left bush differs from saved STEP: '+name)
                    bore = repair._world(_cylinder(cq, 2.015, -13.4, -8), frame)
                    shape = old_shape.cut(bore).clean()
                    expected = repair._world(repair._bushing(cq, -12, 3, -12.4, .4, keyed_shaft=False), frame)
                    match = difference(shape, expected, tolerance, limit)
                    if not match['passed']:
                        raise ValueError('Round-journal expansion changed more than the intended bore: '+name)
                    role = 'outer_keyed_parent_bushing_round_journal'
                checked = shape_check(shape, name+' candidate')
                props = helper.mass_properties(shape, masses[name]['density_kg_per_mm3'])
                if not all(math.isfinite(float(x)) for x in (props['volume_mm3'], props['mass_kg'])):
                    raise ValueError('Non-finite exact mass properties: '+name)
                bounds_error = max(abs(props['bbox_mm'][i][j]-row['bbox_mm'][i][j])
                                   for i in range(2) for j in range(3))
                if bounds_error > bounds_limit:
                    raise ValueError('Part footprint changed: '+name+' '+str(bounds_error))
                if (change['kind'] == 'pin' and props['volume_mm3'] <= old_check['volume_mm3']) or (
                        change['kind'] != 'pin' and props['volume_mm3'] >= old_check['volume_mm3']):
                    raise ValueError('Unexpected material change direction: '+name)
                step = out/'parts'/f'{name}.step'
                mesh = out/'meshes'/f'{name}.stl'
                cq.exporters.export(shape, str(step))
                cq.exporters.export(shape, str(mesh), tolerance=.1, angularTolerance=.1)
                new_row.update(step=step.relative_to(HERE).as_posix(), mesh=mesh.relative_to(HERE).as_posix(),
                    sha256_step=digest(step), sha256_stl=digest(mesh), role=role,
                    source='manufacturing_v1_round_floating_pin_patch', valid=True, solid_count=1,
                    volume_mm3=props['volume_mm3'], bbox_mm=props['bbox_mm'])
                new_mass.update(role=role, **props)
                report['changed_parts'].append(dict(name=name, **change, old_volume_mm3=old_check['volume_mm3'],
                    new_volume_mm3=props['volume_mm3'], delta_volume_mm3=props['volume_mm3']-old_check['volume_mm3'],
                    old_mass_kg=masses[name]['mass_kg'], new_mass_kg=props['mass_kg'],
                    old_bbox_mm=row['bbox_mm'], new_bbox_mm=props['bbox_mm'],
                    maximum_bounds_change_mm=bounds_error, bounds_limit_mm=bounds_limit,
                    step=new_row['step'], mesh=new_row['mesh'], body_preserved=True,
                    source_step=row['step'], source_step_sha256=row['sha256_step'],
                    valid=checked['valid'], solid_count=checked['solid_count']))
            else:
                new_row.update(step='../gesture_hand_v3/'+row['step'].replace('\\', '/'),
                               mesh='../gesture_hand_v3/'+row['mesh'].replace('\\', '/'))
            if new_row['body'] != row['body'] or new_row['step_index'] != row['step_index']:
                raise ValueError('A body or assembly-order datum changed: '+name)
            report['part_checks'].append(dict(name=name, modified=name in changes_by_name,
                                             **shape_check(shape, name)))
            result_rows.append(new_row)
            result_masses.append(new_mass)
            assembly.add(shape, name=name, color=cq.Color(*row['color'][:3]))
            if (index+1) % 25 == 0 or index+1 == len(scene['hand']):
                print(json.dumps(dict(progress='loaded_and_added', completed=index+1, total=349)), flush=True)
        if {r['name'] for r in report['changed_parts']} != set(changes_by_name):
            raise ValueError('Modified part set differs from the declared exact 20')
        design = copy.deepcopy(design)
        design['joint_ranges_deg']['thumb_ip'] = patch['candidate_thumb_ip_range_deg']
        design['manufacturing_candidate'] = dict(name='manufacturing_v1', base_candidate=patch['base_candidate'],
            changed_part_names=list(changes_by_name), pin_kind='round_floating_clevis_pin',
            interpretation=patch['interpretation'], manufacturing_approved=False)
        _write(out/'design.json', design)
        assembly.save(str(out/'hand.step'))
        guard = inputs.final_guard()
        if not guard['passed']:
            raise RuntimeError('Build inputs changed; generated output is not a completed candidate')
        new_scene = dict(hand=result_rows, arm_reference=copy.deepcopy(scene.get('arm_reference', [])),
                         source_sha256=dict(inputs.start), inputs_unchanged=True,
                         base_candidate=patch['base_candidate'], interpretation=patch['interpretation'])
        new_mass_doc = copy.deepcopy(mass)
        new_mass_doc.update(parts=result_masses, source_sha256=dict(inputs.start), inputs_unchanged=True,
            total_mass_kg=sum(r['mass_kg'] for r in result_masses),
            moving_mass_kg=sum(r['mass_kg'] for r in result_masses if r['body'] != 'palm'),
            limitations=list(mass.get('limitations', []))+[
                'Round pin passive spin/friction is not modelled; retained parent-body mass assignment is bookkeeping.',
                'Geometric mass update only; no motor, tendon, structural or hardware-performance approval.'])
        _write(out/'scene.json', new_scene)
        _write(out/'mass_inertia.json', new_mass_doc)
        report.update(completed=True, inputs_unchanged=True, source_sha256=dict(inputs.start),
            input_hash_guard=guard, all_parts_valid_single_solid=True, modified_part_count=20,
            exported_part_file_count=20, referenced_unchanged_part_count=329,
            assembly_step='generated/hand.step', assembly_step_sha256=digest(out/'hand.step'),
            total_mass_kg=new_mass_doc['total_mass_kg'], moving_mass_kg=new_mass_doc['moving_mass_kg'],
            baseline_total_mass_kg=mass['total_mass_kg'], baseline_moving_mass_kg=mass['moving_mass_kg'],
            generated_design='generated/design.json', generated_design_sha256=digest(out/'design.json'))
        # A second end guard covers the interval spent writing metadata too.
        report['input_hash_guard'] = inputs.final_guard()
        if not report['input_hash_guard']['passed']:
            raise RuntimeError('Build inputs changed before completion')
        _write(out/'build_report.json', report)
        print(json.dumps(dict(completed=True, part_count=349, changed_part_count=20,
                              inputs_unchanged=True, output=str(out))), flush=True)
        return 0
    except Exception as exc:
        report.update(completed=False, inputs_unchanged=False, manufacturing_approved=False)
        report['errors'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
        report['input_hash_guard'] = inputs.final_guard()
        _write(out/'build_failure.json', report)
        raise


if __name__ == '__main__':
    raise SystemExit(main())

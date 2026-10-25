"""Construct a separate nominal screw-retained distal-pin candidate, in mm.

Import is inert: CadQuery is supplied explicitly to the public shape functions.
No export, subprocess or CAD operation runs until a function/main is invoked.
The final assembly has 359 parts: modify 10 pins, remove 10 clips, add 10 keeper
washers and 10 screws, preserve the exact STEP/STL bytes of the other 329 parts.
Ten pilot-only machining blanks are exported separately and never assembled.

Public API: make_pin(cq, machining_blank=False), make_washer(cq), make_screw(cq),
pin_outer_envelope(cq, travel=0), washer_sweep(cq, travel=30), and
screw_outer_envelope(cq, travel=0). A positive travel extends the pin envelope
left (-X), and keeper/screw envelopes right (+X). Travel is continuous, not
discrete poses. Thread flanks, elastic fits, preload and physical assembly are
not solved. The parent performs saved-native-part audits separately.
"""
from __future__ import annotations
from pathlib import Path
import argparse, copy, hashlib, importlib.util, json, math, os, sys, traceback

HERE = Path(__file__).resolve().parent
DIGITS = ('index', 'middle', 'ring', 'little', 'thumb')
DENSITY = 7.85e-6
DIMENSIONS = {
    'pin_shaft_radius': 2.0, 'pin_shaft_span_x': [-13.0, 13.15],
    'pin_head_radius': 3.5, 'pin_head_span_x': [-15.0, -13.0],
    'pin_head_slot_width_z': 1.0, 'pin_head_slot_depth_x': 0.8,
    'female_major_radius': 1.0, 'female_major_span_x': [8.75, 13.15],
    'female_full_thread_span_x': [8.75, 12.75], 'thread_pitch': 0.4,
    'tap_runout_max': 1.2, 'tap_runout_min_x': 7.55,
    'pilot_radius': 0.8, 'pilot_drilling_span_x': [7.15, 13.15],
    'drill_point_included_angle_deg': 118.0, 'mouth_radius': 1.2,
    'simplified_major_entry_chamfer_span_x': [12.95, 13.15],
    'blank_entry_chamfer_span_x': [12.75, 13.15],
    'washer_outer_radius': 3.5, 'washer_inner_radius': 1.1, 'washer_span_x': [13.15, 13.95],
    'screw_major_radius': 1.0, 'screw_shaft_span_x': [9.95, 13.95],
    'screw_head_radius': 1.9, 'screw_head_span_x': [13.95, 15.95],
    'screw_socket_across_flats': 1.5, 'screw_socket_span_x': [14.95, 15.95],
}


def _cylinder(cq, radius, lo, hi):
    return cq.Solid.makeCylinder(radius, hi-lo, cq.Vector(lo, 0, 0), cq.Vector(1, 0, 0))


def _cone(cq, radius_lo, radius_hi, lo, hi):
    return cq.Solid.makeCone(radius_lo, radius_hi, hi-lo, cq.Vector(lo, 0, 0), cq.Vector(1, 0, 0))


def _travel(value):
    value = float(value)
    if not math.isfinite(value) or value < 0: raise ValueError('Travel must be finite and nonnegative')
    return value


def drill_tip_x():
    """118-degree included drill point beyond the full-diameter pilot floor."""
    return 7.15 - .8 / math.tan(math.radians(118.0/2))


def _pin(cq, machining_blank, with_slot):
    shape = _cylinder(cq, 2, -13, 13.15).fuse(_cylinder(cq, 3.5, -15, -13))
    pilot = _cylinder(cq, .8, 7.15, 13.16).fuse(_cone(cq, 0, .8, drill_tip_x(), 7.15))
    # The pilot blank has a .4x.4 chamfer. The major-diameter representation
    # removes its inner portion, leaving a visible .2x.2 chamfer in this
    # simplified CAD only. Actual full thread profile begins behind X12.75,
    # not X12.95: the physical .4 chamfer truncates the internal thread crests.
    mouth = _cone(cq, .8, 1.2, 12.75, 13.15).fuse(_cylinder(cq, 1.2, 13.15, 13.16))
    void = pilot.fuse(mouth)
    if not machining_blank: void = void.fuse(_cylinder(cq, 1, 8.75, 13.16))
    shape = shape.cut(void)
    if with_slot:
        # Open across the entire Y width. Overshoot only outside the solid.
        slot = cq.Workplane('XY').box(.9, 8, 1).translate((-14.65, 0, 0)).val()
        shape = shape.cut(slot)
    return shape.clean()


def make_pin(cq, machining_blank=False):
    """Finished simplified female-thread envelope, or separate pilot-only blank.

    The finished internal R1 cylinder represents the M2 major-diameter envelope;
    it is not a tap-drill instruction and contains no helical thread flanks.
    machining_blank=True leaves the R0.8 drill and its point/chamfer only.
    """
    if not isinstance(machining_blank, bool): raise TypeError('machining_blank must be bool')
    return _pin(cq, machining_blank, True)


def make_washer(cq):
    """Nominal DIN9021 M2 keeper: OD7, ID2.2, thickness .8 at X13.15..13.95."""
    return _cylinder(cq, 3.5, 13.15, 13.95).cut(_cylinder(cq, 1.1, 13.14, 13.96)).clean()


def make_screw(cq):
    """Nominal DIN912 M2x4; true 1.5 across-flats socket, no thread helices."""
    screw = _cylinder(cq, 1, 9.95, 13.95).fuse(_cylinder(cq, 1.9, 13.95, 15.95))
    circumdiameter = 1.5 / math.cos(math.pi/6)
    socket = cq.Workplane('YZ').polygon(6, circumdiameter).extrude(1.01).translate((14.95, 0, 0)).val()
    return screw.cut(socket).clean()


def pin_outer_envelope(cq, travel=0):
    """At zero: fill head slot, retain female bore to avoid false screw contact.

    At positive travel: conservative continuous outer sweep from -travel to 0;
    holes and slot are filled. This is a nominal clearance bound, not a solved
    threaded connection. A rotating head has the full cylindrical outer bound.
    """
    travel = _travel(travel)
    if travel == 0: return _pin(cq, False, False)
    return _cylinder(cq, 2, -13-travel, 13.15).fuse(_cylinder(cq, 3.5, -15-travel, -13)).clean()


def washer_sweep(cq, travel=30):
    """Exact constant-annulus translation union over local +X distance travel."""
    travel = _travel(travel)
    return _cylinder(cq, 3.5, 13.15, 13.95+travel).cut(_cylinder(cq, 1.1, 13.14, 13.96+travel)).clean()


def screw_outer_envelope(cq, travel=0):
    """Socket-filled major shaft/head outer bound; positive travel extends +X.

    It is a nominal approach/rotation envelope, not a simulation of screw
    helices advancing through a real internal thread.
    """
    travel = _travel(travel)
    return _cylinder(cq, 1, 9.95, 13.95+travel).fuse(_cylinder(cq, 1.9, 13.95, 15.95+travel)).clean()


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''): value.update(block)
    return value.hexdigest()


def _write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')


class Inputs:
    def __init__(self, repo): self.repo, self.start = Path(repo).resolve(), {}
    def track(self, path, expected=None):
        path = Path(path).resolve()
        if not path.is_relative_to(self.repo) or not path.is_file(): raise ValueError('Missing/escaping input: '+str(path))
        key, value = path.relative_to(self.repo).as_posix(), digest(path)
        if expected is not None and expected != value: raise ValueError('Stale input: '+key)
        if key in self.start and self.start[key] != value: raise ValueError('Changed input: '+key)
        self.start[key] = value; return value
    def read(self, path):
        value = self.track(path); raw = Path(path).read_bytes()
        if hashlib.sha256(raw).hexdigest() != value: raise ValueError('Input changed during read')
        return json.loads(raw.decode('utf-8-sig'))
    def guard(self):
        after = {n: digest(self.repo/n) if (self.repo/n).is_file() else None for n in self.start}
        return dict(passed=self.start == after, start=dict(self.start), end=after)


def _source(repo, root, relative):
    """Resolve baseline sibling references, while refusing repository escape."""
    relative = str(relative).replace('\\', '/')
    if Path(relative).is_absolute() or ':' in relative: raise ValueError('Relative source path required')
    path = (root/relative).resolve()
    if not path.is_relative_to(repo) or not path.is_file(): raise ValueError('Missing/escaping source: '+relative)
    return path


def _module(name, path, inputs):
    path = Path(path).resolve(); sha = inputs.track(path); unique = '_manufacturing_v2_'+name+'_'+sha[:16]
    spec = importlib.util.spec_from_file_location(unique, path); result = importlib.util.module_from_spec(spec)
    sys.modules[unique] = result; spec.loader.exec_module(result)
    if Path(result.__file__).resolve() != path or digest(path) != sha: raise ValueError('Wrong/changed helper module')
    return result


def _valid(shape, label):
    volume = float(shape.Volume()); row = dict(valid=bool(shape.isValid()), solid_count=len(shape.Solids()), volume_mm3=volume)
    if not row['valid'] or row['solid_count'] != 1 or not math.isfinite(volume) or volume <= 0:
        raise ValueError('Expected a valid positive single solid: '+label)
    return row


def _finite_tree(value):
    if isinstance(value, dict): return all(_finite_tree(v) for v in value.values())
    if isinstance(value, (list, tuple)): return all(_finite_tree(v) for v in value)
    return not isinstance(value, (int, float)) or math.isfinite(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=HERE.parents[1])
    parser.add_argument('--required-sources', type=Path, help='Repository-contained required_sources.json; default source_docs/required_sources.json')
    args = parser.parse_args(); repo = args.repo.resolve(); out = HERE/'generated'
    if HERE != repo/'cad/manufacturing_v2': raise ValueError('Run from the selected repository candidate')
    if os.path.lexists(out): raise FileExistsError('Preserve existing output; use a fresh isolated candidate copy')
    inputs = Inputs(repo); inputs.track(__file__); patch = inputs.read(HERE/'design_patch.json')
    expected_counts = dict(expected_base_part_count=349, expected_part_count=359, expected_modified_part_count=10,
                           expected_removed_part_count=10, expected_added_part_count=20, expected_unchanged_part_count=329)
    if any(patch.get(k) != v for k, v in expected_counts.items()) or patch['dimensions_mm'] != DIMENSIONS or patch['metal_density_kg_per_mm3'] != DENSITY:
        raise ValueError('This source implements only the declared 359-part screw-retainer geometry')
    if patch['renamed_parts'] or patch['base_candidate'] != 'cad/manufacturing_v1': raise ValueError('Unexpected rename/base change')
    if patch['boolean_tolerance_mm'] != 1e-6 or patch['bidirectional_difference_limit_mm3'] != 1e-5 or patch['reference_insertion_travel_mm'] != 30:
        raise ValueError('Changed verification thresholds/domain')
    sources_path = args.required_sources.resolve() if args.required_sources else _source(repo, HERE, patch['required_sources'])
    source_document = inputs.read(sources_path)
    if not isinstance(source_document.get('files'), dict) or not source_document['files']: raise ValueError('Required supplier evidence is absent')
    for relative, evidence in source_document['files'].items():
        path = _source(repo, repo, relative)
        if not isinstance(evidence, dict) or path.stat().st_size != evidence['bytes']: raise ValueError('Invalid source byte count: '+relative)
        inputs.track(path, evidence['sha256'])
    base = repo/'cad/manufacturing_v1'
    scene = inputs.read(base/'generated/scene.json'); mass = inputs.read(base/'generated/mass_inertia.json')
    oldbuild = inputs.read(base/'generated/build_report.json'); design = inputs.read(base/'generated/design.json')
    if oldbuild.get('completed') is not True or oldbuild.get('errors') or oldbuild.get('input_hash_guard', {}).get('passed') is not True:
        raise ValueError('Baseline build incomplete')
    if oldbuild['input_hash_guard']['start'] != oldbuild['input_hash_guard']['end']: raise ValueError('Baseline guard differs')
    for document in (scene, mass, oldbuild):
        if document.get('inputs_unchanged') is not True: raise ValueError('Baseline lacks source guard')
        for relative, sha in document['source_sha256'].items(): inputs.track(_source(repo, repo, relative), sha)
    if not (scene['source_sha256'] == mass['source_sha256'] == oldbuild['source_sha256'] == oldbuild['input_hash_guard']['start']):
        raise ValueError('Baseline source coverage differs')
    inputs.track(base/'generated/design.json', oldbuild['generated_design_sha256'])
    inputs.track(base/'generated/hand.step', oldbuild['assembly_step_sha256'])
    rows = scene['hand']; oldnames = [r['name'] for r in rows]; byname = {r['name']: r for r in rows}; masses = {r['name']: r for r in mass['parts']}
    if len(rows) != 349 or len(byname) != 349 or set(masses) != set(byname) or len(mass['parts']) != 349 or scene.get('arm_reference'):
        raise ValueError('Expected complete 349-part hand/mass baseline')
    modified = [f'{d}_joint_pin_{j}' for d in DIGITS for j in (1, 2)]
    removed = [f'{d}_distal{j}_retaining_clip' for d in DIGITS for j in (1, 2)]
    added = [f'{d}_distal{j}_retaining_{kind}' for d in DIGITS for j in (1, 2) for kind in ('washer', 'screw')]
    if patch['modified_names'] != modified or patch['removed_names'] != removed or patch['added_names'] != added:
        raise ValueError('Declared exact changed-name lists differ')
    if not set(modified+removed) <= set(oldnames) or set(added) & set(oldnames): raise ValueError('Part identity collision/missing baseline member')
    unchanged = [n for n in oldnames if n not in set(modified+removed)]
    if len(unchanged) != 329: raise ValueError('Wrong unchanged component count')
    part_changes = dict(modified_names=modified, removed_names=removed, added_names=added, unchanged_names=unchanged, renamed_parts=[])
    sources = {}
    for row in rows:
        if row['body'] != masses[row['name']]['body']: raise ValueError('Baseline mass/body mismatch')
        sources[row['name']] = {}
        for key, hashkey in (('step', 'sha256_step'), ('mesh', 'sha256_stl')):
            path = _source(repo, base, row[key]); inputs.track(path, row[hashkey]); sources[row['name']][key] = path
    # No import-triggered CAD generation; these frozen modules expose functions.
    sys.dont_write_bytecode = True
    import cadquery as cq
    oldbuilder = _module('v1', base/'build.py', inputs)
    geometry = _module('frame_helper', repo/'cad/gesture_hand_v3/fabrication_repairs.py', inputs)
    properties = _module('exact_mass', repo/'cad/form_drive_v1/exterior.py', inputs)
    configs = {c['name']: c for c in design['fingers']}; configs['thumb'] = design['thumb']
    joints = []
    for d in DIGITS:
        for j in (1, 2):
            pin = f'{d}_joint_pin_{j}'; ident = f'{d}_distal{j}'; body = byname[pin]['body']
            expected_body = d+('_opposition' if d == 'thumb' and j == 1 else '_mcp' if j == 1 or d == 'thumb' else '_pip')
            if body != expected_body or byname[ident+'_retaining_clip']['body'] != body or masses[pin]['density_kg_per_mm3'] != DENSITY:
                raise ValueError('Unexpected pin/clip frame, body or density')
            frame = geometry._frame(configs[d], j)
            joints.append(dict(id=ident, digit=d, distal_joint=j, frame=frame, parent_body=body, pin=pin,
                removed_clip=ident+'_retaining_clip', washer=ident+'_retaining_washer', screw=ident+'_retaining_screw',
                axis_origin_mm=frame['origin'], axis_direction=geometry._direction(frame, (1, 0, 0)),
                pin_end_x_mm=13.15, washer_span_x_mm=[13.15, 13.95], screw_shaft_span_x_mm=[9.95, 13.95],
                female_full_thread_span_x_mm=[8.75, 12.75], nominal_full_thread_overlap_mm=2.8,
                nominal_screw_tip_to_major_bottom_mm=1.2, thread_engagement_strength_approved=False))
    report = dict(schema='manufacturing_v2.screw_retained_pin_build.v1', completed=False, errors=[],
        base_candidate=patch['base_candidate'], baseline_commit=patch['baseline_commit'], part_count=359,
        part_changes=part_changes, changed_parts=[], removed_parts=[], added_parts=[], part_checks=[], original_pin_reconstruction=[],
        machining_blanks=[], retention_interfaces=joints, design_patch=patch, required_sources=sources_path.relative_to(repo).as_posix(),
        supplier_evidence=source_document, interpretation=patch['interpretation'], thread_geometry=patch['thread_geometry'],
        machining_callout=patch['machining_callout'], derived_dimensions_mm=dict(drill_tip_x=drill_tip_x(),
            drill_tip_length=7.15-drill_tip_x(), socket_circumdiameter=1.5/math.cos(math.pi/6),
            full_thread_length=4.0, nominal_thread_overlap=2.8, tap_runout_to_drill_cylinder_floor_margin=.4),
        original_geometry_mutated=False, geometry_modified=True, manufacturing_approved=False, purchase_specification_complete=False,
        insertion_sweep_executed=False, fixed_geometry_preserved=True,
        limitations=['Only nominal geometry and exact mass bookkeeping are generated; saved-STEP audits are separate.',
            'No real thread flanks, tolerances, tap accessibility, preload, locking, fatigue or thread-strip calculation.',
            'Machining blanks are pilot-only operation references, not finished assembly components.',
            'Pin/washer/screw share the original parent-body reference; passive spin/friction is not modeled.',
            'No tendon, motor, latency or hardware acceptance.'])
    out.mkdir()
    for name in ('parts', 'meshes', 'machining_blanks'): (out/name).mkdir()
    try:
        local = dict(pin=make_pin(cq), washer=make_washer(cq), screw=make_screw(cq))
        blank = make_pin(cq, machining_blank=True); oldpin = oldbuilder.make_pin(cq, 'round')
        for label, shape in {**local, 'pilot_blank': blank, 'baseline_pin_reconstruction': oldpin}.items(): _valid(shape, label)
        assembly = cq.Assembly(name='manufacturing_v2_screw_retained_distal_pins')
        newrows, newmasses = [], []
        joint_for_pin = {r['pin']: r for r in joints}
        def export_part(name, shape, body, role, color, oldrow=None, oldmass=None):
            check = _valid(shape, name); props = properties.mass_properties(shape, DENSITY)
            if not _finite_tree(props): raise ValueError('Nonfinite mass properties: '+name)
            step, mesh = out/'parts'/f'{name}.step', out/'meshes'/f'{name}.stl'
            cq.exporters.export(shape, str(step)); cq.exporters.export(shape, str(mesh), tolerance=.1, angularTolerance=.1)
            row = copy.deepcopy(oldrow) if oldrow is not None else {}
            row.update(name=name, body=body, role=role, color=list(color), mesh_units='mm',
                step=step.relative_to(HERE).as_posix(), sha256_step=digest(step),
                mesh=mesh.relative_to(HERE).as_posix(), sha256_stl=digest(mesh),
                source='manufacturing_v2_nominal_screw_retention', volume_mm3=props['volume_mm3'], bbox_mm=props['bbox_mm'],
                valid=True, solid_count=1, step_index=len(newrows))
            massrow = copy.deepcopy(oldmass) if oldmass is not None else dict(name=name, body=body, material='steel_assumption')
            massrow.update(role=role, **props)
            report['part_checks'].append(dict(name=name, body=body, **check)); newrows.append(row); newmasses.append(massrow)
            assembly.add(shape, name=name, color=cq.Color(*color[:3]))
            return row, massrow
        for index, row in enumerate(rows):
            name = row['name']; shape = cq.importers.importStep(str(sources[name]['step'])).val(); check = _valid(shape, name+' baseline')
            if abs(check['volume_mm3']-row['volume_mm3']) > max(1e-5, 1e-8*row['volume_mm3']): raise ValueError('Baseline STEP/scene volume differs: '+name)
            if name in removed:
                report['removed_parts'].append(dict(name=name, body=row['body'], source_step=row['step'], source_step_sha256=row['sha256_step'], old_mass_kg=masses[name]['mass_kg']))
            elif name in modified:
                joint = joint_for_pin[name]; frame = joint['frame']
                difference = oldbuilder.difference(shape, geometry._world(oldpin, frame), 1e-6, 1e-5)
                report['original_pin_reconstruction'].append(dict(name=name, **difference))
                if not difference['passed']: raise ValueError('Baseline native pin/frame reconstruction differs: '+name)
                changed = geometry._world(local['pin'], frame)
                newrow, newmass = export_part(name, changed, row['body'], 'slotted_floating_pin_M2_female_major_envelope', row['color'], row, masses[name])
                report['changed_parts'].append(dict(name=name, body=row['body'], frame=frame, old_step_index=row['step_index'], step_index=newrow['step_index'],
                    old_volume_mm3=row['volume_mm3'], new_volume_mm3=newrow['volume_mm3'], old_bbox_mm=row['bbox_mm'], new_bbox_mm=newrow['bbox_mm'],
                    old_mass_kg=masses[name]['mass_kg'], new_mass_kg=newmass['mass_kg'], source_step_sha256=row['sha256_step'],
                    step=newrow['step'], sha256_step=newrow['sha256_step'], body_preserved=True))
                blank_shape = geometry._world(blank, frame); _valid(blank_shape, name+' pilot blank')
                blank_path = out/'machining_blanks'/f'{name}_pilot_blank.step'; cq.exporters.export(blank_shape, str(blank_path))
                report['machining_blanks'].append(dict(name=name+'_pilot_blank', finished_part=name, assembly_part=False, included_in_mass=False,
                    step=blank_path.relative_to(HERE).as_posix(), sha256_step=digest(blank_path), frame=frame,
                    volume_mm3=float(blank_shape.Volume()), pilot_diameter_mm=1.6, drilling_span_x_mm=[7.15, 13.15],
                    drill_tip_x_mm=drill_tip_x(), mouth_diameter_mm=2.4, finished_thread_geometry=False,
                    meaning='Pilot-drilled/chamfered machining blank before tapping; not the assembled female thread envelope.'))
            else:
                newrow = copy.deepcopy(row)
                for key in ('step', 'mesh'): newrow[key] = Path(os.path.relpath(sources[name][key], HERE)).as_posix()
                newrow['step_index'] = len(newrows)
                if any(newrow[k] != row[k] for k in ('sha256_step','sha256_stl','body','volume_mm3','bbox_mm','role')): raise ValueError('Unchanged part record differs')
                newrows.append(newrow); newmasses.append(copy.deepcopy(masses[name])); report['part_checks'].append(dict(name=name, body=row['body'], **check))
                assembly.add(shape, name=name, color=cq.Color(*row['color'][:3]))
            if (index+1)%25 == 0 or index+1 == len(rows): print(json.dumps(dict(progress='baseline_parts_read', completed=index+1, total=349)), flush=True)
        for joint in joints:
            for kind in ('washer', 'screw'):
                name = joint[kind]; shape = geometry._world(local[kind], joint['frame'])
                role = 'DIN9021_M2_nominal_keeper_washer' if kind == 'washer' else 'DIN912_M2x4_nominal_major_envelope_socket_screw'
                row, massrow = export_part(name, shape, joint['parent_body'], role, [.55,.58,.62])
                report['added_parts'].append(dict(name=name, kind=kind, body=joint['parent_body'], frame=joint['frame'],
                    step_index=row['step_index'], step=row['step'], sha256_step=row['sha256_step'],
                    volume_mm3=row['volume_mm3'], mass_kg=massrow['mass_kg'], density_kg_per_mm3=DENSITY))
        expected_new = (set(oldnames)-set(removed)) | set(added)
        if len(newrows) != 359 or len({r['name'] for r in newrows}) != 359 or {r['name'] for r in newrows} != expected_new:
            raise ValueError('New assembly part coverage differs')
        if {r['name'] for r in report['changed_parts']} != set(modified) or {r['name'] for r in report['removed_parts']} != set(removed) or {r['name'] for r in report['added_parts']} != set(added):
            raise ValueError('Changed-name report is incomplete')
        if [r['step_index'] for r in newrows] != list(range(359)) or {r['name'] for r in newmasses} != expected_new:
            raise ValueError('Assembly indices or mass part coverage differ')
        candidate_design = copy.deepcopy(design)
        candidate_design['manufacturing_candidate'] = dict(name='manufacturing_v2', base_candidate=patch['base_candidate'],
            part_changes=part_changes, interpretation=patch['interpretation'], manufacturing_approved=False)
        without_meta = lambda value: {k:v for k,v in value.items() if k != 'manufacturing_candidate'}
        if without_meta(candidate_design) != without_meta(design): raise ValueError('Kinematic design changed')
        _write(out/'design.json', candidate_design); assembly.save(str(out/'hand.step'))
        guard = inputs.guard()
        if not guard['passed']: raise ValueError('Build inputs changed')
        newscene = dict(hand=newrows, arm_reference=[], source_sha256=dict(inputs.start), inputs_unchanged=True,
            base_candidate=patch['base_candidate'], part_changes=part_changes, interpretation=patch['interpretation'])
        newmass = copy.deepcopy(mass)
        newmass.update(parts=newmasses, source_sha256=dict(inputs.start), inputs_unchanged=True,
            total_mass_kg=sum(r['mass_kg'] for r in newmasses), moving_mass_kg=sum(r['mass_kg'] for r in newmasses if r['body'] != 'palm'),
            limitations=list(mass.get('limitations', []))+['Ten pilot machining blanks are excluded from assembly mass.',
                'Female/male threads use simplified major envelopes; mass and passive spin are not calibrated physical hardware values.'])
        _write(out/'scene.json', newscene); _write(out/'mass_inertia.json', newmass)
        report.update(completed=True, inputs_unchanged=True, source_sha256=dict(inputs.start), input_hash_guard=inputs.guard(),
            modified_part_count=10, removed_part_count=10, added_part_count=20, unchanged_part_count=329,
            exported_part_file_count=30, referenced_unchanged_part_count=329, exported_machining_blank_count=10,
            all_parts_valid_single_solid=True, kinematic_design_unchanged=True,
            assembly_step='generated/hand.step', assembly_step_sha256=digest(out/'hand.step'),
            generated_design='generated/design.json', generated_design_sha256=digest(out/'design.json'),
            total_mass_kg=newmass['total_mass_kg'], moving_mass_kg=newmass['moving_mass_kg'],
            baseline_total_mass_kg=mass['total_mass_kg'], baseline_moving_mass_kg=mass['moving_mass_kg'])
        if not report['input_hash_guard']['passed']: raise ValueError('Inputs changed before completion')
        _write(out/'build_report.json', report)
        print(json.dumps(dict(completed=True, parts=359, modified=10, removed=10, added=20, unchanged=329, machining_blanks=10, inputs_unchanged=True)), flush=True)
        return 0
    except Exception as exc:
        report.update(completed=False, inputs_unchanged=False, input_hash_guard=inputs.guard())
        report['errors'].append(dict(error=repr(exc), traceback=traceback.format_exc())); _write(out/'build_failure.json', report)
        raise


if __name__ == '__main__': raise SystemExit(main())

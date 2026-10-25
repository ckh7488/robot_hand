"""Build an isolated one-axis fixture from the committed thumb tendon CAD.

No change to accepted hand proportions/poses. No hardware operation. The
upstream thumb joints are absent from this test coupon, not locked in a hand.
"""
from pathlib import Path
import argparse
import traceback
import numpy as np
from common import CAD, SOURCE, BASE_SHA, PARENT, EXCLUDED, Inputs, digest, write_json, rigid, bounds, source_rows, cord_segments, validate_coverage


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    repo, out = a.repo.resolve(), a.output.resolve()
    if out.exists():
        raise FileExistsError('Preserve prior outputs')
    out.mkdir(parents=True)
    (out / 'parts').mkdir()
    inputs = Inputs(repo)
    report = dict(schema='thumb_ip_bench_build.v1', completed=False, errors=[],
                  source_sha256=inputs.hashes, baseline_sha=BASE_SHA,
                  manufacturing_approved=False, latency_measured=False)
    try:
        import cadquery as cq
        from cadquery.occ_impl.shapes import setThreads
        setThreads(1)
        inputs.bind(__file__)
        inputs.bind(repo / CAD / 'common.py')
        geo = inputs.module('_bench_geometry', repo / SOURCE / 'geometry.py')
        fixture = inputs.module('_bench_parent_fixture', repo / CAD / 'fixture.py')
        base = inputs.module('_bench_common_base', repo / CAD / 'base.py')
        for source in sorted((repo / CAD / 'source_docs').rglob('*')):
            if source.is_file():
                inputs.bind(source)
        hardware = inputs.read(repo / CAD / 'source_docs/facts.json')
        socket = inputs.read(repo / CAD / 'source_docs/wera_tool_facts.json')
        required_dimensions = dict(diameter_mm=hardware['socket_screws']['thread_diameter_mm'],
            pitch_mm=hardware['socket_screws']['thread_pitch_mm'],
            head_diameter_mm=hardware['socket_screws']['head_diameter_mm']['upper'],
            head_height_mm=hardware['socket_screws']['head_height_mm']['upper'],
            socket_across_flats_mm=hardware['socket_screws']['socket_across_flats_mm'],
            socket_depth_mm=hardware['socket_screws']['drive_depth_mm'],
            nut_across_flats_mm=hardware['nut']['across_flats_mm']['upper'],
            nut_height_mm=hardware['nut']['height_mm']['upper'],
            washer_od_mm=hardware['washer']['outer_diameter_mm']['upper'],
            washer_id_mm=hardware['washer']['inner_diameter_mm']['lower'],
            washer_thickness_mm=hardware['washer']['thickness_mm']['upper'])
        if any(abs(base.HARDWARE[k]-v)>1e-9 for k,v in required_dimensions.items()):
            raise ValueError('Base hardware dimensions differ from source facts')
        if socket['published_dimensions_mm'] != dict(hex_output_across_flats=7, overall_length=50,
                d1_outer_dimension=10.8, d2_rear_diameter=11.8):
            raise ValueError('Socket exterior dimensions changed')
        routing = inputs.module('_bench_display_routing', repo / SOURCE / 'routing.py')
        joint = inputs.module('_bench_display_joint', repo / SOURCE / 'joint_drive.py')
        rows, old_report = source_rows(inputs)
        inherited = list(rows)
        named = {r['name']: r for r in rows}
        def native(name):
            row = named[name]
            return rigid(cq, cq.importers.importStep(str(inputs.resolve_part(row))).val(), row['neutral_matrix'])
        parent = native(PARENT)
        supports = fixture.build(cq, parent)
        platform = base.build(cq, native('drive_bench_baseplate'), supports['foot_holes'])
        if [len(supports[k]) for k in ('parts','foot_holes','contacts','assembly')] != [8,4,12,8]:
            raise ValueError('Incomplete parent support declarations')
        if [len(platform[k]) for k in ('parts','contacts','assembly')] != [54,60,24]:
            raise ValueError('Incomplete platform declarations')
        def export(part):
            name, shape = part['name'], part['shape']
            facts = geo.require_solid(shape, name)
            step = out / 'parts' / (name + '.step')
            mesh = step.with_suffix('.stl')
            cq.exporters.export(shape, str(step))
            cq.exporters.export(shape, str(mesh), tolerance=.035, angularTolerance=.12)
            return dict(name=name, body=part.get('body', 'fixed'), role=part['role'],
                location='generated', origin_group='fixture', step=step.relative_to(out).as_posix(),
                mesh=mesh.relative_to(out).as_posix(), sha256_step=digest(step), sha256_stl=digest(mesh),
                neutral_matrix=np.eye(4).tolist(), color=part.get('color', [.4, .48, .6]),
                volume_mm3=facts['volume_mm3'], solid_count=1, native_bbox_mm=bounds(shape),
                bbox_coordinates='Native STEP coordinates; apply neutral_matrix for IP placement')
        modified = set(platform['replace_names'])
        if modified != {'drive_bench_baseplate'}:
            raise ValueError('Unexpected inherited part modification')
        rows = [r for r in rows if r['name'] not in modified]
        added = [*supports['parts'], *platform['parts']]
        rows.extend(export(p) for p in added)
        if len({r['name'] for r in rows}) != len(rows):
            raise ValueError('Duplicate physical part')
        scene = dict(schema='thumb_ip_bench_scene.v1', parts=rows,
            coordinates='IP local mm; +X revolute axis; motor centre (0,-120,0)',
            motor_to_ip_local_translation_mm=[-6., -120., 0.],
            motor_rotation_axis='+X', motor_rotation_per_ip_rotation=.77,
            hand_geometry_modified=False, detached_coupon=True, excluded_upstream_bushings=EXCLUDED,
            source_coupon_part_count=30, source_motor_part_count=33,
            modified_source_parts=sorted(modified),
            contacts=supports['contacts'] + platform['contacts'],
            assembly=supports['assembly'] + platform['assembly'],
            thread_zones_motor_frame=old_report['motor_interfaces']['declared_thread_overlap_regions'],
            central_horn_retainer_missing=True, tendon_preload_adjustment_complete=False,
            full_hand_integration=False)
        report['coverage'] = validate_coverage(scene, inherited)
        scene['display_cords'] = {}
        for q in (0., -15.):
            display = []
            for cord in cord_segments(cq, routing, joint, old_report['motor_interfaces']['anchors'], q):
                part = dict(cord, name=f'display_q{q:g}_'+cord['name'],
                            body='fixed', role='nominal_undeformed_cord_display', color=[.9,.26,.12])
                display.append(export(part))
            scene['display_cords'][str(q)] = display
        write_json(out / 'scene.json', scene)
        report.update(completed=True, part_count=len(rows), new_saved_parts=len(added),
            support_spec=supports['spec'], platform_spec=platform['spec'],
            modified_source_parts=sorted(modified), preserved_source_parts=len(rows)-len(added),
            generated_sha256={p.relative_to(out).as_posix(): digest(p) for p in sorted(out.rglob('*')) if p.is_file()},
            limitations=['Isolated one-axis coupon, not a full-hand fixture or motor layout.',
                'Nominal geometric fit only: material, torque, tolerances, friction and strength remain unqualified.',
                'Central metal-horn retainer, individual cord length/preload adjustment and joint sensing remain incomplete.',
                'No physical drive, measured delay or 50/100 ms performance approval.'])
    except Exception as error:
        report['errors'].append(dict(error=repr(error), traceback=traceback.format_exc()))
    finally:
        report['input_hash_guard'] = inputs.guard()
        if not report['input_hash_guard']['passed']:
            report['completed'] = False
        write_json(out / 'build_report.json', report)
    print({k: report[k] for k in ('completed', 'errors')}, flush=True)
    return 0 if report['completed'] and not report['errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

"""Inert audit of the separately saved short adjustment key.

run(cq, geo, rows, placed, scene, tool_native) -> result dictionary.
rows is name->physical row; placed(q,D) returns ALL158 actual physical shapes.
scene.assembly_tools contains one key row with its own metadata, neutral_matrix,
body/axis/pitch. tool_native is that hash-bound STEP reimport, not rebuilt CAD.
The caller binds all read files and this source. No import-time CAD or I/O.
"""
from __future__ import annotations

import math


NAME = 'tension_custom_adjustment_key'
TARGET = 'tension_adjuster_screw'
DISTANCES = (118.0, 120.0, 122.0)
FLOATS = (0.0, 0.1)
LIMIT = 1e-4
DIM_TOL = 1e-5


def run(cq, geo, rows, placed, scene, tool_native):
    """Fail closed on missing data, unknown scope or failed solid operations.

    Socket is INCLUDED in every actual-key insertion test. Only the rotation
    exterior envelope excludes it, justified separately by saved finite socket
    surfaces and the invariant screw-to-key rigid transform. Screw axial float
    moves the screw and tool only; the existing thrust washer is NOT moved.
    """
    result = dict(
        schema='thumb_ip_tension_key_access.v1', completed=False, passed=False,
        distances_mm=list(DISTANCES), q_ip_deg=0.0,
        screw_tool_axial_offsets_y_mm=list(FLOATS),
        material_overlap_limit_mm3=LIMIT, errors=[], cases=[],
        scope={
            'separate_tool_not_physical_assembly_part': True,
            'expected_physical_part_count': 158,
            'actual_insertion_maximum_step_mm': 0.5,
            'actual_insertion_continuous': False,
            'continuous_rotation_exterior': 'axisymmetric conservative envelope; only at declared D/float conditions',
            'socket_rotation_exception': 'separate actual finite socket/hex test plus relative rigid-transform invariance',
            'float_moved_physical_parts': [TARGET],
            'washer_moved': False,
            'whole_fixture_assembly_approved': False,
            'whole_hand_tendon_routing': False,
            'strength_grip_thread_preload_or_latency_approved': False,
        })
    try:
        # Runtime-only dependencies. No CAD, file I/O or import occurs above.
        import numpy as np
        from OCP.BRep import BRep_Tool
        from OCP.GeomLib import GeomLib_IsPlanarSurface
        from common import bounds, overlap, placed_matrix, rigid

        def finite(value, label):
            v = float(value)
            if not math.isfinite(v):
                raise ValueError('Nonfinite '+label)
            return v

        def close(a, b, tolerance=DIM_TOL):
            return abs(float(a)-float(b)) <= tolerance

        def vec(value, label):
            a = np.asarray(value, dtype=float)
            if a.shape != (3,) or not np.isfinite(a).all():
                raise ValueError('Invalid '+label)
            return a

        def checked_volume(shape):
            d = geo.describe(shape)
            v = finite(d['volume_mm3'], 'signed material volume')
            if not (d['valid'] and d['bounded'] and d['positive_solids']) or v < 0.0:
                raise ValueError('Invalid material result '+str(d))
            return v

        def common_volume(a, b):
            return checked_volume(geo.boolean(cq, 'common', a, b))

        def difference_volume(a, b):
            return checked_volume(geo.boolean(cq, 'cut', a, b))

        def hex_wire(af, y):
            r = af/math.sqrt(3.0)
            vv = [cq.Vector(-21.0+r*math.cos(i*math.pi/3.0), y,
                             -20.5+r*math.sin(i*math.pi/3.0)) for i in range(6)]
            return cq.Wire.makePolygon(vv+[vv[0]])

        def hex_prism(af, y0, y1):
            return cq.Solid.extrudeLinear(hex_wire(af, y0), [], cq.Vector(0, y1-y0, 0))

        def slab(y0, y1):
            return cq.Solid.makeBox(20.0, y1-y0, 20.0, cq.Vector(-31.0, y0, -30.5))

        def plane_data(face):
            fit = GeomLib_IsPlanarSurface(BRep_Tool.Surface_s(face.wrapped), 1e-7)
            if not fit.IsPlanar():
                return None
            plane = fit.Plan()
            p, n = plane.Location(), plane.Axis().Direction()
            return (np.array([p.X(), p.Y(), p.Z()]),
                    np.array([n.X(), n.Y(), n.Z()]))

        def face_common_area(a, b):
            hit = geo.boolean(cq, 'common', a, b)
            area = finite(hit.Area(), 'face area')
            if not hit.isValid() or area < 0.0:
                raise ValueError('Invalid finite face common')
            return area

        def finite_hex_faces(shape, af, y0, y1):
            """Match six real planar surfaces and cover the full finite patches."""
            prism = hex_prism(af, y0, y1)
            expected = []
            for face in prism.Faces():
                data = plane_data(face)
                if data is not None and abs(data[1][1]) < 1e-7:
                    expected.append((face, data))
            if len(expected) != 6:
                raise ValueError('Reference finite hex did not have six side faces')
            actual = [(f, plane_data(f)) for f in shape.Faces()]
            rows_out = []
            for patch, (p, n) in expected:
                area, match_count, offsets = 0.0, 0, []
                for face, data in actual:
                    if data is None:
                        continue
                    pp, nn = data
                    if (abs(float(np.dot(n, nn))) < 1.0-1e-7 or
                            abs(float(np.dot(pp-p, n))) > DIM_TOL):
                        continue
                    area += face_common_area(face, patch)
                    match_count += 1
                    offsets.append(abs(float(np.dot(pp-np.array([-21., y0, -20.5]), nn))))
                target_area = patch.Area()
                rows_out.append(dict(expected_area_mm2=target_area, actual_common_area_mm2=area,
                    matching_actual_faces=match_count, measured_axis_to_plane_mm=offsets,
                    finite_span_y_mm=[y0, y1], passed=area >= target_area-1e-5))
            return dict(across_flats_mm=af, faces=rows_out,
                        passed=len(rows_out)==6 and all(v['passed'] for v in rows_out))

        def floor_patch(shape):
            patch = cq.Face.makeFromWires(hex_wire(3.0, -71.0))
            area, count = 0.0, 0
            for face in shape.Faces():
                data = plane_data(face)
                if data is None:
                    continue
                p, n = data
                if abs(n[1]) > 1.0-1e-7 and abs(p[1]+71.0) < DIM_TOL:
                    value = face_common_area(face, patch)
                    if value > 1e-8:
                        area += value
                        count += 1
            required = patch.Area()
            return dict(plane_y_mm=-71.0, actual_common_area_mm2=area,
                expected_patch_area_mm2=required, actual_faces=count,
                passed=area >= required-1e-5)

        if not isinstance(rows, dict) or len(rows) != 158:
            raise ValueError('Expected explicit158 physical rows')
        if (not isinstance(scene.get('parts'), list) or len(scene['parts']) != 158 or
                {r['name'] for r in scene['parts']} != set(rows)):
            raise ValueError('Scene physical coverage mismatch')
        if NAME in rows or TARGET not in rows:
            raise ValueError('Separate key was added to physical rows or screw missing')
        tools = scene.get('assembly_tools')
        if not isinstance(tools, list) or len(tools) != 1 or tools[0].get('name') != NAME:
            raise ValueError('Expected exactly one declared separate adjustment key')
        tool_row = tools[0]
        meta = tool_row.get('metadata')
        if not isinstance(meta, dict) or meta.get('schema') != 'thumb_ip_tension_custom_adjustment_key.v1':
            raise ValueError('Missing/unknown saved key metadata')
        if meta.get('included_in_physical_assembly') is not False or meta.get('included_in_assembly_mass') is not False:
            raise ValueError('Key must be excluded from physical assembly and mass')
        if meta.get('phase_reference_part') != TARGET:
            raise ValueError('Incorrect key phase reference')
        for label, row in [('tool', tool_row), ('screw', rows[TARGET])]:
            if row.get('body') != 'adjuster_rotor':
                raise ValueError(label+' does not share the adjuster rotor frame')
            if not np.allclose(vec(row.get('adjuster_axis'), label+' axis'), [-21.,-73.,-20.5], atol=1e-9, rtol=0):
                raise ValueError(label+' axis changed')
            if not close(row.get('travel_mm_per_revolution', float('nan')), -.7, 1e-12):
                raise ValueError(label+' signed pitch changed')
            if not np.allclose(vec(row.get('adjuster_axis_direction', [0.,1.,0.]), label+' axis direction'), [0.,1.,0.], atol=1e-9, rtol=0):
                raise ValueError(label+' axis direction changed')
        expected_parameters = dict(axis_x_mm=-21., axis_z_mm=-20.5,
            axis_reference_y_mm=-73., socket_front_y_mm=-69., socket_depth_mm=2.,
            socket_across_flats_mm=3., tip_y_mm=-70.5, hex_across_flats_mm=2.9,
            tip_lead_length_mm=.2, tip_lead_angle_deg=45., knob_rear_y_mm=-59.,
            knob_front_y_mm=-53., knob_diameter_mm=14., entry_translation_mm=12.,
            signed_travel_mm_per_revolution=-.7)
        p = meta.get('parameters', {})
        if any(k not in p or not close(p[k], v, 1e-9) for k,v in expected_parameters.items()):
            raise ValueError('Key dimensions changed; explicit checker review required')
        declared_path = meta.get('assembly', [])
        if (len(declared_path) != 1 or declared_path[0].get('kind') != 'sampled_translation' or
                declared_path[0].get('part') != NAME or declared_path[0].get('stage_excluded') != [] or
                declared_path[0].get('initial_translation_mm') != [0.,12.,0.] or
                declared_path[0].get('final_translation_mm') != [0.,0.,0.] or
                not close(declared_path[0].get('maximum_step_mm', float('nan')), .5)):
            raise ValueError('Insertion declaration differs from bounded scope')

        native_description = geo.require_solid(tool_native, 'actual saved separate key')
        neutral_tool = rigid(cq, tool_native, placed_matrix(tool_row, 0., 120.))
        neutral_parts = placed(0., 120.)
        if not isinstance(neutral_parts, dict) or set(neutral_parts) != set(rows):
            raise ValueError('placed() physical coverage mismatch')
        neutral_screw = neutral_parts[TARGET]
        geo.require_solid(neutral_screw, 'actual saved target screw')
        key_box, screw_box = bounds(neutral_tool), bounds(neutral_screw)
        if (not close(key_box[1], -70.5) or not close(key_box[4], -53.) or
                not close(screw_box[4], -69.)):
            raise ValueError('Actual key end or screw head-front coordinate differs')

        # A swept exterior may be deliberately conservative around the nose.
        stem = cq.Solid.makeCylinder(2.9/math.sqrt(3.0), 11.7,
            cq.Vector(-21.,-70.5,-20.5), cq.Vector(0.,1.,0.))
        wheel = cq.Solid.makeCylinder(7., 6., cq.Vector(-21.,-59.,-20.5), cq.Vector(0.,1.,0.))
        envelope = geo.boolean(cq, 'fuse', stem, wheel)
        geo.require_solid(envelope, 'axisymmetric key exterior envelope')
        outside = difference_volume(neutral_tool, envelope)
        result['saved_tool_envelope_containment'] = dict(native=native_description,
            actual_bounds_mm=key_box, outside_material_mm3=outside,
            passed=outside <= LIMIT, cylinder_axis=[-21.,-73.,-20.5],
            stem_radius_mm=2.9/math.sqrt(3.), stem_span_y_mm=[-70.5,-58.8],
            knob_radius_mm=7., knob_span_y_mm=[-59.,-53.])

        key_faces = finite_hex_faces(neutral_tool, 2.9, -70.3, -69.0)
        socket_faces = finite_hex_faces(neutral_screw, 3.0, -71.0, -69.0)
        socket_floor = floor_patch(neutral_screw)
        complete_hex = hex_prism(2.9, -70.3, -69.0)
        actual_hex_strip = geo.boolean(cq, 'common', neutral_tool, slab(-70.3,-69.0))
        missing_hex = difference_volume(complete_hex, neutral_tool)
        extra_hex = difference_volume(actual_hex_strip, complete_hex)
        socket_material = common_volume(neutral_screw, hex_prism(3.0,-71.0,-69.0))
        target_overlap = common_volume(neutral_tool, neutral_screw)
        finite_fit = dict(key_full_hex_faces=key_faces, socket_faces=socket_faces,
            actual_socket_floor=socket_floor,
            full_hex_missing_material_mm3=missing_hex, full_hex_extra_material_mm3=extra_hex,
            socket_space_material_mm3=socket_material, actual_key_target_overlap_mm3=target_overlap,
            actual_tip_y_mm=key_box[1], actual_head_front_y_mm=screw_box[4],
            total_tip_penetration_mm=screw_box[4]-key_box[1],
            nominal_full_hex_engagement_mm=1.3, actual_tip_to_floor_mm=key_box[1]+71.,
            nominal_per_flat_gap_mm=.05,
            supplier_fit_and_tolerance_approved=False,
            passed=(key_faces['passed'] and socket_faces['passed'] and socket_floor['passed'] and
                    max(missing_hex,extra_hex,socket_material,target_overlap) <= LIMIT and
                    close(screw_box[4]-key_box[1],1.5) and close(key_box[1]+71.,.5)))
        result['actual_finite_socket_fit'] = finite_fit

        gap_groups = {
            'head_cap': ['tension_adjuster_head_cap'],
            'common_plate': ['bench_common_plate'],
            'parent_shell': ['thumb_segment2_segment_shell'],
            'parent_saddles': sorted(n for n,r in rows.items() if r.get('role')=='custom_split_saddle'),
        }
        if len(gap_groups['parent_saddles']) != 4 or any(n not in rows for ns in gap_groups.values() for n in ns):
            raise ValueError('Missing explicit finite-gap geometry')

        def scan(shape, parts, excluded=()):
            excluded = set(excluded)
            if not excluded <= set(parts):
                raise ValueError('Unknown collision exemption')
            box = bounds(shape)
            hits, exact = [], 0
            for name, other in parts.items():
                if name in excluded:
                    continue
                if overlap(box, bounds(other)):
                    exact += 1
                    value = common_volume(shape, other)
                    if value > LIMIT:
                        hits.append(dict(obstacle=name, overlap_mm3=value))
            return dict(tested_part_count=len(parts)-len(excluded), exact_pair_count=exact,
                excluded=sorted(excluded), collisions=hits, completed=True, passed=not hits)

        def finite_gaps(shape, parts):
            groups = {}
            for group, names in gap_groups.items():
                distances = []
                for name in names:
                    value = finite(shape.distance(parts[name]), 'actual finite part gap')
                    if value < 0.0:
                        raise ValueError('Negative distance')
                    distances.append(dict(part=name, distance_mm=value))
                groups[group] = dict(values=distances, minimum_mm=min(d['distance_mm'] for d in distances))
            return groups

        target_reference = placed_matrix(rows[TARGET], 0., 120.)
        key_reference = placed_matrix(tool_row, 0., 120.)
        relative_reference = np.linalg.inv(target_reference) @ key_reference
        relative_rows = []
        for distance in DISTANCES:
            originals = placed(0., distance)
            if not isinstance(originals, dict) or set(originals) != set(rows):
                raise ValueError('placed() omitted or added physical shapes')
            key_matrix = placed_matrix(tool_row, 0., distance)
            target_matrix = placed_matrix(rows[TARGET], 0., distance)
            relative = np.linalg.inv(target_matrix) @ key_matrix
            relative_error = float(np.max(np.abs(relative-relative_reference)))
            relative_rows.append(dict(distance_mm=distance, max_matrix_error=relative_error,
                rotation_deg=(120.-distance)/(-.7)*360., passed=relative_error <= 1e-9))
            phased_key = rigid(cq, tool_native, key_matrix)
            # This is the audit's passed mapping of the actual target, not a
            # rebuilt ideal screw. Validate its rigid correspondence as well.
            actual_relative_screw = rigid(cq, neutral_screw, target_matrix @ np.linalg.inv(target_reference))
            target_forward_error = max(difference_volume(actual_relative_screw, originals[TARGET]),
                                       difference_volume(originals[TARGET], actual_relative_screw))
            if target_forward_error > LIMIT:
                raise ValueError('placed target geometry differs from declared rigid frame')
            for offset in FLOATS:
                parts = dict(originals)
                parts[TARGET] = originals[TARGET].translate((0.,offset,0.))
                key = phased_key.translate((0.,offset,0.))
                geo.require_solid(key, 'positioned saved key')
                geo.require_solid(parts[TARGET], 'positioned target screw')
                case = dict(distance_mm=distance, q_ip_deg=0., axial_offset_y_mm=offset,
                    moved_physical_parts=[TARGET], existing_washer_kept_nominal=True,
                    target_rigid_correspondence_error_mm3=target_forward_error,
                    insertion_samples=[], completed=False, passed=False)
                result['cases'].append(case)
                # Other physical parts remain at their nominal placed values.
                case['moved_screw_against_others'] = scan(parts[TARGET], parts, [TARGET])
                for i in range(25):
                    displacement = 12.0-i*.5
                    entering = key.translate((0.,displacement,0.))
                    check = scan(entering, parts)
                    check.update(sample=i, added_y_mm=displacement,
                                 finite_gaps=finite_gaps(entering, parts))
                    case['insertion_samples'].append(check)
                case['insertion_scope'] = dict(samples=25, expected_tests=25*158,
                    actual_tests=sum(c['tested_part_count'] for c in case['insertion_samples']),
                    maximum_step_mm=.5, continuous=False, target_included=True,
                    physical_parts_included=sorted(parts), exclusions=[])
                if case['insertion_scope']['actual_tests'] != 25*158:
                    raise ValueError('Incomplete actual-key insertion coverage')
                rotated_envelope = envelope.translate((0.,offset,0.))
                containment_here = difference_volume(key, rotated_envelope)
                case['rotation_exterior'] = scan(rotated_envelope, parts, [TARGET])
                case['rotation_exterior'].update(
                    conservative_axisymmetric_continuous=True,
                    actual_key_outside_envelope_mm3=containment_here,
                    actual_containment_passed=containment_here <= LIMIT,
                    exception_justification='target socket handled by actual finite fit and identical-phase relative-transform proof')
                case['rotation_exterior']['passed'] &= containment_here <= LIMIT
                case['minimum_finite_gaps_over_insertion_samples'] = {
                    group: min(v['finite_gaps'][group]['minimum_mm'] for v in case['insertion_samples'])
                    for group in gap_groups}
                case['completed'] = True
                case['passed'] = (case['moved_screw_against_others']['passed'] and
                    all(v['passed'] for v in case['insertion_samples']) and
                    case['rotation_exterior']['passed'] and relative_error <= 1e-9)
                print('Saved short-key access D='+str(distance)+' float='+str(offset)+
                      ' completed, passed='+str(case['passed']), flush=True)
        result['relative_rotation_invariance'] = dict(
            shared_axis_mm=[-21.,-73.,-20.5], direction=[0.,1.,0.], signed_pitch_mm=-.7,
            reference_screw_to_key_matrix=relative_reference.tolist(), selected_distances=relative_rows,
            continuous_relative_invariance_basis='Same rigid +Y rotation map, same axis and pitch; common axial translation commutes with this rotation. No independent key phase is introduced.',
            physical_torque_transfer_or_socket_backlash_proven=False,
            passed=all(r['passed'] for r in relative_rows))
        result['coverage'] = dict(physical_parts=158, separate_tools=1, cases=6,
            insertion_samples_per_case=25, actual_key_pair_tests=6*25*158,
            rotation_envelope_pair_tests=6*157, shifted_screw_pair_tests=6*157,
            socket_fit_reference_distance_mm=120., source_hashes_bound_by_caller=True)
        result['completed'] = len(result['cases'])==6 and all(c['completed'] for c in result['cases'])
        result['passed'] = (result['completed'] and
            result['saved_tool_envelope_containment']['passed'] and finite_fit['passed'] and
            result['relative_rotation_invariance']['passed'] and all(c['passed'] for c in result['cases']))
    except Exception as error:
        result['errors'].append(repr(error))
        result['completed'] = False
        result['passed'] = False
    return result

"""Inert saved-BRep checks for the tension-stage supplementary mechanisms.

Only run() performs geometry. It reads no files, changes no input metadata or
source shape, exports nothing and launches no process. Caller supplies reopened,
hash-bound geometry through placed(q,D), and binds this source in its audit.
Results distinguish completed calculations from passed geometric criteria.

Version2 adds the travelling nut and its actual plate+lid pocket compound.
Scope: qIP=0 at the requested D samples; nominal supplier-size geometry. Float
vertices and angular samples are not continuous tolerance, wear, load, screw
preload, full assembly, or powered-operation approval.
"""
from __future__ import annotations

import copy
import math
from collections import Counter


EXPECTED_KINDS = {
    'conical_seat': 2, 'keyed_nut_pocket': 3, 'axial_float': 2,
    'correlated_axial_float': 1, 'carriage_stop': 2,
}


def run(cq, geo, rows, placed, scene, distances):
    """Return one result per declaration; absent/unknown/invalid data fails closed.

    rows is name -> physical scene row. placed(0.,D) must return ALL actual saved
    physical shapes in current IP-local coordinates. No display cords/blanks
    belong in this mapping. The caller independently checks the cord paths.
    This implementation deliberately validates the current ten-declaration
    schema; extending a future candidate requires explicit checker review.
    """
    # These imports occur only inside the invoked audit, never during import.
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Cylinder
    from checks import LIMIT, contact_area, declared_in_frame
    from common import bounds, overlap

    results = []
    cache = {}

    def finite(value, label):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('Nonfinite '+label)
        return value

    def vector(value, label):
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError('Invalid vector '+label)
        return [finite(v, label) for v in value]

    def norm(v):
        return math.sqrt(sum(x*x for x in v))

    def dot(a, b):
        return sum(x*y for x, y in zip(a, b))

    def sub(a, b):
        return [x-y for x, y in zip(a, b)]

    def plus(a, b):
        return [x+y for x, y in zip(a, b)]

    def xyz(p):
        return [p.X(), p.Y(), p.Z()]

    def unit(value, label):
        v = vector(value, label)
        if abs(norm(v)-1.) > 1e-9:
            raise ValueError('Nonunit '+label)
        return v

    def same(a, b, tolerance=1e-9):
        return len(a) == len(b) and all(abs(x-y) <= tolerance for x, y in zip(a, b))

    def frame_of(d):
        value = d.get('access_frame', d.get('contact_frame', d.get('frame', 'fixed')))
        if value not in ('fixed', 'carriage'):
            raise ValueError('Unsupported supplemental frame '+str(value))
        return value

    def framed_point(point, d, distance):
        result = vector(point, 'declared point')
        if frame_of(d) == 'carriage':
            result[1] += 120.-distance
        return result

    def require_names(names):
        if not names or len(names) != len(set(names)) or any(n not in rows for n in names):
            raise ValueError('Missing, duplicate or unknown physical part names: '+str(names))

    def parts_at(distance):
        if distance not in cache:
            value = placed(0., distance)
            if not isinstance(value, dict) or set(value) != set(rows):
                raise ValueError('placed() omitted or added physical parts')
            cache[distance] = value
        return cache[distance]

    def material(a, b):
        common = geo.boolean(cq, 'common', a, b)
        facts = geo.describe(common)
        value = finite(facts['volume_mm3'], 'intersection volume')
        if not facts['valid'] or not facts['bounded'] or value < -1e-9:
            raise ValueError('Invalid intersection geometry '+str(facts))
        return value, common

    def collision_sample(parts, transforms):
        """Every moved part against every other actual part, including each other."""
        require_names(list(transforms))
        changed = dict(parts)
        for name, transform in transforms.items():
            moved = transform(parts[name])
            geo.require_solid(moved, 'supplemental '+name)
            changed[name] = moved
        boxes = {name: bounds(shape) for name, shape in changed.items()}
        names = sorted(changed)
        affected = set(transforms)
        checked = exact = 0
        hits, errors = [], []
        for i, a in enumerate(names):
            for b in names[i+1:]:
                if a not in affected and b not in affected:
                    continue
                checked += 1
                if not overlap(boxes[a], boxes[b]):
                    continue
                exact += 1
                try:
                    value, _ = material(changed[a], changed[b])
                    if value > LIMIT:
                        hits.append(dict(pair=[a, b], overlap_mm3=value))
                except Exception as error:
                    errors.append(dict(pair=[a, b], error=repr(error)))
        expected = len(affected)*(2*len(changed)-len(affected)-1)//2
        if checked != expected:
            raise ValueError('Incomplete supplemental collision coverage')
        return changed, dict(moved_parts=sorted(affected), included_parts=names,
            tested_pair_count=checked, expected_pair_count=expected, exact_pair_count=exact,
            collisions=hits, errors=errors, completed=not errors,
            passed=not hits and not errors, same_body_pairs_included=True,
            contact_exemptions=[], material_overlap_limit_mm3=LIMIT)

    def contact(parts, declaration, distance, frame):
        d = copy.deepcopy(declaration)
        d['contact_frame'] = frame
        d.pop('access_frame', None)
        result = contact_area(cq, geo, parts, declared_in_frame(d, distance))
        result.update(completed=True, distance_mm=distance, q_deg=0.)
        return result

    def finite_cone_check(d, distance):
        a, b = d['a'], d['b']
        require_names([a, b])
        direction = unit(d['axis_direction'], 'cone axis')
        if not same(direction, [0., 0., 1.]):
            raise ValueError('finite_span_z_mm currently requires +Z cone axis')
        origin = framed_point(d['axis_origin_mm'], d, distance)
        radii = [finite(v, 'cone radius') for v in d['radii_mm']]
        span = [finite(v, 'cone finite span') for v in d['finite_span_z_mm']]
        minimum = finite(d['minimum_area_mm2'], 'minimum cone area')
        if len(radii) != 2 or len(span) != 2 or not (0 < radii[0] < radii[1] and span[0] < span[1] and minimum > 0):
            raise ValueError('Invalid finite cone dimensions')
        expected_angle = math.atan((radii[1]-radii[0])/(span[1]-span[0]))
        parts = parts_at(distance)
        selected = {}
        descriptions = {}
        for name in (a, b):
            selected[name], descriptions[name] = [], []
            for index, face in enumerate(parts[name].Faces()):
                surface = BRepAdaptor_Surface(face.wrapped, True)
                if surface.GetType() != GeomAbs_Cone:
                    continue
                cone = surface.Cone()
                apex, axis = xyz(cone.Apex()), xyz(cone.Axis().Direction())
                if abs(dot(axis, direction)) < 1.-1e-8:
                    continue
                delta = sub(apex, origin)
                perpendicular = sub(delta, [dot(delta, direction)*v for v in direction])
                angle = abs(cone.SemiAngle())
                if norm(perpendicular) > 1e-5 or abs(angle-expected_angle) > 1e-7:
                    continue
                actual_radii = [abs(z-apex[2])*math.tan(angle) for z in span]
                if not same(actual_radii, radii, 1e-5):
                    continue
                box = bounds(face)
                if box[2] > span[0]+1e-5 or box[5] < span[1]-1e-5:
                    continue
                selected[name].append((face, apex, angle))
                descriptions[name].append(dict(face_index=index, apex_mm=apex,
                    axis_direction=axis, semi_angle_deg=math.degrees(angle),
                    radii_at_declared_z_mm=actual_radii, face_bbox_mm=box))
            if not selected[name]:
                raise ValueError('No actual finite CONE face matching '+name)
        r = radii[1]+.01
        clip = cq.Solid.makeBox(2*r, 2*r, span[1]-span[0],
                    cq.Vector(origin[0]-r, origin[1]-r, span[0]))
        area = 0.
        records = []
        for fa, apex_a, angle_a in selected[a]:
            for fb, apex_b, angle_b in selected[b]:
                if norm(sub(apex_a, apex_b)) > 1e-5 or abs(angle_a-angle_b) > 1e-7:
                    continue
                shared = geo.boolean(cq, 'common', fa, fb)
                if not shared.Faces():
                    continue
                clipped = geo.boolean(cq, 'common', shared, clip)
                value = finite(clipped.Area(), 'finite conical common area')
                if value > 1e-7:
                    area += value
                    records.append(dict(actual_area_mm2=value))
        volume, _ = material(parts[a], parts[b])
        return dict(distance_mm=distance, q_deg=0., faces=descriptions,
            finite_span_z_mm=span, actual_common_area_mm2=area,
            face_pair_areas=records, minimum_area_mm2=minimum,
            solid_overlap_mm3=volume, completed=True,
            passed=bool(records) and area+1e-5 >= minimum and volume <= LIMIT,
            scope='Coincident actual analytic CONE faces clipped to the declared finite axial interval; no ideal replacement cone or normal-plane projection.')

    def keyed_scan(d):
        name, pocket = d['nut'], d['pocket_part']
        additional = d.get('additional_pocket_parts', [])
        if not isinstance(additional, list):
            raise ValueError('additional_pocket_parts must be an explicit list')
        pocket_names = [pocket, *additional]
        require_names([name, *pocket_names])
        body = rows[name]['body']
        if body not in ('fixed', 'carriage') or any(rows[n]['body'] != body for n in pocket_names):
            raise ValueError('Nut and every pocket part must share fixed or carriage body')
        if frame_of(d) != body:
            raise ValueError('Keyed pocket declaration frame does not match its physical body')
        # The caller's audit checks actual saved-shape placement and same-body
        # rigidity before using this function. Also reject incompatible frame
        # maps here, without claiming a repeated BRep at each nut angle/D.
        import numpy as np
        from common import placed_matrix
        frame_checks = []
        for q, distance in ((0.,118.), (0.,120.), (0.,122.), (15.,121.)):
            expected = np.eye(4)
            if body == 'carriage':
                expected[1,3] = 120.-distance
            errors = {}
            for part_name in [name, *pocket_names]:
                neutral = np.asarray(rows[part_name]['neutral_matrix'], dtype=float)
                if (neutral.shape != (4,4) or not np.isfinite(neutral).all() or
                        not np.allclose(neutral[3], [0,0,0,1], atol=1e-9, rtol=0) or
                        not np.allclose(neutral[:3,:3].T@neutral[:3,:3], np.eye(3), atol=1e-9, rtol=0) or
                        abs(np.linalg.det(neutral[:3,:3])-1.) > 1e-9):
                    raise ValueError('Nonrigid keyed-pocket neutral matrix: '+part_name)
                delta = placed_matrix(rows[part_name], q, distance) @ np.linalg.inv(neutral)
                error = float(np.max(np.abs(delta-expected)))
                errors[part_name] = error
                if error > 1e-9:
                    raise ValueError('Same-body keyed-pocket placement is not invariant: '+part_name)
            frame_checks.append(dict(q_deg=q, distance_mm=distance,
                max_transform_error_by_part=errors, passed=True))
        point = vector(d['axis_origin_mm'], 'nut axis origin')
        axis = unit(d['axis_direction'], 'nut axis direction')
        if not same(axis, [0., 1., 0.]):
            raise ValueError('finite_span_y_mm currently requires +Y')
        span = [finite(v, 'nut finite span') for v in d['finite_span_y_mm']]
        af, pocket_af = finite(d['nut_af_mm'], 'nut AF'), finite(d['pocket_af_mm'], 'pocket AF')
        if len(span) != 2 or span[0] >= span[1] or not 0 < af < pocket_af:
            raise ValueError('Invalid nut dimensions')
        if name == 'tension_adjuster_travelling_nut':
            if (body != 'carriage' or pocket_names != ['drive_bench_baseplate','tension_adjuster_nut_lid'] or
                    not same(point, [-21.,-91.3,-20.5]) or not same(span, [-91.3,-88.1]) or
                    abs(af-7.) > 1e-9 or abs(pocket_af-7.2) > 1e-9):
                raise ValueError('Unexpected travelling-nut capture geometry/frame contract')
        parts = parts_at(120.)
        shape = parts[name]
        geo.require_solid(shape, 'saved keyed nut '+name)
        for part_name in pocket_names:
            geo.require_solid(parts[part_name], 'saved keyed pocket member '+part_name)
        # This compound is assembled actual saved material, not an invented
        # rectangular/hex cavity or a replacement for either separate part.
        wall = (parts[pocket] if len(pocket_names)==1 else
                cq.Compound.makeCompound([parts[n] for n in pocket_names]))
        bb = bounds(shape)
        if abs(bb[1]-span[0]) > 1e-5 or abs(bb[4]-span[1]) > 1e-5:
            raise ValueError('Actual nut finite axial extent differs from declaration')
        bore_faces = []
        for index, face in enumerate(shape.Faces()):
            ad = BRepAdaptor_Surface(face.wrapped, True)
            if ad.GetType() != GeomAbs_Cylinder:
                continue
            cylinder = ad.Cylinder()
            direction, location = xyz(cylinder.Axis().Direction()), xyz(cylinder.Location())
            delta = sub(location, point)
            off = sub(delta, [dot(delta, axis)*v for v in axis])
            fbb = bounds(face)
            if (abs(cylinder.Radius()-2.) < 1e-5 and abs(dot(direction, axis)) > 1.-1e-8
                    and norm(off) < 1e-5 and fbb[1] <= span[0]+1e-5 and fbb[4] >= span[1]-1e-5):
                bore_faces.append(index)
        if not bore_faces:
            raise ValueError('Actual finite M4 major-bore cylinder does not establish the declared rotation axis')
        nominal, _ = material(shape, wall)
        capture = cq.Solid.makeCylinder(af/math.sqrt(3.)+.001, span[1]-span[0],
                    cq.Vector(point[0], span[0], point[2]), cq.Vector(*axis))
        end = plus(point, axis)
        samples = {}

        def measure(angle):
            key = round(angle, 12)
            if key not in samples:
                moved = shape.rotate(point, end, angle)
                volume, hit = material(moved, wall)
                outside = 0.
                if volume > LIMIT:
                    outside = finite(geo.describe(geo.boolean(cq, 'cut', hit, capture))['volume_mm3'], 'key overlap outside finite pocket')
                samples[key] = dict(rotation_deg=angle, overlap_mm3=volume,
                    outside_declared_capture_mm3=outside, blocked=volume > LIMIT)
            return samples[key]

        limits = []
        for sign in (-1., 1.):
            endpoint = measure(sign*30.)
            lower, upper = 0., None
            # First threshold crossing in sampled order, not an assumed
            # globally monotone collision response over arbitrary rotations.
            for degree in range(1, 31):
                item = measure(sign*degree)
                if item['blocked']:
                    upper = float(degree)
                    break
                lower = float(degree)
            if upper is not None:
                for _ in range(12):
                    middle = (lower+upper)/2.
                    if measure(sign*middle)['blocked']:
                        upper = middle
                    else:
                        lower = middle
            limits.append(dict(direction_sign=sign, endpoint_30deg=endpoint,
                last_sample_below_limit_deg=sign*lower,
                first_sample_above_limit_deg=None if upper is None else sign*upper,
                bracket_width_deg=None if upper is None else upper-lower,
                passed=endpoint['blocked'] and upper is not None and endpoint['outside_declared_capture_mm3'] <= LIMIT))
        outside_ok = all(v['outside_declared_capture_mm3'] <= LIMIT for v in samples.values())
        return dict(q_deg=0., distance_mm=120., body=body, declaration_frame=frame_of(d),
            actual_pocket_parts=pocket_names,
            reused_same_body_pair_at_other_distances=True,
            reused_fixed_pair_at_other_distances=(body=='fixed'),
            same_body_transform_checks=frame_checks,
            caller_saved_shape_same_body_rigid_proof_required=True,
            actual_axis_bore_face_indices=bore_faces, nominal_overlap_mm3=nominal,
            angle_scan_step_deg=1., bisection_iterations=12, directional_limits=limits,
            angular_samples=list(samples.values()), completed=True,
            passed=nominal <= LIMIT and all(v['passed'] for v in limits) and outside_ok,
            overlap_threshold_mm3=LIMIT,
            scope='Centre-fixed Y rotation of the saved nut against its actual saved pocket member(s), at neutral D120; first material-overlap threshold bracket only. Other D reuse requires the same-body rigid placement proof and does not include nut lift, axial shift, tilt, tolerance, exact zero-contact angle, continuous free rotation, wrench access or torque capacity.')

    try:
        if not isinstance(rows, dict) or not rows:
            raise ValueError('Missing physical row map')
        names = [r['name'] for r in scene['parts']]
        if len(names) != len(set(names)) or set(names) != set(rows):
            raise ValueError('Scene/row map mismatch')
        declarations = scene['supplemental_checks']
        if not isinstance(declarations, list) or Counter(d.get('kind') for d in declarations) != Counter(EXPECTED_KINDS):
            raise ValueError('Missing/unknown/duplicate supplemental kinds for current ten-declaration schema')
        keyed_names = {d.get('nut') for d in declarations if d.get('kind')=='keyed_nut_pocket'}
        if keyed_names != {'tension_adjuster_head_cap_nut_1', 'tension_adjuster_head_cap_nut_2',
                           'tension_adjuster_travelling_nut'}:
            raise ValueError('Missing or substituted named keyed nut')
        ds = [finite(v, 'audit distance') for v in distances]
        if len(ds) != len(set(ds)) or set(ds) != {118., 120., 122.}:
            raise ValueError('Supplemental suite requires explicit D118/120/122')
        identities = []
        for d in declarations:
            kind = d['kind']
            identity = (kind, d.get('a'), d.get('b')) if kind == 'conical_seat' else (
                (kind, d.get('nut')) if kind == 'keyed_nut_pocket' else
                (kind, d.get('carriage_y_mm')) if kind == 'carriage_stop' else
                (kind, d.get('part'), tuple(d.get('group_parts', []))))
            identities.append(identity)
        if len(set(identities)) != len(identities):
            raise ValueError('Duplicate supplemental identity')
    except Exception as error:
        return [dict(kind='supplemental_schema', completed=False, passed=False,
                     errors=[repr(error)], input_source_guard_owned_by_caller=True)]

    for index, declaration in enumerate(declarations):
        d = copy.deepcopy(declaration)
        kind = d['kind']
        result = dict(index=index, kind=kind, declaration=d, completed=False, passed=False,
                      checks=[], errors=[], source='actual_saved_parts_supplied_by_placed',
                      q_deg=0., continuous_configuration_space_certified=False)
        try:
            if kind == 'conical_seat':
                result['checks'] = [finite_cone_check(d, distance) for distance in ds]
            elif kind == 'keyed_nut_pocket':
                for distance in ds:
                    _, check = collision_sample(parts_at(distance), {d['nut']: lambda shape: shape.copy()})
                    check.update(distance_mm=distance, q_deg=0., label='nominal_nut_against_all_physical_parts')
                    result['checks'].append(check)
                result['checks'].append(keyed_scan(d))
            elif kind == 'axial_float':
                group = d.get('group_parts', [d['part']])
                require_names(group)
                if d['part'] not in group:
                    raise ValueError('Primary float part absent from group')
                offsets = [vector(v, 'float offset') for v in d['offsets_mm']]
                if len(offsets) != 2 or not same(offsets[0], [0., 0., 0.]) or offsets[1][1] <= 0 or offsets[1][0] != 0 or offsets[1][2] != 0:
                    raise ValueError('Expected two nonnegative +Y float endpoints')
                expected_play = .1 if len(group) == 2 else .2
                if abs(offsets[1][1]-expected_play) > 1e-9:
                    raise ValueError('Unexpected nominal float span')
                seats = d['seats']
                if len(seats) != 2 or any(not any(same(vector(s['offset_mm'], 'seat offset'), o) for o in offsets) for s in seats):
                    raise ValueError('Missing endpoint seat declaration')
                if len({tuple(s['offset_mm']) for s in seats}) != 2:
                    raise ValueError('Duplicated seat endpoint')
                for distance in ds:
                    for offset in offsets:
                        shifted, check = collision_sample(parts_at(distance), {
                            n: (lambda shape, o=offset: shape.translate(o)) for n in group})
                        check.update(distance_mm=distance, q_deg=0., offset_mm=offset)
                        check['seats'] = [contact(shifted, seat, distance, frame_of(d)) for seat in seats
                                          if same(vector(seat['offset_mm'], 'seat offset'), offset)]
                        check['passed'] = check['passed'] and all(s['passed'] for s in check['seats'])
                        result['checks'].append(check)
            elif kind == 'correlated_axial_float':
                group = d['group_parts']; require_names(group)
                if len(group) != 2 or frame_of(d) != 'fixed' or len(d['vertices']) != 3:
                    raise ValueError('Expected the two-part fixed-frame correlated triangular domain')
                if any(rows[n]['body'] != 'adjuster_rotor' for n in group):
                    raise ValueError('Correlated float group must contain actual adjuster rotor rows')
                # Group order is not accepted as physical meaning. Bind by
                # the distinct actual role declarations in the saved scene.
                main = [n for n in group if 'adjuster_major_envelope' in rows[n]['role']]
                washers = [n for n in group if 'floating_thrust_washer' in rows[n]['role']]
                if len(main) != 1 or len(washers) != 1 or main == washers:
                    raise ValueError('Cannot identify screw and washer from explicit saved roles')
                sname, wname = main[0], washers[0]
                related = [r for r in declarations if r['kind'] == 'axial_float'
                           and set(r.get('group_parts', [])) == set(group)]
                if len(related) != 1:
                    raise ValueError('Missing unique head-capture endpoint declarations')
                endpoint_seats = related[0]['seats']
                rear = [r for r in endpoint_seats if same(vector(r['offset_mm'], 'rear offset'), [0., 0., 0.])]
                front = [r for r in endpoint_seats if same(vector(r['offset_mm'], 'front offset'), [0., .1, 0.])]
                underside = [r for r in scene['contacts'] if {r['a'], r['b']} == {sname, wname}]
                if len(rear) != 1 or len(front) != 1 or len(underside) != 1:
                    raise ValueError('Missing actual washer/post, head/cap or head/washer seat')
                values = []
                for vertex in d['vertices']:
                    if set(vertex) != set(group):
                        raise ValueError('Incomplete correlated vertex')
                    transforms = {n: vector(vertex[n], 'correlated offset') for n in group}
                    sv, wv = transforms[sname], transforms[wname]
                    if any(v[0] != 0 or v[2] != 0 for v in transforms.values()) or not 0 <= wv[1] <= sv[1] <= .1+1e-9:
                        raise ValueError('Vertex outside declared correlated domain')
                    values.append((sv[1], wv[1]))
                    for distance in ds:
                        shifted, check = collision_sample(parts_at(distance), {
                            n: (lambda shape, o=offset: shape.translate(o)) for n, offset in transforms.items()})
                        check.update(distance_mm=distance, q_deg=0., offsets_by_part_mm=transforms)
                        # A loose washer does not move as a permanently joined
                        # screw group. Require only the real seats at this vertex.
                        required_seats = []
                        if abs(wv[1]) < 1e-9:
                            required_seats.append(copy.deepcopy(rear[0]))
                        if abs(sv[1]-.1) < 1e-9:
                            required_seats.append(copy.deepcopy(front[0]))
                        if abs(sv[1]-wv[1]) < 1e-9:
                            under = copy.deepcopy(underside[0])
                            under['plane_origin_mm'] = plus(vector(under['plane_origin_mm'], 'underhead plane'), sv)
                            required_seats.append(under)
                        if len(required_seats) != 2:
                            raise ValueError('Unexpected correlated vertex support state')
                        check['seats'] = [contact(shifted, seat, distance, 'fixed') for seat in required_seats]
                        check['passed'] = check['passed'] and all(s['passed'] for s in check['seats'])
                        result['checks'].append(check)
                if set(values) != {(0., 0.), (.1, 0.), (.1, .1)}:
                    raise ValueError('Missing/duplicated correlated-domain vertex')
                result['domain_scope'] = 'All three nominal vertices; intermediate correlated states are not certified by a vertex-only sample.'
            elif kind == 'carriage_stop':
                require_names([d['a'], d['b']])
                shift = finite(d['carriage_y_mm'], 'stop position')
                if shift not in (-2., 2.) or rows[d['a']]['body'] != 'carriage' or rows[d['b']]['body'] != 'fixed':
                    raise ValueError('Invalid endpoint stop pair or frame')
                distance = 120.-shift
                origin = vector(d['plane_origin_mm'], 'stop plane')
                lo, hi = d['expected_patch_bounds_xz_mm']
                if len(lo) != 2 or len(hi) != 2 or not (lo[0] < hi[0] and lo[1] < hi[1]):
                    raise ValueError('Missing finite stop contact patch')
                seat = copy.deepcopy(d)
                seat['patch_box_mm'] = [[lo[0], origin[1]-.05, lo[1]], [hi[0], origin[1]+.05, hi[1]]]
                parts = parts_at(distance)
                check = contact(parts, seat, distance, 'fixed')
                value, _ = material(parts[d['a']], parts[d['b']])
                check['mutual_overlap_mm3'] = value
                check['passed'] = check['passed'] and value <= LIMIT
                result['checks'].append(check)
            else:
                raise ValueError('Unimplemented supplemental kind '+kind)
            if not result['checks']:
                raise ValueError('No supplemental measurements produced')
            result['completed'] = all(check.get('completed', False) for check in result['checks'])
            result['passed'] = result['completed'] and all(check.get('passed', False) for check in result['checks'])
        except Exception as error:
            result['errors'].append(repr(error))
            result['completed'] = False
            result['passed'] = False
        results.append(result)
    return results

"""Audit saved fixture BReps, declared contacts and assembly access.

The detached coupon, all motor hardware and new fixture are checked together.
Sampling is not a continuous collision or real fit/strength certification.
"""
from pathlib import Path
import argparse
import itertools
import math
import traceback
import numpy as np
from common import CAD, SOURCE, Inputs, write_json, rigid, bounds, overlap, placed_matrix, rotation_x, translation, source_rows, validate_coverage

POSES = [-20., -15., 0., 15., 30., 45., 60., 75., 85.]
LIMIT = 1e-4


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument('--generated', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    repo, generated, output = a.repo.resolve(), a.generated.resolve(), a.output.resolve()
    if output.exists():
        raise FileExistsError('Preserve earlier report')
    output.parent.mkdir(parents=True, exist_ok=True)
    inputs = Inputs(repo, generated)
    report = dict(schema='thumb_ip_bench_saved_audit.v1', completed=False, passed=False,
        errors=[], source_sha256=inputs.hashes, cases=[], cord_checks=[], contacts=[], assembly=[],
        criteria=dict(overlap_limit_mm3=LIMIT, poses_deg=POSES, maximum_sample_gap_deg=15.,
                      contact_area_tolerance_mm2=1e-5),
        manufacturing_approved=False, measured_latency=False,
        scope='Detached 30-part thumb coupon, motor module and fixture in a single IP coordinate frame; no whole-hand assembly or continuous motion certification.')
    def save():
        report['input_hash_guard'] = inputs.guard()
        write_json(output, report)
    try:
        import cadquery as cq
        from cadquery.occ_impl.shapes import setThreads
        setThreads(1)
        inputs.bind(__file__)
        inputs.bind(repo / CAD / 'common.py')
        geo = inputs.module('_bench_audit_geometry', repo / SOURCE / 'geometry.py')
        routing = inputs.module('_bench_audit_routing', repo / SOURCE / 'routing.py')
        joint = inputs.module('_bench_audit_joint', repo / SOURCE / 'joint_drive.py')
        build = inputs.read(generated / 'build_report.json')
        if not (build['completed'] and not build['errors'] and build['input_hash_guard']['passed']):
            raise ValueError('Incomplete build')
        for key, value in build['source_sha256'].items():
            if not key.startswith('repo:'):
                raise ValueError('Unexpected build source root')
            inputs.bind(repo / key[5:], value)
        for key, value in build['generated_sha256'].items():
            inputs.bind(generated / key, value)
        scene = inputs.read(generated / 'scene.json')
        if not (scene['detached_coupon'] and not scene['full_hand_integration'] and
                scene['motor_to_ip_local_translation_mm'] == [-6., -120., 0.] and
                scene['motor_rotation_per_ip_rotation'] == .77 and scene['motor_rotation_axis'] == '+X'):
            raise ValueError('Fixture frame/scope mismatch')
        report['coverage'] = validate_coverage(scene, source_rows(inputs)[0])
        rows = {r['name']: r for r in scene['parts']}
        if len(rows) != len(scene['parts']) or len(rows) != build['part_count']:
            raise ValueError('Part count mismatch')
        native = {}
        validity = {}
        for name, row in rows.items():
            shape = cq.importers.importStep(str(inputs.resolve_part(row))).val()
            facts = geo.describe(shape)
            expected = 12 if name == 'drive_m077_stator' else 1
            if not (facts['valid'] and facts['bounded'] and facts['positive_solids'] and
                    facts['solid_count'] == expected and facts['volume_mm3'] > 0):
                raise ValueError('Invalid saved part: ' + name)
            if abs(facts['volume_mm3'] - row['volume_mm3']) > max(1e-5, facts['volume_mm3'] * 1e-8):
                raise ValueError('Saved volume differs: ' + name)
            t = np.asarray(row['neutral_matrix'])
            if t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
                raise ValueError('Invalid transform: ' + name)
            if not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-9, rtol=0) or abs(np.linalg.det(t[:3, :3]) - 1) > 1e-9:
                raise ValueError('Non-rigid transform: ' + name)
            native[name] = shape
            validity[name] = facts
        report['native_validity'] = validity
        report['physical_part_count'] = len(rows)
        report['native_solid_count'] = sum(r['solid_count'] for r in validity.values())
        print('Saved parts loaded', flush=True)
        def placed(q):
            return {n: rigid(cq, s, placed_matrix(rows[n], q)) for n, s in native.items()}
        def volume(a, b):
            return geo.describe(geo.boolean(cq, 'common', a, b))['volume_mm3']
        zones = {tuple(sorted(r['pair'])): r for r in scene['thread_zones_motor_frame']}
        if len(zones) != 8:
            raise ValueError('Expected eight pre-existing named motor thread regions')
        def thread_zone(pair, q):
            spec = zones[pair]
            shape = cq.Solid.makeCylinder(spec['radius_mm'], spec['length_mm'],
                                         cq.Vector(*spec['start_mm']), cq.Vector(*spec['axis']))
            bodies = {rows[n]['body'] for n in pair}
            if len(bodies) != 1:
                raise ValueError('Thread pair crosses rigid bodies')
            if bodies == {'motor_output'}:
                shape = shape.rotate((0, 0, 0), (1, 0, 0), .77 * q)
            elif bodies != {'fixed'}:
                raise ValueError('Invalid thread body')
            return shape.translate((-6., -120., 0.))
        # Full neutral audit establishes invariant same-body contacts; the
        # remaining samples recheck every cross-body pair without inheritance
        # from the previous hand audit or selective omission of new hardware.
        neutral = None
        for q in [0.] + [v for v in POSES if v != 0.]:
            parts = placed(q)
            boxes = {n: bounds(s) for n, s in parts.items()}
            if q == 0.:
                neutral = parts
            case = dict(q_deg=q, checked_pairs=0, exact_pairs=0, invariant_pairs=0,
                        collisions=[], declared_thread_contacts=[])
            for n, m in itertools.combinations(sorted(parts), 2):
                if q != 0. and rows[n]['body'] == rows[m]['body']:
                    case['invariant_pairs'] += 1
                    continue
                case['checked_pairs'] += 1
                if not overlap(boxes[n], boxes[m]):
                    continue
                case['exact_pairs'] += 1
                hit = geo.boolean(cq, 'common', parts[n], parts[m])
                v = geo.describe(hit)['volume_mm3']
                if v <= LIMIT:
                    continue
                pair = (n, m)
                if pair in zones:
                    outside = geo.describe(geo.boolean(cq, 'cut', hit, thread_zone(pair, q)))['volume_mm3']
                    case['declared_thread_contacts'].append(dict(pair=list(pair), overlap_mm3=v, outside_mm3=outside))
                    if outside <= LIMIT:
                        continue
                case['collisions'].append(dict(pair=[n, m], overlap_mm3=v))
            report['cases'].append(case)
            print(dict(pose=q, collisions=len(case['collisions']), exact=case['exact_pairs']), flush=True)
            save()
        # Each entire physical cord is split into five contiguous working
        # components. All components are checked against all bench hardware.
        old = inputs.read(repo / SOURCE / 'generated/build_report.json')
        anchors = old['motor_interfaces']['anchors']
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
        from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
        def cylinder(p, end, radius=.3):
            p = cq.Vector(*p)
            delta = cq.Vector(*end) - p
            return cq.Solid.makeCylinder(radius, delta.Length, p, delta.normalized())
        def arc(x, y, radius, aa, bb):
            lo, hi = sorted([aa, bb])
            if not 0 < hi-lo < 2*math.pi:
                raise ValueError('Invalid working wrap')
            axis = gp_Ax2(gp_Pnt(x, y, 0), gp_Dir(1, 0, 0), gp_Dir(0, math.cos(lo), math.sin(lo)))
            result = cq.Shape.cast(BRepPrimAPI_MakeTorus(axis, radius, .3, hi-lo).Shape())
            geo.require_solid(result, 'cord arc')
            return result
        for q in POSES:
            parts = placed(q)
            boxes = {n: bounds(s) for n, s in parts.items()}
            tails = joint.tendon_geometry(q)
            for i, route in enumerate(routing.route(q)):
                tail = cylinder(anchors[i]['tail_start_mm'], anchors[i]['tail_exit_mm']).rotate((0,0,0),(1,0,0), .77*q).translate((-6.,-120.,0.))
                pieces = dict(motor_tail=tail,
                    motor_wrap=arc(route['track_x_mm'], -120., 10., route['motor_anchor_angle_rad'], route['contact_angle_rad']),
                    free_span=cylinder(route['motor_contact_mm'], route['joint_contact_mm']),
                    joint_wrap=arc(route['track_x_mm'], 0., 7.7, route['contact_angle_rad'], route['joint_anchor_angle_rad']),
                    joint_tail=cylinder(tails[i]['anchor_mm'], tails[i]['tail_end_mm']))
                for label, cord in pieces.items():
                    box = bounds(cord)
                    for name, shape in parts.items():
                        exact = overlap(box, boxes[name])
                        v = volume(cord, shape) if exact else 0.
                        report['cord_checks'].append(dict(q_deg=q, track=i, segment=label,
                            part=name, exact=exact, overlap_mm3=v))
            print(dict(cord_pose=q, hits=sum(r['overlap_mm3']>LIMIT for r in report['cord_checks'])), flush=True)
            save()
        # Face-to-face contact is measured, not inferred from intended boxes.
        def touches(a, b):
            return all(a[i] <= b[i+3]+1e-6 and b[i] <= a[i+3]+1e-6 for i in range(3))
        def on_plane(face, origin, normal):
            from OCP.BRep import BRep_Tool
            from OCP.GeomLib import GeomLib_IsPlanarSurface
            test = GeomLib_IsPlanarSurface(BRep_Tool.Surface_s(face.wrapped),1e-6)
            if not test.IsPlanar():
                return False
            plane = test.Plan()
            location = np.asarray([plane.Location().X(),plane.Location().Y(),plane.Location().Z()])
            direction = np.asarray([plane.Axis().Direction().X(),plane.Axis().Direction().Y(),plane.Axis().Direction().Z()])
            return abs(np.dot(location-origin,normal))<1e-5 and abs(np.dot(direction,normal))>1-1e-6
        for declaration in scene['contacts']:
            aa, bb = declaration['a'], declaration['b']
            if aa == bb or aa not in neutral or bb not in neutral:
                raise ValueError('Invalid declared contact')
            area = 0.
            pairs = 0
            origin = np.asarray(declaration['plane_origin_mm'],dtype=float)
            normal = np.asarray(declaration['normal'],dtype=float)
            if abs(np.linalg.norm(normal)-1)>1e-6:
                raise ValueError('Invalid contact normal')
            clip = None
            if 'patch_box_mm' in declaration:
                lo,hi = map(np.asarray,declaration['patch_box_mm'])
                clip = cq.Solid.makeBox(*(hi-lo),cq.Vector(*lo))
            elif 'patch_radius_mm' in declaration:
                clip = cq.Solid.makeCylinder(declaration['patch_radius_mm'],.2,
                    cq.Vector(*(origin-.1*normal)),cq.Vector(*normal))
            for fa in neutral[aa].Faces():
                if not on_plane(fa,origin,normal):
                    continue
                ba = bounds(fa)
                for fb in neutral[bb].Faces():
                    if not touches(ba, bounds(fb)) or not on_plane(fb,origin,normal):
                        continue
                    common = geo.boolean(cq, 'common', fa, fb)
                    if clip is not None and common.Faces():
                        common = geo.boolean(cq,'common',common,clip)
                    value = common.Area()
                    if value > 1e-6:
                        area += value
                        pairs += 1
            minimum = declaration.get('minimum_area_mm2', .1)
            report['contacts'].append(dict(declaration, actual_area_mm2=area,
                face_pairs=pairs, passed=area+1e-5 >= minimum and minimum > 0))
        print(dict(contacts=len(report['contacts']), failed=sum(not r['passed'] for r in report['contacts'])),flush=True)
        save()
        # Screw insertion uses separate conservative shaft and head sweeps;
        # driver access is a third test. Staged exclusions stay explicit.
        def positive_dimension(value, label):
            value = float(value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Expected finite positive assembly dimension: ' + label)
            return value
        def access_vector(value, label):
            value = np.asarray(value, dtype=float)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError('Expected finite assembly vector: ' + label)
            return value
        def final_containment(name, envelope):
            geo.require_solid(envelope, 'Declared final envelope: ' + name)
            outside = geo.describe(geo.boolean(cq, 'cut', neutral[name], envelope))['volume_mm3']
            return dict(checked=True, actual_part=name, outside_volume_mm3=outside,
                tolerance_mm3=LIMIT, passed=outside <= LIMIT,
                coordinates='IP-local neutral mm',
                method='Saved actual part minus its declared final solid envelope; no thread or strength approval')
        for declaration in scene['assembly']:
            print(dict(assembly_part=declaration['part'], kind=declaration['kind']),flush=True)
            if declaration.get('kind') == 'lateral_jaw':
                name = declaration['part']
                start = np.asarray(declaration['initial_translation_mm'],dtype=float)
                end = np.asarray(declaration['final_translation_mm'],dtype=float)
                distance = float(np.linalg.norm(start-end))
                count = max(1,math.ceil(distance))
                # Coupon and all jaws are a separate preassembly. The common
                # base, motor and clamp/foot fasteners have not been installed.
                obstacles = {n:s for n,s in neutral.items() if n!=name and
                    (rows[n]['origin_group']=='coupon' or rows[n]['role']=='custom_split_saddle')}
                hits=[]
                checks=0
                for index in range(count+1):
                    offset=start+(end-start)*index/count
                    shape=neutral[name].translate(offset.tolist())
                    box=bounds(shape)
                    for other,obstacle in obstacles.items():
                        checks += 1
                        if overlap(box,bounds(obstacle)):
                            v=volume(shape,obstacle)
                            if v>LIMIT:
                                hits.append(dict(sample=index,obstacle=other,overlap_mm3=v))
                report['assembly'].append(dict(declaration,checks=checks,collisions=hits,
                    samples=count+1,maximum_step_mm=distance/count,
                    included_obstacles=sorted(obstacles),excluded_before_stage=sorted(set(rows)-set(obstacles)-{name}),
                    passed=not hits,continuous_sweep=False))
                continue
            if declaration.get('kind') == 'axial_nut':
                name = declaration['part']
                mate = declaration['mate_screw']
                p = access_vector(declaration['upper_face'], name + ' upper_face')
                axis = access_vector(declaration['axis'], name + ' axis')
                if name not in rows or mate not in rows or name == mate or abs(np.linalg.norm(axis)-1)>1e-9:
                    raise ValueError('Invalid nut access declaration')
                radius = positive_dimension(declaration['across_flats_mm'], name + ' across_flats_mm')/math.sqrt(3.)
                travel = positive_dimension(declaration.get('entry_travel_mm',30.), name + ' entry_travel_mm')
                height = positive_dimension(declaration.get('height_mm',3.2), name + ' height_mm')
                socket_radius = positive_dimension(declaration['socket_max_dia_mm'], name + ' socket_max_dia_mm')/2
                socket_length = positive_dimension(declaration['socket_length_mm'], name + ' socket_length_mm')
                fit = final_containment(name, cylinder(p, p+height*axis, radius))
                sweeps = dict(
                    nut_outer_sweep=cylinder(p, p+(height+travel)*axis, radius),
                    socket_outer_sweep=cylinder(p, p+(socket_length+travel)*axis, socket_radius))
                excluded = {name, mate, *declaration.get('stage_excluded',[])}
                if not excluded <= set(rows):
                    raise ValueError('Unknown staged nut exclusion')
                hits=[]
                checks=0
                for label, tool in sweeps.items():
                    box=bounds(tool)
                    for other, shape in neutral.items():
                        if other in excluded:
                            continue
                        checks += 1
                        if overlap(box,bounds(shape)):
                            v=volume(tool,shape)
                            if v>LIMIT:
                                hits.append(dict(envelope=label,obstacle=other,overlap_mm3=v))
                report['assembly'].append(dict(declaration,checks=checks,collisions=hits,
                    final_envelope_containment=fit, passed=not hits and fit['passed'],
                    scope='Conservative exterior insertion/tool envelopes. Nut and mating screw excluded for intended threaded/socket engagement; tool internal rod clearance, extension/handle and tightening are not certified.'))
                continue
            if declaration.get('kind') != 'axial_screw':
                raise ValueError('Unimplemented assembly declaration: ' + str(declaration.get('kind')))
            name = declaration['part']
            p = access_vector(declaration['underhead'], name + ' underhead')
            axis = access_vector(declaration['axis'], name + ' axis')
            if abs(np.linalg.norm(axis)-1) > 1e-9 or name not in rows:
                raise ValueError('Invalid screw access frame')
            length = positive_dimension(declaration['length_mm'], name + ' length_mm')
            travel = max(length+10., positive_dimension(declaration.get('entry_travel_mm', 20.), name + ' entry_travel_mm'))
            headheight = positive_dimension(declaration['head_height_mm'], name + ' head_height_mm')
            shaft_radius = positive_dimension(declaration['shaft_radius_mm'], name + ' shaft_radius_mm')
            head_radius = positive_dimension(declaration['head_radius_mm'], name + ' head_radius_mm')
            driver_length = positive_dimension(declaration.get('driver_length_mm',40.), name + ' driver_length_mm')
            driver_radius = positive_dimension(declaration.get('driver_radius_mm',2.), name + ' driver_radius_mm')
            final_envelope = geo.boolean(cq, 'fuse',
                cylinder(p-length*axis, p, shaft_radius),
                cylinder(p, p+headheight*axis, head_radius))
            fit = final_containment(name, final_envelope)
            sweep = dict(
                shaft=cylinder(p-length*axis, p+travel*axis, shaft_radius),
                head=cylinder(p, p+(travel+headheight)*axis, head_radius),
                driver=cylinder(p+(headheight-.5)*axis,
                                p+(headheight+driver_length)*axis, driver_radius))
            critical_clearance = None
            if name.startswith('bench_parent_mount_'):
                from OCP.BRepExtrema import BRepExtrema_DistShapeShape
                station = (int(name.split('_')[3])+1)//2
                obstacle = f'bench_parent_saddle{station}_clamp_screw'
                values = {}
                for label, shape in [('final_screw',neutral[name]),('head_insertion_envelope',sweep['head'])]:
                    distance = BRepExtrema_DistShapeShape(shape.wrapped,neutral[obstacle].wrapped)
                    distance.Perform()
                    if not distance.IsDone():
                        raise ValueError('Unresolved clearance distance')
                    values[label+'_distance_mm'] = distance.Value()
                critical_clearance = dict(obstacle=obstacle, **values,
                    nominal_saved_BRep_only=True, tolerance_stack_qualified=False)
            excluded = {name, *declaration.get('stage_excluded', [])}
            if not excluded <= set(rows):
                raise ValueError('Unknown staged assembly exclusion')
            hits = []
            count = 0
            for label, tool in sweep.items():
                box = bounds(tool)
                for other, shape in neutral.items():
                    if other in excluded:
                        continue
                    count += 1
                    if overlap(box, bounds(shape)):
                        v = volume(tool, shape)
                        if v > LIMIT:
                            hits.append(dict(envelope=label, obstacle=other, overlap_mm3=v))
            report['assembly'].append(dict(declaration, sweep_travel_mm=travel,
                checks=count, collisions=hits, final_envelope_containment=fit,
                cross_clamp_clearance=critical_clearance,
                passed=not hits and fit['passed']))
        report['summary'] = dict(
            parts=len(rows), poses=len(report['cases']),
            collisions=sum(len(c['collisions']) for c in report['cases']),
            cord_part_checks=len(report['cord_checks']),
            cord_collisions=sum(r['overlap_mm3']>LIMIT for r in report['cord_checks']),
            declared_contacts=len(report['contacts']),
            failed_contacts=sum(not r['passed'] for r in report['contacts']),
            screw_access_paths=sum(r['kind']=='axial_screw' for r in report['assembly']),
            nut_access_paths=sum(r['kind']=='axial_nut' for r in report['assembly']),
            lateral_jaw_paths=sum(r['kind']=='lateral_jaw' for r in report['assembly']),
            failed_final_envelope_containment=sum(
                r.get('final_envelope_containment',{}).get('passed') is False for r in report['assembly']),
            blocked_assembly_paths=sum(not r['passed'] for r in report['assembly']))
        report['completed'] = True
        report['passed'] = not any(report['summary'][key] for key in
            ('collisions','cord_collisions','failed_contacts','blocked_assembly_paths'))
    except Exception as error:
        report['errors'].append(dict(error=repr(error), traceback=traceback.format_exc()))
    finally:
        save()
        if not report['input_hash_guard']['passed']:
            report['passed'] = False
            save()
    print({k: report.get(k) for k in ('completed','passed','summary','errors')}, flush=True)
    return 0 if report['completed'] and report['passed'] and not report['errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

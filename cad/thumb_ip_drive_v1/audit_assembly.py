"""Read-only, saved-STEP assembly checks for the thumb IP clamp candidate.

The prescribed insertion envelopes are continuous and conservative. They are
checked in the neutral local thumb subassembly, not against the entire hand.
No geometry is rebuilt, no thread fit/strength is certified, and no minimum
clearance or motion-performance claim follows from a zero material overlap.
Import is inert; CadQuery is imported only by main().
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path, PurePosixPath
import time
import traceback


CHILD = 'thumb_segment3_tip_core'
PARENT = 'thumb_segment2_segment_shell'
PLATE = 'thumb_ip_tendon_clamp'
EPS = 1e-5


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe_path(base, relative):
    """New scene paths must be relative and contained in their declared root."""
    text = str(relative).replace('\\', '/')
    p = PurePosixPath(text)
    if p.is_absolute() or ':' in text or '..' in p.parts or not p.parts:
        raise ValueError('Unsafe relative input: ' + text)
    result = (base / str(p)).resolve()
    if not result.is_relative_to(base):
        raise ValueError('Input escaped declared root: ' + text)
    return result


def convex_hull(points):
    """2-D exact-float monotone hull; used only for verified straight hex prisms."""
    pts = sorted(set((float(x), float(z)) for x, z in points))
    if len(pts) < 3:
        raise ValueError('Degenerate polygon')
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for seq, target in ((pts, lower), (list(reversed(pts)), upper)):
        for point in seq:
            while len(target) >= 2 and cross(target[-2], target[-1], point) <= 1e-10:
                target.pop()
            target.append(point)
    return lower[:-1] + upper[:-1]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument('--generated', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(argv)
    repo, generated, output = args.repo.resolve(), args.generated.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError('Preserve prior evidence; choose a new output: ' + str(output))
    started = time.monotonic()
    hashes = {}
    report = dict(schema='thumb_ip_saved_assembly_audit.v1', completed=False, passed=False,
                  errors=[], source_sha256=hashes, manufacturing_approved=False,
                  physical_latency_measured=False, minimum_clearance_measured=False,
                  criteria=dict(material_overlap_limit_mm3=EPS, region_difference_limit_mm3=EPS,
                                saved_volume_absolute_tolerance_mm3=1e-5,
                                saved_volume_relative_tolerance=1e-8,
                                source_frame_tolerance_mm=1e-9),
                  scope='Saved neutral thumb_mcp/thumb_ip subassembly; explicit staged assembly with cap removed. No whole-hand, moving-pose, load or tolerance approval.',
                  operation_failures=[])

    def bind(path, expected=None):
        p = Path(path).resolve()
        if not (p.is_relative_to(repo) or p.is_relative_to(generated)):
            raise ValueError('Input outside repo/generated roots: ' + str(p))
        value = sha(p)
        if expected is not None and value != expected:
            raise ValueError('Stale input ' + str(p))
        key = str(p)
        if key in hashes and hashes[key] != value:
            raise ValueError('Input changed while reading ' + key)
        hashes[key] = value
        return p

    def read(path):
        return json.loads(bind(path).read_text(encoding='utf-8-sig'))

    def module(name, path):
        spec = importlib.util.spec_from_file_location(name, bind(path))
        value = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(value)
        return value

    def note(stage):
        print(json.dumps(dict(stage=stage, elapsed_s=time.monotonic()-started)), flush=True)

    def save():
        end = {}
        for path in hashes:
            try:
                end[path] = sha(path)
            except OSError:
                end[path] = None
        report['input_hash_guard'] = dict(start=dict(hashes), end=end, passed=end == hashes)
        report['inputs_unchanged'] = report['input_hash_guard']['passed']
        if not report['inputs_unchanged']:
            report['passed'] = False
        report['elapsed_s'] = time.monotonic()-started
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')

    try:
        bind(__file__)
        import cadquery as cq
        from cadquery.occ_impl.shapes import setThreads
        from OCP.Bnd import Bnd_Box
        from OCP.BRepBndLib import BRepBndLib
        setThreads(1)
        geo = module('_thumb_assembly_geometry', repo/'cad/thumb_ip_drive_v1/geometry.py')
        frames = module('_thumb_assembly_frames', repo/'cad/gesture_hand_v3/fabrication_repairs.py')
        build = read(generated/'build_report.json')
        scene, design = read(generated/'scene.json'), read(generated/'design.json')
        if not build.get('completed') or build.get('errors') or not build.get('input_hash_guard', {}).get('passed'):
            raise ValueError('Incomplete or stale build')
        for key, value in build['source_sha256'].items():
            bind(safe_path(repo, key), value)
        for key, value in build['generated_sha256'].items():
            bind(safe_path(generated, key), value)
        for required in ('scene.json', 'design.json'):
            if required not in build['generated_sha256']:
                raise ValueError('Build does not bind ' + required)
        baseline_root = repo/'cad/manufacturing_v2'
        baseline_scene = read(baseline_root/'generated/scene.json')
        baseline_design = read(baseline_root/'generated/design.json')
        if design != baseline_design:
            raise ValueError('Accepted kinematic design changed')
        frame = frames._frame(design['thumb'], 2)
        if scene['thumb_ip_frame'] != frame:
            raise ValueError('Saved thumb frame differs from design-derived frame')
        report['thumb_ip_frame'] = frame
        p = build['joint_parameters']
        expected = dict(anchor_angle_deg=0., bolt_positions_xt_mm=[[0.,-3.8],[0.,3.8]],
                        clamp_width_mm=17.4, clamp_radial_span_mm=[8.,10.],
                        clamp_tangent_span_mm=[-6.3,6.3], nut_af_mm=4.,
                        nut_radial_span_mm=[4.7,6.3], bolt_head_diameter_mm=4.7,
                        bolt_socket_af_mm=1.3, bolt_length_including_head_mm=6.,
                        assembly_window_centre_xy_mm=[0.,32.], assembly_window_length_mm=14.,
                        assembly_window_width_mm=6.4, assembly_window_extrusion_mm=13.)
        if any(p.get(k) != v for k, v in expected.items()):
            raise ValueError('Assembly envelope is only defined for the declared two-centreline-bolt geometry')
        report['parameters'] = dict(expected, axis_convention='X=hinge; +Y=radial/screw entry; Z=T',
                                    radial_screw_envelope_translation_mm=[0.,30.],
                                    screw_window_entry=dict(head_y_mm=35., tip_y_mm=29., start_z_mm=22.),
                                    tool=dict(hex_af_mm=1.3, tip_y_mm=[9.25,15.], shaft_diameter_mm=3.,
                                              shaft_y_mm=[15.,65.], handle_diameter_mm=12., handle_y_mm=[65.,95.]),
                                    nut_translation_x_mm=[-30.,30.])
        rows = {r['name']:r for r in scene['hand']}
        original = {r['name']:r for r in baseline_scene['hand']}
        added = {PLATE} | {f'{PLATE}_{kind}_{i}' for i in (1,2) for kind in ('screw','nut')}
        changed = {CHILD, PARENT} | added
        if (len(rows) != len(scene['hand']) or len(original) != 359 or len(rows) != 364 or
            set(build['modified_names']) != {CHILD, PARENT} or set(build['added_names']) != added or
            set(rows) != set(original) | added or set(build['unchanged_names']) != set(original)-{CHILD,PARENT}):
            raise ValueError('Unexpected physical part correspondence')
        all_asset_paths = {}
        for n, row in rows.items():
            if row['location'] not in ('repo', 'generated'):
                raise ValueError('Unknown scene location')
            base = repo if row['location']=='repo' else generated
            step = bind(safe_path(base, row['step']), row['sha256_step'])
            bind(safe_path(base, row['mesh']), row['sha256_stl'])
            all_asset_paths[n] = step
            if n in original and n not in changed:
                if any(row[k] != original[n][k] for k in ('body','sha256_step','sha256_stl','volume_mm3')):
                    raise ValueError('Unchanged part differs from baseline: ' + n)
        local_rows = {n:r for n,r in rows.items() if r['body'] in ('thumb_mcp','thumb_ip')}
        if not changed <= set(local_rows):
            raise ValueError('Changed parts are not in scoped thumb subassembly')

        def volume(shape):
            return geo.describe(shape)['volume_mm3']

        def bounds(shape):
            box = Bnd_Box()
            BRepBndLib.AddOptimal_s(shape.wrapped, box, False, False)
            return [float(x) for x in box.Get()]

        def overlap(a, b):
            return all(min(a[i+3],b[i+3])-max(a[i],b[i]) > 1e-7 for i in range(3))

        def box(size, centre):
            return cq.Workplane('XY').box(*size).translate(centre).val()

        def ycyl(radius, lo, hi, x=0., t=0.):
            return cq.Solid.makeCylinder(radius, hi-lo, cq.Vector(x,lo,t), cq.Vector(0,1,0))

        def xcyl(radius, lo, hi):
            return cq.Solid.makeCylinder(radius, hi-lo, cq.Vector(lo,0,0), cq.Vector(1,0,0))

        def hex_prism(af, lo, hi, x=0., t=0.):
            radius = af/math.sqrt(3.)
            wire = cq.Wire.makePolygon([cq.Vector(x+radius*math.cos(i*math.pi/3.),lo,
                                                       t+radius*math.sin(i*math.pi/3.)) for i in range(6)], close=True)
            return cq.Solid.extrudeLinear(wire, [], cq.Vector(0,hi-lo,0))

        parts, part_facts = {}, []
        for n in sorted(local_rows):
            world = cq.importers.importStep(str(all_asset_paths[n])).val()
            facts = geo.require_solid(world, n)
            declared = local_rows[n]['volume_mm3']
            if abs(facts['volume_mm3']-declared) > max(1e-5,1e-8*abs(declared)):
                raise ValueError('Saved STEP/scene volume mismatch: ' + n)
            parts[n] = frames._local(world, frame)
            part_facts.append(dict(name=n, body=local_rows[n]['body'], **facts))
        boxes = {n:bounds(s) for n,s in parts.items()}
        report.update(parts=part_facts, local_part_names=sorted(parts), changed_names=sorted(changed),
                      scene_asset_hashes_verified=len(rows), local_step_parts_loaded=len(parts))
        cap = sorted(n for n in parts if n.startswith('thumb_segment3_rounded_tip_cap'))
        if len(cap) != 5:
            raise ValueError('Expected tip cap plus four cap fasteners')
        report['uninstalled_cap_assembly'] = cap
        access = {n:s for n,s in parts.items() if n not in cap}
        note('saved native parts loaded and bound')

        def common(a, b):
            return geo.boolean(cq, 'common', a, b)

        def hits(shape, obstacles, label):
            bb = bounds(shape)
            result = dict(label=label, envelope_bounds_mm=bb, obstacles=sorted(obstacles),
                          exact_pairs=0, box_screened_pairs=0, intersections=[], errors=[], clear=False)
            for n, solid in sorted(obstacles.items()):
                if not overlap(bb, boxes[n]):
                    result['box_screened_pairs'] += 1
                    continue
                result['exact_pairs'] += 1
                try:
                    intersection = common(shape, solid)
                    v = volume(intersection)
                    if v > EPS:
                        result['intersections'].append(dict(part=n, volume_mm3=v,
                                                           intersection_bounds_mm=bounds(intersection)))
                except Exception as exc:
                    failure = dict(check=label, part=n, error=repr(exc))
                    result['errors'].append(failure)
                    report['operation_failures'].append(failure)
            result['clear'] = not result['intersections'] and not result['errors']
            return result

        neutral = dict(pair_scope='Every changed/new part against every local native/new part, same body included; no exemptions',
                       tested_pairs=0, exact_pairs=0, box_screened_pairs=0, intersections=[], errors=[])
        for a,b in itertools.combinations(sorted(parts), 2):
            if a not in changed and b not in changed:
                continue
            neutral['tested_pairs'] += 1
            if not overlap(boxes[a], boxes[b]):
                neutral['box_screened_pairs'] += 1
                continue
            neutral['exact_pairs'] += 1
            try:
                inter = common(parts[a], parts[b]); v = volume(inter)
                if v > EPS:
                    neutral['intersections'].append(dict(parts=[a,b], volume_mm3=v, intersection_bounds_mm=bounds(inter)))
            except Exception as exc:
                failure = dict(check='neutral', parts=[a,b], error=repr(exc))
                neutral['errors'].append(failure); report['operation_failures'].append(failure)
        neutral['clear'] = not neutral['intersections'] and not neutral['errors']
        report['neutral_changed_pairs'] = neutral

        # Re-import the original checkpoint solids, never a previous scratch candidate.
        originals = {}
        for n in (CHILD,PARENT):
            path = (baseline_root/original[n]['step']).resolve()
            if not path.is_relative_to(repo):
                raise ValueError('Baseline STEP escaped repository')
            originals[n] = frames._local(cq.importers.importStep(str(bind(path, original[n]['sha256_step']))).val(), frame)
            geo.require_solid(originals[n], 'baseline '+n)
        report['preservation_regions'] = []
        def region(name, probe, part):
            before = common(originals[part], probe); after = common(parts[part], probe)
            removed = volume(geo.boolean(cq, 'cut', before, after))
            added_v = volume(geo.boolean(cq, 'cut', after, before))
            old_volume = volume(before)
            row = dict(id=name, part=part, reference='original manufacturing_v2 saved STEP',
                       region_bounds_mm=bounds(probe), original_material_mm3=old_volume,
                       current_material_mm3=volume(after), removed_mm3=removed, added_mm3=added_v,
                       unchanged=old_volume>EPS and max(removed,added_v)<=EPS)
            report['preservation_regions'].append(row)
        for name,r,lo,hi in [('child_D_bore_wall_and_flat',3.65,-8.01,8.01),
                             ('child_left_flange_seat',4.5,-8.01,-7.8),('child_right_flange_seat',4.5,7.8,8.01)]:
            region(name, xcyl(r,lo,hi), CHILD)
        for name,r,lo,hi in [('parent_left_bore_wall',3.65,-12.01,-8.99),('parent_right_bore_wall',3.65,8.99,12.01),
                             ('parent_left_flange_seat',4.5,-12.01,-11.8),('parent_right_flange_seat',4.5,11.8,12.01)]:
            region(name, xcyl(r,lo,hi), PARENT)
        length = design['thumb']['links'][2]
        for i,x in enumerate((-4.,4.),1):
            region(f'cap_clamp{i}_boss', ycyl(3.5,length-10.,length-3.,x), CHILD)
            region(f'cap_clamp{i}_rib', box((2.4,6.8,7.9),(x,length-6.6,3.95)), CHILD)
            region(f'cap_clamp{i}_nut_seat', ycyl(2.4,length-5.3,length-5.1,x), CHILD)
            region(f'cap_clamp{i}_cap_seat', ycyl(3.5,length-3.1,length-2.9,x), CHILD)
        report['finite_bore_probes'] = []
        for n,lo,hi in ((CHILD,-8.,8.),(PARENT,-12.,-9.),(PARENT,9.,12.)):
            # Existing D-shaped clear bore; the flat is intentionally retained.
            cylinder = xcyl(3.4,lo,hi)
            probe = common(cylinder, box((hi-lo+1.,30.,30.),((lo+hi)/2.,3.03-15.,0.)))
            row = hits(probe, {n:parts[n]}, 'finite_D_bore_'+n+'_'+str(lo))
            row.update(part=n, axis='+X', radius_mm=3.4, flat_y_mm=3.03, x_span_mm=[lo,hi])
            report['finite_bore_probes'].append(row)
        note('neutral intersections and original protected regions checked')
        save()

        report['staged_screw_insertions'] = []
        report['straight_tool_approaches'] = []
        for i,t in enumerate((-3.8,3.8),1):
            moving = f'{PLATE}_screw_{i}'
            obstacles = {n:s for n,s in access.items() if n!=moving}
            vertical = box((4.7,6.,22.-(t-2.35)), (0.,32.,(22.+t-2.35)/2.))
            radial = geo.boolean(cq,'fuse', ycyl(1.,4.,40.,0.,t), ycyl(2.35,10.,40.,0.,t),
                                 cq.Solid.makeCone(1.,2.35,1.35,cq.Vector(0.,8.65,t),cq.Vector(0,1,0)))
            # Verify that the actual saved screw is contained at both endpoints.
            containment = dict(final_in_radial_mm3=volume(geo.boolean(cq,'cut',parts[moving],radial)),
                transfer_in_both_mm3=volume(geo.boolean(cq,'cut',parts[moving].translate((0.,25.,0.)),common(vertical,radial))))
            # Entry head centre Z=22-R2.35; whole head lies below the top bound.
            containment['entry_in_vertical_mm3'] = volume(geo.boolean(cq,'cut',
                parts[moving].translate((0.,25.,22.-2.35-t)),vertical))
            stages = [hits(vertical,obstacles,f'screw_{i}_window_entry'), hits(radial,obstacles,f'screw_{i}_radial_entry')]
            report['staged_screw_insertions'].append(dict(screw=moving, final_t_mm=t,
                excluded=sorted(set(parts)-set(obstacles)), conservative_continuous_envelopes=True,
                actual_screw_containment=containment, stages=stages,
                clear=all(s['clear'] for s in stages) and max(containment.values())<=EPS))
            tools = dict(tip=hex_prism(1.3,9.25,15.,0.,t),shaft=ycyl(1.5,15.,65.,0.,t),handle=ycyl(6.,65.,95.,0.,t))
            checks = {name:hits(s,access,f'screw_{i}_tool_{name}') for name,s in tools.items()}
            report['straight_tool_approaches'].append(dict(screw=moving,components=checks,
                clear=all(s['clear'] for s in checks.values()),scope='Nominal straight tool in its final access position; hand grip and tool strength unverified'))
        plate_obstacles = {n:s for n,s in access.items() if n!=PLATE and not n.startswith(PLATE+'_screw_')}
        report['plate_side_insertions'] = []
        for side in (-1.,1.):
            envelope = box((47.4,2.,12.6),(15.*side,9.,0.))
            row = hits(envelope,plate_obstacles,'plate_side_'+str(side))
            row.update(direction_x=side, translation_mm=[0.,30.*side], conservative_continuous_envelope=True,
                       excluded=sorted(set(parts)-set(plate_obstacles)),
                       actual_plate_outside_envelope_mm3=volume(geo.boolean(cq,'cut',parts[PLATE],envelope)))
            row['clear'] = row['clear'] and row['actual_plate_outside_envelope_mm3']<=EPS
            report['plate_side_insertions'].append(row)
        note('plate, screw and driver access checked')

        def nut_outer_prism(solid):
            """Extract saved hex vertices, verify the entire saved nut is a straight annulus.

            The convex hull of the actual outer hex and its translated copy is
            exactly the continuous sweep of the filled outer prism. Filling its
            axial bore is conservative for surrounding material clearance.
            """
            bb = bounds(solid); y0,y1 = bb[1],bb[4]
            candidates = []
            for face in solid.Faces():
                if face.geomType()!='PLANE':
                    continue
                normal = face.normalAt()
                if abs(abs(normal.y)-1.)>1e-7 or abs(face.Center().y-y0)>1e-6:
                    continue
                outer = face.outerWire()
                if len(outer.Edges())!=6 or any(e.geomType()!='LINE' for e in outer.Edges()):
                    continue
                vertices = [(v.X,v.Z) for v in outer.Vertices()]
                polygon = convex_hull(vertices)
                if len(polygon)==6:
                    candidates.append(polygon)
            if len(candidates)!=1:
                raise ValueError('Cannot identify one actual saved straight hex outer face')
            polygon = candidates[0]
            def prism(poly):
                wire = cq.Wire.makePolygon([cq.Vector(x,y0,z) for x,z in poly],close=True)
                return cq.Solid.extrudeLinear(wire,[],cq.Vector(0,y1-y0,0))
            filled = prism(polygon)
            centre = ((bb[0]+bb[3])/2.,(bb[2]+bb[5])/2.)
            reconstructed = geo.boolean(cq,'cut',filled,ycyl(1.,y0-.01,y1+.01,*centre))
            differences = [volume(geo.boolean(cq,'cut',solid,reconstructed)),volume(geo.boolean(cq,'cut',reconstructed,solid))]
            if max(differences)>EPS:
                raise ValueError('Saved nut is not the verified straight smooth-bore hex prism')
            return polygon, prism, dict(y_span_mm=[y0,y1],outer_xz_vertices_mm=[list(v) for v in polygon],
                                       actual_prism_reconstruction_difference_mm3=differences)

        report['nut_side_insertions'] = []
        for i in (1,2):
            moving = f'{PLATE}_nut_{i}'
            polygon, prism, facts = nut_outer_prism(parts[moving])
            # Nut loading is performed before inserting the distal link in the fork.
            obstacles = {n:s for n,s in parts.items() if local_rows[n]['body']=='thumb_ip' and n not in cap
                         and n not in (PLATE,moving) and not n.startswith(PLATE+'_screw_')}
            checks = []
            for side in (-1.,1.):
                hull = convex_hull(polygon+[(x+30.*side,z) for x,z in polygon])
                envelope = prism(hull)
                row = hits(envelope,obstacles,f'nut_{i}_side_{side}')
                row.update(direction_x=side,translation_mm=[0.,30.*side],
                           conservative_continuous_envelope=True, bore_filled=True)
                checks.append(row)
            report['nut_side_insertions'].append(dict(nut=moving, **facts,
                excluded=sorted(set(parts)-set(obstacles)), phases='Before parent fork, plate, screws and tip cap installation; other nut present',
                sides=checks,clear=any(x['clear'] for x in checks)))
        note('actual saved hex-nut continuous side loading checked')

        opening = cq.Workplane('XY').center(0.,32.).slot2D(14.,6.4,90.).extrude(13.).val()
        original_opening = common(originals[CHILD],opening)
        remaining_opening = common(parts[CHILD],opening)
        report['assembly_opening'] = dict(uncovered=True, cutter_bounds_mm=bounds(opening),
            original_material_mm3=volume(original_opening), current_material_mm3=volume(remaining_opening),
            original_material_bounds_mm=bounds(original_opening) if original_opening.Solids() else None,
            clear=volume(original_opening)>EPS and volume(remaining_opening)<=EPS,
            strength_approved=False)
        report['core_gate'] = dict(saved_local_parts_valid_single_positive=True,
            neutral_changed_pairs_clear=neutral['clear'], original_regions_preserved=all(r['unchanged'] for r in report['preservation_regions']),
            finite_bores_clear=all(r['clear'] for r in report['finite_bore_probes']),
            plate_side_access=any(r['clear'] for r in report['plate_side_insertions']),
            screw_entries_clear=all(r['clear'] for r in report['staged_screw_insertions']),
            straight_tools_clear=all(r['clear'] for r in report['straight_tool_approaches']),
            nuts_side_loadable=all(r['clear'] for r in report['nut_side_insertions']),
            new_assembly_opening_clear=report['assembly_opening']['clear'],
            no_boolean_failures=not report['operation_failures'])
        report['summary'] = dict(scene_physical_parts=len(rows),local_native_parts=len(parts),changed_parts=len(changed),
            neutral_intersections=len(neutral['intersections']),preservation_regions=len(report['preservation_regions']),
            failed_preservation_regions=sum(not x['unchanged'] for x in report['preservation_regions']),
            blocked_screw_entries=sum(not x['clear'] for x in report['staged_screw_insertions']),
            blocked_tools=sum(not x['clear'] for x in report['straight_tool_approaches']),
            blocked_plate_sides=sum(not x['clear'] for x in report['plate_side_insertions']),
            nuts_without_clear_side=sum(not x['clear'] for x in report['nut_side_insertions']),
            blocked_nut_sides=sum(not s['clear'] for r in report['nut_side_insertions'] for s in r['sides']),
            operation_failure_count=len(report['operation_failures']))
        report['limitations'] = [
            'Only neutral thumb_mcp/thumb_ip parts are loaded. Other fingers, palm, fixture, and motor module are outside assembly-access scope.',
            'The cap and its four fastening parts are absent during access checks. They are included for final neutral changed-part intersections.',
            'Nut loading precedes parent-fork installation. The opposite nut is present. Both directions are reported; one clear side per nut is required.',
            'Continuous conservative solid envelopes cover only the named paths and tool positions. No hand/finger grasp, driver alignment tolerance, installation fixtures or deformed rope is modelled.',
            'The window is an uncovered local upper-wall opening. Appearance acceptance, minimum wall, fatigue, strength, and exposed-rope protection remain unapproved.',
            'Flange/cap regions are compared with original checkpoint material, not a global thickness or load-bearing certification.',
            'Smooth nominal M2 envelopes do not certify thread tolerance, engagement, torque, preload or nut seating. Nut pocket axial clearance and rope compression remain physical assembly questions.',
            'Actual tendon routing, tension, motor retention, sampled moving poses and latency are separate checks; this report grants no 100 ms or hardware approval.'
        ]
        report['completed'] = True
        report['passed'] = all(report['core_gate'].values()) and not report['errors']
    except Exception as exc:
        report['errors'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    finally:
        save()
    note(dict(completed=report['completed'],passed=report['passed'],summary=report.get('summary'),errors=report['errors']))
    return 0 if report['completed'] and report['passed'] and not report['errors'] else 1


if __name__=='__main__':
    raise SystemExit(main())

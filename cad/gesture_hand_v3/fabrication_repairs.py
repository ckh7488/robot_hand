"""Local manufacturing-concept repairs for the FORM002 exterior handoff.

API: repair(parts, design) -> (parts, report). Parts are dictionaries containing
name, world-mm CadQuery shape, body, color, role and density_kg_per_mm3. Importing
this module performs no CAD work or I/O. The caller exports and checks the result.

Accepted visual meshes are never treated as round hardware. All ten distal pin
and bushing sets below use physical circular diameters and explicit flat keys.
Their outer link envelopes and joint frames are preserved. The six palm cover
posts and screws are rebuilt from their measured centres/seating levels.

These are nominal custom-machined geometry concepts. No vendor specification,
fit class, clip spring calculation, strength, life or fabrication release is
implied. Fascia/cap attachment and thin relief-edge cleanup are separate stages.
"""
from __future__ import annotations

import math

RHO = {'polymer': 1.25e-6, 'bronze': 8.8e-6, 'steel': 7.85e-6,
       'aluminum': 2.70e-6}
COLORS = {'bronze': (.62, .45, .20), 'steel': (.55, .58, .62)}


def _cyl(cq, radius, length, start, axis=(1, 0, 0)):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(*axis))


def _box(cq, size, centre):
    return cq.Workplane('XY').box(*size).translate(centre).val()


def _d_prism(cq, radius, x, length, flat_y):
    cylinder = _cyl(cq, radius, length, (x, 0, 0))
    remove = _box(cq, (length+2, radius+2-flat_y, 2*radius+4),
                  (x+length/2, (flat_y+radius+2)/2, 0))
    return cylinder.cut(remove)


def _ring(cq, outer, inner, x, length):
    return _cyl(cq, outer, length, (x, 0, 0)).cut(
        _cyl(cq, inner, length+2, (x-1, 0, 0)))


def _frame(config, joint):
    """Return the neutral distal-joint frame; joint is 1=PIP/MCP or 2=DIP/IP."""
    base = config.get('base_yaw_deg', 0)
    yaw = config.get('second_yaw_deg', 0)
    roll = config.get('second_roll_deg', 0)
    h = config.get('flexion_offset', [0, 0, 0])[2]
    lengths = config['links']
    a = math.radians(base)
    b = math.radians(yaw)
    extra = lengths[1] if joint == 2 else 0
    px, py = -math.sin(b)*extra, lengths[0]+math.cos(b)*extra
    origin = [config['root'][0]+math.cos(a)*px-math.sin(a)*py,
              config['root'][1]+math.sin(a)*px+math.cos(a)*py,
              config['root'][2]+h]
    return dict(origin=origin, yaw=base+yaw, roll=roll)


def _direction(frame, vector):
    x, y, z = vector
    a, b = math.radians(frame['roll']), math.radians(frame['yaw'])
    rx, rz = math.cos(a)*x+math.sin(a)*z, -math.sin(a)*x+math.cos(a)*z
    return [math.cos(b)*rx-math.sin(b)*y, math.sin(b)*rx+math.cos(b)*y, rz]


def _point(frame, point):
    v = _direction(frame, point)
    return [v[i]+frame['origin'][i] for i in range(3)]


def _world(shape, frame):
    return shape.rotate((0, 0, 0), (0, 1, 0), frame['roll']).rotate(
        (0, 0, 0), (0, 0, 1), frame['yaw']).translate(frame['origin'])


def _local(shape, frame):
    return shape.translate([-v for v in frame['origin']]).rotate(
        (0, 0, 0), (0, 0, 1), -frame['yaw']).rotate(
        (0, 0, 0), (0, 1, 0), -frame['roll'])


def _segment_frame(config, segment):
    if segment > 1:
        return _frame(config, segment-1)
    offset = config.get('flexion_offset', [0, 0, 0])
    return dict(origin=[config['root'][i]+offset[i] for i in range(3)],
                yaw=config.get('base_yaw_deg', 0), roll=0)


def _web(cq, width, points_yz):
    wire = cq.Wire.makePolygon([cq.Vector(-width/2, y, z) for y, z in points_yz], close=True)
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(width, 0, 0))


def _skin_planes(shape):
    """Measure the two main planar faces, including affine BSpline planes."""
    planes = []
    for face in shape.Faces():
        normal = face.normalAt()
        if abs(normal.z) < .98:
            continue
        centre = face.Center()
        if any(abs((v.Center()-centre).dot(normal)) > 1e-5 for v in face.Vertices()):
            continue
        planes.append((face.Area(), centre.z+(normal.x*centre.x+normal.y*centre.y)/normal.z,
                       -normal.x/normal.z, -normal.y/normal.z))
    planes.sort(reverse=True)
    if len(planes) < 2:
        raise ValueError('Could not identify two actual fascia planes')
    candidates = [p[1:] for p in planes[:2]]
    return sorted(candidates, key=lambda p: p[0])


def _height(plane, x, y):
    return plane[0]+plane[1]*x+plane[2]*y


def _above_plane(cq, plane):
    pts = [(-100, -100), (100, -100), (100, 200), (-100, 200)]
    wire = cq.Wire.makePolygon([cq.Vector(x, y, _height(plane, x, y)) for x, y in pts], close=True)
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(0, 0, 100))


def _exact_bounds(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, box, False, False)
    values = box.Get()
    return list(values[:3]), list(values[3:])


def _bushing(cq, x, length, flange_x, flange_width, keyed_shaft=False):
    outer = _d_prism(cq, 3.38, x, length, 3.0)
    outer = outer.fuse(_cyl(cq, 4.5, flange_width, (flange_x, 0, 0)))
    lo = min(x, flange_x)-1
    hi = max(x+length, flange_x+flange_width)+1
    bore = (_d_prism(cq, 2.015, lo, hi-lo, 1.73) if keyed_shaft else
            _cyl(cq, 2.015, hi-lo, (lo, 0, 0)))
    return outer.cut(bore)


def _cover_profile(design):
    # FORM002 explicitly stretches the source world palm in Y by 150/127.
    settings = design.get('fabrication_cover_profile', {})
    ramp = settings.get('ramp_end_y_mm', 35*design['palm_target_envelope_mm'][1]/127)
    low, high = settings.get('wrist_front_z_mm', 14), settings.get('front_z_mm', 30)
    return ramp, low, high, settings.get('vertical_wall_mm', 3.5)


def _above_cover(cq, profile, offset=0):
    ramp, low, high, _ = profile
    points = [(-30, low+offset), (0, low+offset), (ramp, high+offset),
              (220, high+offset), (220, 100), (-30, 100)]
    wire = cq.Wire.makePolygon([cq.Vector(-180, y, z) for y, z in points], close=True)
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(360, 0, 0))


def _front_height(y, profile):
    ramp, low, high, _ = profile
    return low+(high-low)*max(0, min(1, y/ramp))


def _final_thumb_clearances(cq, thenar, second_shell, config):
    """Final finite cuts after every union; no change to the joint frames.

    A matching cylindrical face alone did not prove the saved proximal bore
    empty. Recut the complete finite bore and test actual mating hardware.
    """
    root = _segment_frame(config, 1)
    thenar = thenar.cut(_world(_cyl(cq, 3.4, 16.02, (-8.01, 0, 0)), root))
    distal1, distal2 = _frame(config, 1), _frame(config, 2)
    for x in (-9, 8.65):
        thenar = thenar.cut(_world(_cyl(cq, 4.85, .35, (x, 0, 0)), distal1))
    for x in (-12.45, 12):
        second_shell = second_shell.cut(_world(_cyl(cq, 4.55, .45, (x, 0, 0)), distal2))
    return thenar.clean(), second_shell.clean()


def _cover_washer_geometry(cq, profile, x, y, recess_mm=0):
    """Return washer, optional cover pocket, and actual finite seat heights."""
    if not 0 <= recess_mm <= profile[3]-2.5:
        raise ValueError('Cover recess must retain at least 2.5 mm vertical wall')
    if recess_mm and y-4.3 < profile[0]:
        raise ValueError('This local recessed seat is defined only on the flat cover')
    top = _front_height(y+4.2, profile)+.5-recess_mm
    bottom = _front_height(y-4.2, profile)-recess_mm
    washer = _cyl(cq, 4.2, top-bottom, (x, y, bottom), (0, 0, 1)).intersect(
        _above_cover(cq, profile, -recess_mm))
    washer = washer.cut(_cyl(cq, 1.7, 8, (x, y, bottom-.1), (0, 0, 1)))
    pocket = (_cyl(cq, 4.3, recess_mm+.1, (x, y, bottom), (0, 0, 1))
              if recess_mm else None)
    return washer, pocket, top, bottom


def repair(parts, design):
    """Rebuild distal retention and palm cover attachments in candidate copies.

    The body mappings supplied by the caller are preserved. New distal hardware
    uses the same parent/child canonical body names. Existing source files and
    the user-accepted reference are never edited.
    """
    import cadquery as cq

    result = {p['name']: dict(p) for p in parts}
    if len(result) != len(parts):
        raise ValueError('Duplicate input part names')
    before = {name: row['shape'].Volume() for name, row in result.items()}
    changes, additions, removed = {}, [], []
    cylinders, seats, keys, retained, shape_probes, cover_interfaces = [], [], [], [], [], []
    structural_repairs, skin_fasteners = [], []
    extended = design.get('fabrication_extended_repairs', True)

    def modify(name, shape, reason):
        if name not in result:
            raise KeyError('Required repair source missing: '+name)
        result[name]['shape'] = shape
        result[name]['source'] = 'gesture_hand_v3_fabrication_repair'
        changes.setdefault(name, []).append(reason)

    def add(name, shape, body, material, role, color=None):
        if name in result:
            raise ValueError('Duplicate added component: '+name)
        result[name] = dict(name=name, shape=shape, body=body,
                            color=color or COLORS[material], role=role,
                            material=material+'_nominal_assumption',
                            density_kg_per_mm3=RHO[material],
                            source='gesture_hand_v3_fabrication_repair')
        additions.append(name)

    def replace_hardware(name, shape, body, material, role):
        if name not in result:
            raise KeyError('Required old hardware missing: '+name)
        modify(name, shape, 'Replaced transformed display hardware with nominal physical geometry')
        result[name].update(body=body, material=material+'_nominal_assumption',
                            density_kg_per_mm3=RHO[material], role=role,
                            color=COLORS[material])

    configs = [*design['fingers'], dict(design['thumb'], name='thumb')]
    if extended:
        for config in configs:
            n = config['name']
            for segment in (1, 2, 3):
                name = f'{n}_segment{segment}_'+('thenar_shell' if n == 'thumb' and segment == 1 else
                                               'tip_core' if segment == 3 else 'segment_shell')
                frame = _segment_frame(config, segment)
                local = _local(result[name]['shape'], frame)
                if not (n == 'thumb' and segment == 1):
                    depth = 18 if segment == 3 else 22
                    bottom = -depth/2-.3
                    web_points = [(3, -7), (11, bottom), (17, bottom+.5),
                                  (17, bottom+3.0), (11, bottom+3.4), (3, -4)]
                    local = local.fuse(_web(cq, 14, web_points))
                    modify(name, _world(local, frame), 'Integral inward proximal neck web; original underside datum retained')
                    structural_repairs.append(dict(part=name, operation='inward_neck_web',
                        frame=frame, x_span_mm=[-7, 7], polygon_yz_mm=web_points,
                        vertical_web_at_y11_mm=3.4, vertical_web_at_y17_mm=2.5,
                        minimum_wall_or_strength_approved=False))
                if segment < 3:
                    distal = _frame(config, segment)
                    prior = result[name]['shape']
                    cutter = _world(_box(cq, (29, 6.4, 30), (0, -6.8, 19.8)), distal)
                    after = prior.cut(cutter)
                    modify(name, after, 'Removed unsupported upper relief lip near distal hinge; bore region retained')
                    structural_repairs.append(dict(part=name, operation='relief_lip_removal',
                        distal_frame=distal, cutter_local_min_mm=[-14.5, -10, 4.8],
                        cutter_local_max_mm=[14.5, -3.6, 34.8],
                        removed_mm3=prior.Volume()-after.Volume(),
                        minimum_wall_or_strength_approved=False))

    for config in configs:
        n = config['name']
        labels = ['opposition', 'mcp', 'ip'] if n == 'thumb' else ['mcp', 'pip', 'dip']
        for joint in (1, 2):
            frame = _frame(config, joint)
            prefix = f'{n}_distal{joint}'
            parent_body, child_body = n+'_'+labels[joint-1], n+'_'+labels[joint]
            parent_name = f'{n}_segment{joint}_'+('thenar_shell' if n == 'thumb' and joint == 1 else 'segment_shell')
            child_name = f'{n}_segment{joint+1}_'+('tip_core' if joint == 2 else 'segment_shell')
            parent, child = result[parent_name]['shape'], result[child_name]['shape']
            if n == 'thumb' and joint == 1:
                # FORM002's unequal Y scaling changed the old circular eye
                # clearance into a tighter ellipse than the incoming child.
                # Restore the real rotary eye clearance in the final frame.
                parent = parent.cut(_world(_cyl(cq, 8.4, 17.6, (-8.8, 0, 0)), frame))
            # Fill the old circular/elliptical hole locally, then make the final
            # actual D bore. The plug radius remains inside the existing eye.
            for x in (-12, 9):
                parent = parent.fuse(_world(_cyl(cq, 3.65, 3, (x, 0, 0)), frame))
                parent = parent.cut(_world(_d_prism(cq, 3.4, x-.01, 3.02, 3.03), frame))
            child = child.fuse(_world(_cyl(cq, 3.65, 16, (-8, 0, 0)), frame))
            child = child.cut(_world(_d_prism(cq, 3.4, -8.01, 16.02, 3.03), frame))
            modify(parent_name, parent, f'Distal{joint}: final round-radius/flat-key parent bores after all source unions')
            modify(child_name, child, f'Distal{joint}: final round-radius/flat-key central-eye bore')

            for side, x, flange in [('left', -12, -12.4), ('right', 9, 12)]:
                name = f'{n}_segment{joint}_distal_{side}_bushing'
                bush = _bushing(cq, x, 3, flange, .4, keyed_shaft=side == 'left')
                replace_hardware(name, _world(bush, frame), parent_body, 'bronze', 'keyed_flanged_parent_bushing')
                keys.append(dict(id=name+'_outer_key', part=name, mating_part=parent_name,
                                 plane_origin_mm=_point(frame, (0, 3.0, 0)),
                                 plane_normal=_direction(frame, (0, 1, 0)),
                                 axial_origin_mm=frame['origin'], axial_direction=_direction(frame, (1, 0, 0)),
                                 span_mm=[x, x+3], mating_flat_offset_mm=.03))
                seats.append(dict(id=name+'_flange_seat', parts=[name, parent_name],
                                  plane_origin_mm=_point(frame, (-12 if side == 'left' else 12, 0, 0)),
                                  normal=_direction(frame, (1, 0, 0)), patch_radius_mm=4.5))
            old_child_bush = f'{n}_segment{joint+1}_proximal_bushing'
            if old_child_bush not in result:
                raise KeyError(old_child_bush)
            del result[old_child_bush]
            removed.append(old_child_bush)
            child_bushes = []
            for side, x, flange in [('left', -8, -8.6), ('right', 0, 8)]:
                name = prefix+'_child_'+side+'_bushing'
                add(name, _world(_bushing(cq, x, 8, flange, .6), frame), child_body,
                    'bronze', 'keyed_flanged_child_bushing')
                child_bushes.append(name)
                cylinders.append(dict(id=name+'_journal', part=name, axis_origin_mm=frame['origin'],
                                      axis_direction=_direction(frame, (1, 0, 0)),
                                      radius_mm=2.015, span_mm=[x, x+8]))
                keys.append(dict(id=name+'_outer_key', part=name, mating_part=child_name,
                                 plane_origin_mm=_point(frame, (0, 3.0, 0)),
                                 plane_normal=_direction(frame, (0, 1, 0)),
                                 axial_origin_mm=frame['origin'], axial_direction=_direction(frame, (1, 0, 0)),
                                 span_mm=[x, x+8], mating_flat_offset_mm=.03))
                seats.append(dict(id=name+'_flange_seat', parts=[name, child_name],
                                  plane_origin_mm=_point(frame, (-8 if side == 'left' else 8, 0, 0)),
                                  normal=_direction(frame, (1, 0, 0)), patch_radius_mm=4.5))
            pin = _cyl(cq, 2, 27.4, (-13, 0, 0)).fuse(_cyl(cq, 3.5, 2, (-15, 0, 0)))
            pin = pin.cut(_box(cq, (4, 4, 10), (-11, 3.7, 0)))
            pin = pin.cut(_ring(cq, 2.4, 1.6, 13.1, .8))
            pin_name = f'{n}_joint_pin_{joint}'
            replace_hardware(pin_name, _world(pin, frame), parent_body, 'steel', 'headed_keyed_grooved_distal_pin')
            for side, x in [('left', -13), ('right', 12.4)]:
                add(prefix+'_'+side+'_pin_washer', _world(_ring(cq, 3.7, 2.05, x, .6), frame),
                    parent_body, 'steel', 'distal_pin_washer')
            for side, x in [('left', -9), ('right', 8.65)]:
                add(prefix+'_'+side+'_thrust_washer', _world(_ring(cq, 4.8, 2.05, x, .35), frame),
                    parent_body, 'steel', 'distal_thrust_washer')
            clip = _ring(cq, 3.5, 1.65, 13.15, .7).cut(_box(cq, (2, 5, 2.6), (13.5, 2.5, 0)))
            clip_name = prefix+'_retaining_clip'
            add(clip_name, _world(clip, frame), parent_body, 'steel', 'nominal_external_spring_clip')
            shape_probes.append(dict(id=prefix+'_actual_pin_clear_path', moving_part=pin_name,
                                     against_parts=[parent_name, child_name,
                                       f'{n}_segment{joint}_distal_left_bushing',
                                       f'{n}_segment{joint}_distal_right_bushing', *child_bushes],
                                     maximum_intersection_mm3=1e-4,
                                     scope='Placed actual keyed pin shape; not an assembly-path or tolerance proof'))
            retained.append(dict(id=prefix, parent_body=parent_body, child_body=child_body,
                                 axis_origin_mm=frame['origin'], axis_direction=_direction(frame, (1, 0, 0)),
                                 pin=pin_name, clip=clip_name,
                                 pin_head_span_mm=[-15, -13], fork_outer_span_mm=[-12, 12],
                                 groove_span_mm=[13.1, 13.9], clip_span_mm=[13.15, 13.85],
                                 pin_tip_mm=14.4, child_eye_span_mm=[-8, 8],
                                 left_pin_flat_mm=1.7, left_bush_flat_mm=1.73,
                                 clip_spring_and_fit_approved=False))

    if extended:
        def attach_skin(skin_name, structure_name, frame, positions, planes=None):
            """Two nominal M2 clamps and captive nuts, all in the link frame."""
            skin = _local(result[skin_name]['shape'], frame)
            structure = _local(result[structure_name]['shape'], frame)
            lower, upper = planes or _skin_planes(skin)
            above_lower, above_upper = _above_plane(cq, lower), _above_plane(cq, upper)
            for index, (x, y) in enumerate(positions):
                prefix = skin_name+f'_clamp{index+1}'
                boss_radius = 3.5
                seat_min = _height(lower, x, y)-boss_radius*math.hypot(lower[1], lower[2])
                seat_max = _height(lower, x, y)+boss_radius*math.hypot(lower[1], lower[2])
                nut_top = seat_min-2.0
                nut_bottom = nut_top-1.6
                boss_bottom = nut_bottom-.6
                boss = _cyl(cq, boss_radius, seat_max-boss_bottom+.1,
                            (x, y, boss_bottom), (0, 0, 1)).cut(above_lower)
                # Side insertion is from the hollow link cavity before the skin
                # is fitted. This is an explicit opening, not a hidden solid nut.
                pocket = cq.Workplane('XY').polygon(6, 4.85).extrude(1.8).translate(
                    (x, y, nut_bottom-.2)).val()
                slot = _box(cq, (7.2, 4.9, 1.8), (x-2.6, y, nut_top-.9))
                structure = structure.fuse(boss).cut(pocket.fuse(slot))
                bore = _cyl(cq, 1.2, seat_max-boss_bottom+3, (x, y, boss_bottom-.1), (0, 0, 1))
                structure = structure.cut(bore)
                skin = skin.cut(_cyl(cq, 1.2, 12, (x, y, seat_min-2), (0, 0, 1)))
                washer_radius = 2.3
                washer_top = _height(upper, x, y)+washer_radius*math.hypot(upper[1], upper[2])+.25
                washer_bottom = _height(upper, x, y)-washer_radius*math.hypot(upper[1], upper[2])
                washer = _cyl(cq, washer_radius, washer_top-washer_bottom,
                              (x, y, washer_bottom), (0, 0, 1)).intersect(above_upper)
                washer = washer.cut(_cyl(cq, 1.1, 3, (x, y, washer_bottom-.1), (0, 0, 1)))
                shaft_length = math.ceil(washer_top-nut_bottom+1.3)
                shaft_bottom = washer_top-shaft_length
                bolt = _cyl(cq, 1, shaft_length, (x, y, shaft_bottom), (0, 0, 1)).fuse(
                    _cyl(cq, 1.9, 1.2, (x, y, washer_top), (0, 0, 1)))
                socket = cq.Workplane('XY').polygon(6, 1.5).extrude(.7).translate((x, y, washer_top+.5)).val()
                bolt = bolt.cut(socket)
                nut = cq.Workplane('XY').polygon(6, 4.62).extrude(1.6).translate((x, y, nut_bottom)).val()
                nut = nut.cut(_cyl(cq, 1, 2, (x, y, nut_bottom-.1), (0, 0, 1)))
                body = result[structure_name]['body']
                add(prefix+'_bolt', _world(bolt, frame), body, 'steel', 'nominal_M2_skin_clamp_bolt')
                add(prefix+'_nut', _world(nut, frame), body, 'steel', 'nominal_M2_skin_captive_nut')
                add(prefix+'_washer', _world(washer, frame), body, 'steel', 'profiled_skin_seating_washer')
                skin_fasteners.append(dict(id=prefix, skin=skin_name, structure=structure_name,
                    bolt=prefix+'_bolt', nut=prefix+'_nut', washer=prefix+'_washer', body=body,
                    frame=frame, centre_local_xy_mm=[x, y],
                    axis_origin_mm=_point(frame, (x, y, 0)), axis_direction=_direction(frame, (0, 0, 1)),
                    skin_lower_plane_local=lower, skin_upper_plane_local=upper,
                    boss_radius_mm=boss_radius, boss_bottom_local_z_mm=boss_bottom,
                    boss_upper_seat_local_z_mm=_height(lower, x, y),
                    shaft_radius_mm=1, clearance_radius_mm=1.2,
                    nut_span_local_z_mm=[nut_bottom, nut_top],
                    shaft_span_local_z_mm=[shaft_bottom, washer_top],
                    shaft_length_mm=shaft_length, tip_beyond_nut_mm=nut_bottom-shaft_bottom,
                    washer_top_local_z_mm=washer_top,
                    driver_path_and_thread_preload_approved=False))
                for seat_name, pair_names, plane, radius in [
                    ('boss_skin', [structure_name, skin_name], lower, 3.5),
                    ('washer_skin', [prefix+'_washer', skin_name], upper, 2.3),
                    ('head_washer', [prefix+'_bolt', prefix+'_washer'], (washer_top, 0, 0), 1.9),
                    ('nut_boss', [prefix+'_nut', structure_name], (nut_top, 0, 0), 2.4)]:
                    normal = [-plane[1], -plane[2], 1]
                    norm = math.sqrt(sum(v*v for v in normal))
                    seats.append(dict(id=prefix+'_'+seat_name, parts=pair_names,
                        plane_origin_mm=_point(frame, (x, y, _height(plane, x, y))),
                        normal=_direction(frame, [v/norm for v in normal]), patch_radius_mm=radius))
            modify(structure_name, _world(structure, frame), 'Integral captive-nut bosses and actual M2 clearance passages')
            modify(skin_name, _world(skin, frame), 'Actual clamp through-holes; retained by two M2 bolts and nuts')

        for config in configs:
            n = config['name']
            for segment in (1, 2):
                frame = _segment_frame(config, segment)
                structure_name = f'{n}_segment{segment}_'+('thenar_shell' if n == 'thumb' and segment == 1 else 'segment_shell')
                dark_name = f'{n}_segment{segment}_dark_faceplate'
                transition_name = f'{n}_segment{segment}_palmar_transition'
                if dark_name in result and transition_name in result:
                    local_dark = _local(result[dark_name]['shape'], frame)
                    dark_lo, dark_hi = _exact_bounds(local_dark)
                    if dark_hi[1]-dark_lo[1] < 4:
                        joined = _local(result[transition_name]['shape'].fuse(result[dark_name]['shape']), frame)
                        joined = joined.cut(_box(cq, (100, 100, 100), (0, dark_hi[1]-1.3+50, 0)))
                        modify(transition_name, _world(joined, frame),
                               'Merged unfastenable sub-4mm fascia sliver into adjacent retained skin; trimmed its relief wedge')
                        del result[dark_name]
                        removed.append(dark_name)
                        structural_repairs.append(dict(operation='merge_unfastenable_fascia_sliver',
                            original_part=dark_name, retained_part=transition_name,
                            original_sliver_length_mm=dark_hi[1]-dark_lo[1],
                            local_colour_boundary_removed=True))
                for suffix in ('dark_faceplate', 'palmar_transition'):
                    name = f'{n}_segment{segment}_{suffix}'
                    if name not in result:
                        continue
                    local = _local(result[name]['shape'], frame)
                    lo, hi = _exact_bounds(local)
                    # The angled relief used to leave a sharp taper at the end.
                    # A square edge 1.3 mm earlier retains the full sheet gauge.
                    if suffix == 'dark_faceplate':
                        local = local.cut(_box(cq, (100, 100, 100), (0, hi[1]-1.3+50, 0)))
                        modify(name, _world(local, frame), 'Square-trimmed thin relief wedge at end of fascia')
                        lo, hi = _exact_bounds(local)
                    result[name].update(material='aluminum_nominal_assumption', density_kg_per_mm3=RHO['aluminum'],
                                        role='clamped_0p9mm_formed_metal_fascia_concept')
                    span = hi[1]-lo[1]
                    if span < 13:
                        positions = [(-4, (lo[1]+hi[1])/2), (4, (lo[1]+hi[1])/2)]
                    else:
                        positions = [(0, lo[1]+4.7), (0, hi[1]-4.7)]
                    attach_skin(name, structure_name, frame, positions)
            frame = _segment_frame(config, 3)
            name, core = n+'_segment3_rounded_tip_cap', n+'_segment3_tip_core'
            length = config['links'][2]
            cap = _local(result[name]['shape'], frame)
            structure = _local(result[core]['shape'], frame)
            structure = structure.cut(_box(cq, (100, 100, 100), (0, length-3.2+50, 0)))
            def hex_y(x, y, z, diameter, depth):
                points = [cq.Vector(x+diameter/2*math.cos(i*math.pi/3), y,
                                    z+diameter/2*math.sin(i*math.pi/3)) for i in range(6)]
                return cq.Solid.extrudeLinear(cq.Wire.makePolygon(points, close=True), [], cq.Vector(0, depth, 0))
            for i, x in enumerate((-4, 4)):
                prefix = name+f'_clamp{i+1}'
                boss = _cyl(cq, 3.5, 7, (x, length-10, 0), (0, 1, 0))
                # Dorsal internal rib connects the nut boss to the actual core
                # roof. It remains below the retained outer tip-skin surface.
                rib = _box(cq, (2.4, 6.8, 7.9), (x, length-6.6, 3.95))
                pocket = hex_y(x, length-7, 0, 4.85, 1.8)
                insertion = _box(cq, (4.9, 1.8, 5), (x, length-6.1, -2.5))
                structure = structure.fuse(boss).fuse(rib).cut(pocket.fuse(insertion))
                structure = structure.cut(_cyl(cq, 1.2, 10, (x, length-11, 0), (0, 1, 0)))
                cap = cap.cut(_cyl(cq, 1.2, 5, (x, length-4, 0), (0, 1, 0)))
                cap = cap.cut(_cyl(cq, 2.05, 1.4, (x, length-1.3, 0), (0, 1, 0)))
                bolt = _cyl(cq, 1, 8, (x, length-9.3, 0), (0, 1, 0)).fuse(
                    _cyl(cq, 1.9, 1.2, (x, length-1.3, 0), (0, 1, 0)))
                bolt = bolt.cut(hex_y(x, length-.8, 0, 1.5, .8))
                nut = hex_y(x, length-6.8, 0, 4.62, 1.6).cut(
                    _cyl(cq, 1, 2, (x, length-6.9, 0), (0, 1, 0)))
                body = result[core]['body']
                add(prefix+'_bolt', _world(bolt, frame), body, 'steel', 'recessed_axial_M2_tip_bolt')
                add(prefix+'_nut', _world(nut, frame), body, 'steel', 'nominal_M2_tip_captive_nut')
                skin_fasteners.append(dict(id=prefix, skin=name, structure=core,
                    bolt=prefix+'_bolt', nut=prefix+'_nut', body=body,
                    frame=frame, axis_origin_mm=_point(frame, (x, 0, 0)), axis_direction=_direction(frame, (0, 1, 0)),
                    local_axis='Y', boss_radius_mm=3.5, boss_front_y_mm=length-3,
                    cap_inner_front_y_mm=length-3, core_trim_front_y_mm=length-3.2,
                    head_span_local_y_mm=[length-1.3, length-.1], head_counterbore_radius_mm=2.05,
                    nut_span_local_y_mm=[length-6.8, length-5.2],
                    shaft_span_local_y_mm=[length-9.3, length-1.3],
                    shaft_radius_mm=1, clearance_radius_mm=1.2, shaft_length_mm=8,
                    tip_beyond_nut_mm=2.5, external_tip_length_preserved=True,
                    driver_path_and_thread_preload_approved=False))
                for seat_name, pair_names, y, radius in [
                    ('boss_cap', [core, name], length-3, 3.5),
                    ('head_cap', [prefix+'_bolt', name], length-1.3, 1.9),
                    ('nut_boss', [prefix+'_nut', core], length-5.2, 2.4)]:
                    seats.append(dict(id=prefix+'_'+seat_name, parts=pair_names,
                        plane_origin_mm=_point(frame, (x, y, 0)),
                        normal=_direction(frame, (0, 1, 0)), patch_radius_mm=radius))
            modify(core, _world(structure, frame), 'Trimmed 0.2 mm front clearance and integral axial-nut bosses with dorsal ribs')
            modify(name, _world(cap, frame), 'Two recessed axial M2 heads and actual shaft passages; original tip extent retained')

    thumb = dict(design['thumb'], name='thumb')
    thenar_name, second_name = 'thumb_segment1_thenar_shell', 'thumb_segment2_segment_shell'
    thenar, second = _final_thumb_clearances(cq, result[thenar_name]['shape'],
                                           result[second_name]['shape'], thumb)
    modify(thenar_name, thenar, 'Final R3.4 proximal through-bore and finite R4.85 thrust-washer pockets after all unions')
    modify(second_name, second, 'Finite R4.55 distal bushing-flange pockets; original axial seating planes retained')
    structural_repairs.append(dict(operation='final_thumb_finite_hardware_clearance',
        root_frame=_segment_frame(thumb, 1), root_bore_radius_mm=3.4,
        root_bore_span_mm=[-8.01, 8.01], root_bushing_outer_radius_mm=3.38,
        nominal_radial_gap_mm=.02,
        distal1_frame=_frame(thumb, 1), thrust_pocket_radius_mm=4.85,
        thrust_pocket_spans_mm=[[-9, -8.65], [8.65, 9]],
        distal2_frame=_frame(thumb, 2), flange_pocket_radius_mm=4.55,
        flange_pocket_spans_mm=[[-12.45, -12], [12, 12.45]],
        printed_fit_or_strength_approved=False))
    for hardware, shell in [
        ('thumb_root_proximal_left_bushing', thenar_name),
        ('thumb_root_proximal_right_bushing', thenar_name),
        ('thumb_distal1_left_thrust_washer', thenar_name),
        ('thumb_distal1_right_thrust_washer', thenar_name),
        ('thumb_segment2_distal_left_bushing', second_name),
        ('thumb_segment2_distal_right_bushing', second_name)]:
        # A standalone pre-root stage has no replacement root bushings. The
        # integrated candidate includes them and must test their actual solids.
        if hardware in result:
            shape_probes.append(dict(id=hardware+'_actual_shell_clearance',
                moving_part=hardware, against_parts=[shell], maximum_intersection_mm3=1e-4,
                scope='Actual finite placed hardware against final shell; nominal geometry only'))

    # Circular cover posts replace the visually stretched ellipses. Their
    # centres and cover seating levels come from the actual new candidate.
    palm_name = 'Palm_tapered_chassis'
    palm = result[palm_name]['shape']
    cover_name = 'Palm_shaped_front_cover'
    cover = result[cover_name]['shape']
    cover_profile = _cover_profile(design)
    upper_space = _above_cover(cq, cover_profile)
    inner_space = _above_cover(cq, cover_profile, -cover_profile[3])
    floor = design.get('palm_back_z_mm', -44)+3.5
    post_names = [name for name in result if name.startswith('Cover_post_')]
    if len(post_names) != 6:
        raise ValueError('Expected six explicit source cover-post identities')
    for post_name in post_names:
        old = result[post_name]
        bounds = _exact_bounds(old['shape'])
        x, y = [(bounds[0][i]+bounds[1][i])/2 for i in (0, 1)]
        top = _front_height(y, cover_profile)-cover_profile[3]
        if abs(bounds[1][2]-top) > .1:
            raise ValueError('Source post does not match declared FORM002 ramp at centre: '+post_name)
        top_min = _front_height(y-4.8, cover_profile)-cover_profile[3]
        top_max = _front_height(y+4.8, cover_profile)-cover_profile[3]
        post = _cyl(cq, 4.8, top_max-floor+.3, (x, y, floor-.2), (0, 0, 1)).cut(inner_space)
        post = post.cut(_cyl(cq, 1.8, top_max-floor+1, (x, y, floor-.3), (0, 0, 1)))
        nut_top = top_min-1.9
        nut_bottom = nut_top-2.4
        pocket = cq.Workplane('XY').polygon(6, 6.58).extrude(2.6).translate((x, y, nut_bottom-.2)).val()
        slot = _box(cq, (6.6, 7, 2.6), (x, y-3, nut_top-1.3))
        post = post.cut(pocket.fuse(slot))
        palm = palm.fuse(post)
        # Fill the transformed elliptical source hole inside the existing skin
        # profile before cutting the actual circular clearance passage.
        patch = _cyl(cq, 2.4, 65, (x, y, -10), (0, 0, 1)).intersect(inner_space).cut(upper_space)
        cover = cover.fuse(patch).cut(_cyl(cq, 1.8, 65, (x, y, -10), (0, 0, 1)))
        suffix = post_name[len('Cover_post_'):]
        screw_name = 'Cover_screw_'+suffix
        recess = design.get('fabrication_cover_recess_mm', {'-18_104': .5}).get(suffix, 0)
        washer, seat_pocket, washer_top, washer_bottom = _cover_washer_geometry(
            cq, cover_profile, x, y, recess)
        if seat_pocket is not None:
            cover = cover.cut(seat_pocket)
        washer_name = 'Cover_seat_washer_'+suffix
        add(washer_name, washer, 'palm', 'steel', 'profiled_cover_seating_washer')
        shaft_length = 2*math.ceil((washer_top-nut_bottom+2)/2)
        shaft_bottom = washer_top-shaft_length
        screw = _cyl(cq, 1.5, shaft_length, (x, y, shaft_bottom), (0, 0, 1))
        screw = screw.fuse(_cyl(cq, 3.3, 1.65, (x, y, washer_top), (0, 0, 1)))
        socket = cq.Workplane('XY').polygon(6, 2.31).extrude(1.1).translate((x, y, washer_top+.65)).val()
        screw = screw.cut(socket)
        replace_hardware(screw_name, screw, 'palm', 'steel', 'nominal_complete_M3_cover_bolt')
        nut = cq.Workplane('XY').polygon(6, 6.35).extrude(2.4).translate((x, y, nut_bottom)).val()
        nut = nut.cut(_cyl(cq, 1.5, 2.6, (x, y, nut_bottom-.1), (0, 0, 1)))
        nut_name = 'Cover_nut_'+suffix
        add(nut_name, nut, 'palm', 'steel', 'nominal_side_inserted_M3_cover_nut')
        del result[post_name]
        removed.append(post_name)
        cover_interfaces.append(dict(id=suffix, original_post=post_name, fused_into=palm_name,
                                     centre_xy_mm=[x, y], floor_z_mm=floor, post_top_z_at_centre_mm=top,
                                     post_top_z_range_mm=[top_min, top_max],
                                     cover_top_z_at_centre_mm=top+cover_profile[3],
                                     washer=washer_name, washer_top_z_mm=washer_top,
                                     washer_seat_recess_mm=recess,
                                     washer_bottom_z_at_centre_mm=_front_height(y, cover_profile)-recess,
                                     counterbore_radius_mm=4.3 if recess else None,
                                     vertical_cover_wall_at_recess_mm=cover_profile[3]-recess,
                                     screw=screw_name, nut=nut_name,
                                     shaft_radius_mm=1.5, clearance_radius_mm=1.8,
                                     shaft_span_z_mm=[shaft_bottom, washer_top], nut_span_z_mm=[nut_bottom, nut_top],
                                     shaft_length_mm=shaft_length, nut_upper_seat_z_mm=nut_top,
                                     tip_beyond_nut_mm=nut_bottom-shaft_bottom,
                                     seat_surface='Conformal piecewise planar FORM002 ramp, not a horizontal plane',
                                     helical_threads_and_preload_approved=False))
        seats.append(dict(id=suffix+'_bolt_washer', parts=[screw_name, washer_name],
                          plane_origin_mm=[x, y, washer_top], normal=[0, 0, 1], patch_radius_mm=3.3))
        seats.append(dict(id=suffix+'_nut_post', parts=[nut_name, palm_name],
                          plane_origin_mm=[x, y, nut_top], normal=[0, 0, 1], patch_radius_mm=3.175))
        for y_sample in [max(0, min(y, cover_profile[0]-1e-6))] if y-4.8 < cover_profile[0] else []:
            slope = (cover_profile[2]-cover_profile[1])/cover_profile[0]
            seats.append(dict(id=suffix+'_post_cover_ramp', parts=[palm_name, cover_name],
                              plane_origin_mm=[x, y, cover_profile[1]+slope*y-cover_profile[3]],
                              normal=[0, -slope/math.sqrt(1+slope*slope), 1/math.sqrt(1+slope*slope)],
                              patch_radius_mm=4.8, patch_centre_xy_mm=[x, y]))
            seats.append(dict(id=suffix+'_washer_cover_ramp', parts=[washer_name, cover_name],
                              plane_origin_mm=[x, y, cover_profile[1]+slope*y],
                              normal=[0, -slope/math.sqrt(1+slope*slope), 1/math.sqrt(1+slope*slope)],
                              patch_radius_mm=4.2, patch_centre_xy_mm=[x, y]))
        if y+4.8 >= cover_profile[0]:
            seats.append(dict(id=suffix+'_post_cover_flat', parts=[palm_name, cover_name],
                              plane_origin_mm=[x, y, cover_profile[2]-cover_profile[3]], normal=[0, 0, 1],
                              patch_radius_mm=4.8, patch_centre_xy_mm=[x, y]))
            seats.append(dict(id=suffix+'_washer_cover_flat', parts=[washer_name, cover_name],
                              plane_origin_mm=[x, y, cover_profile[2]-recess], normal=[0, 0, 1],
                              patch_radius_mm=4.2, patch_centre_xy_mm=[x, y]))
    modify(palm_name, palm, 'Six round captive-nut cover posts fused physically into the retained palm floor')
    modify(cover_name, cover, 'Six actual circular cover passages and declared local washer-seat recess')

    for name, part in result.items():
        if name in changes or name in additions:
            shape = part['shape'].clean()
            if not shape.isValid() or len(shape.Solids()) != 1:
                raise ValueError('Repair part not one valid connected solid: '+name)
            part.update(shape=shape, valid=True, solid_count=1, volume_mm3=shape.Volume())
    report = dict(schema_version='gesture_hand_v3.fabrication_repairs.v1',
                  completed_stages=['Ten distal retained/keyed hinge sets',
                                    'Six complete cover bolts and captured nuts; posts fused into floor']+
                                   (['Local relief-lip removal and inward proximal neck webs',
                                     'Twelve formed-metal fascia clamps and five recessed axial tip-cap attachments; one unfastenable sliver merged'] if extended else []),
                  nominal_construction_only=True, manufacture_approved=False,
                  added_names=additions, removed_names=removed,
                  modified_parts=[dict(name=name, reasons=reason, original_volume_mm3=before[name],
                                       new_volume_mm3=result[name]['shape'].Volume())
                                  for name, reason in changes.items()],
                  cylinder_interfaces=cylinders, seating_interfaces=seats,
                  flat_key_interfaces=keys, pin_retention=retained,
                  clearance_shape_probes=shape_probes, cover_fasteners=cover_interfaces,
                  structural_repairs=structural_repairs, skin_fasteners=skin_fasteners,
                  attached_skin_count=len({r['skin'] for r in skin_fasteners}),
                  cover_profile=dict(ramp_end_y_mm=cover_profile[0], wrist_front_z_mm=cover_profile[1],
                                     plateau_front_z_mm=cover_profile[2], vertical_wall_mm=cover_profile[3]),
                  remaining=([] if extended else ['Thirteen fascia and five fingertip caps still need designed attachment.',
                             'Thin relief lips and proximal neck webs require local rebuild.'])+
                            ['Local wall gauges do not establish a global minimum; a new thickness screen is required.',
                             'Source root mounting must be rechecked against the accepted root positions.',
                             'No tolerance class, thread purchase specification, clip elastic installation, load or life approval.',
                             'All repairs require actual static/gesture/transition checks after export.'],
                  assembly_order=['Preassemble distal bushes, thrust washers and keyed pin before attaching neighbouring fingers.',
                                  'Fit the opposing groove clip using a separately qualified spring-clip installation.',
                                  'Insert cover nuts through side slots before fitting the cover, then tighten the six cover bolts.'])
    return list(result.values()), report

"""Inert proposed milling features. No CAD imports or import-time operations.

Caller supplies cq and its non-destructive, one-solid-checked boolean callback.
This file is a proposal only: no shape has been built or audited from it.
"""
import math

GUIDE_RADIUS = 0.1
POCKET = {
    'x': [-25.2, -16.8], 'y': [-91.3, -87.9],
    'floor_z': -24.1, 'cut_top_z': -16.99,
    'endmill_radius': 0.5, 'floor_corner_radius': 0.05,
    'corner_relief_radius': 0.55,
}


def _bbox(shape):
    b = shape.BoundingBox()
    return [b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax]


def _one(shape, label):
    solids = shape.Solids()
    if len(solids) != 1 or not shape.isValid() or solids[0].Volume() <= 0:
        raise ValueError(label + ': expected one valid positive solid')
    return solids[0]


def _at(value, target, tol=1e-6):
    return abs(value-target) <= tol


def round_guide_corners(cq, shoe_before_holes, side):
    """Call IMMEDIATELY after the three-box shoe fuse, BEFORE all hole cuts.

    Select exactly the full 75 mm floor edge and 37 mm roof edge. No nearest-edge
    fallback: unexpected topology stops the build. Fillets are concave and add
    the small material left by a real R0.1 cutter nose, not a clearance waiver.
    """
    if side not in ('left', 'right'):
        raise ValueError('Unknown side')
    input_volume = shoe_before_holes.Volume()
    original_box = _bbox(shoe_before_holes)
    work = _one(shoe_before_holes.copy(), side + ' uncut shoe')
    wall_x = -56.15 if side == 'left' else 14.15
    targets = [('lower_guide', -21.8, -165., -90., 75.),
               ('upper_lip', -18.5, -146., -109., 37.)]
    selected, rows = [], []
    for name, z, ymin, ymax, length in targets:
        matches = []
        for edge in work.Edges():
            if edge.geomType() != 'LINE':
                continue
            bb = _bbox(edge)
            if (all(_at(a, b) for a, b in zip(bb,
                    [wall_x, ymin, z, wall_x, ymax, z]))
                    and _at(edge.Length(), length)):
                matches.append(edge)
        if len(matches) != 1:
            raise ValueError(f'{side}/{name}: expected exactly one line edge, got {len(matches)}')
        selected.extend(matches)
        rows.append(dict(id=name, edge_count=1, length_mm=length,
                         bounds_mm=_bbox(matches[0])))
    if len(selected) != 2 or selected[0].isSame(selected[1]):
        raise ValueError('Guide must have two distinct selected edges')
    rounded = work.fillet(GUIDE_RADIUS, selected)
    _one(rounded, side + ' R0.1 guide')
    if any(abs(a-b) > 2e-6 for a, b in zip(_bbox(rounded), original_box)):
        raise ValueError('Guide corner fillet changed the outside envelope')
    delta = rounded.Volume() - input_volume
    if not (0.0 < delta < 1.0):
        raise ValueError('Expected small concave added material; delta=' + str(delta))
    if abs(shoe_before_holes.Volume() - input_volume) > 1e-8:
        raise ValueError('Input shoe was mutated')
    return rounded, dict(side=side, internal_radius_mm=GUIDE_RADIUS,
        selected_line_edges=rows, added_material_mm3=delta,
        application_stage='three_box_fuse_before_any_holes',
        machining=guide_machining_spec(side), validated_by_CAD_audit=False)


def guide_machining_spec(side):
    if side not in ('left', 'right'):
        raise ValueError('Unknown side')
    sign = 1.0 if side == 'left' else -1.0
    tip_x = -56.15 if side == 'left' else 14.15
    clear_tip_x = -40.0 if side == 'left' else -2.0
    return dict(
        setup='Single detached shoe on its outer side; no other assembly parts present',
        access_axis=[sign, 0., 0.], tool_diameter_mm=3., nose_corner_radius_mm=.1,
        minimum_diameter3_neck_length_mm=14., shank_diameter_mm=6.,
        shank_span_from_tip_mm=[14., 30.],
        cutter_tip_x_mm=tip_x, clear_tip_x_mm=clear_tip_x,
        center_z_passes_mm=[-20.3, -20.0],
        center_y_start_end_mm=[-166.6, -88.4],
        operation_order=[
            'Enter along X while Y=-166.6, beyond the end of the standalone shoe.',
            'Hold the tip X at the declared wall; feed along the complete Y range.',
            'Withdraw along X beyond the inner shoe edge before changing Z.',
            'Repeat at the other declared Z, then drill mount/table-foot holes.'
        ],
        required_checks=[
            'Actual bullnose tool versus final shoe, including both Y ends and two Z passes.',
            'Cylinder neck/shank approach and tool holder envelope, with only this shoe fixtured.',
            'Selected cutter SKU must satisfy these dimensions; no purchased tool is claimed.',
            'D118/120/122 saved plate fit, bearing seats and lateral shoe insertion unchanged gates.'
        ],
        tool_route_verified=False)


def guide_side_endmill(cq, boolean, side, tip_mm):
    """Optional future audit tool: real R0.1 nose plus neck and larger shank.

    A sharp cylinder reaching the wall is NOT the same tool and falsely overlaps
    the deliberate concave guide fillet. Tool holder is not included here.
    """
    if side not in ('left', 'right'):
        raise ValueError('Unknown side')
    d = 1. if side == 'left' else -1.
    p = cq.Vector(*tip_mm)
    axis = cq.Vector(d, 0., 0.)
    neck = cq.Solid.makeCylinder(1.5, 14., p, axis)
    edge_matches = [e for e in neck.Edges()
        if e.geomType() == 'CIRCLE'
        and _at(_bbox(e)[0], tip_mm[0]) and _at(_bbox(e)[3], tip_mm[0])]
    if len(edge_matches) != 1:
        raise ValueError('Exactly one circular tool nose edge required')
    neck = neck.fillet(.1, edge_matches)
    shank_start = cq.Vector(tip_mm[0] + d*14., tip_mm[1], tip_mm[2])
    shank = cq.Solid.makeCylinder(3., 16., shank_start, axis)
    return boolean('fuse', neck, shank)


def travelling_nut_pocket_tool(cq, boolean):
    """Return the replacement U-pocket cutter, not a cut on an already voided tab.

    The old lower hex AND upper rectangular-loading cutters must be replaced by
    this cutter. Cutting it after the old pocket would retain unmachinable R0
    floor corners. Final tool comprises a milled rounded rectangle, four Ø1.1
    vertical corner escapes, and an actual R0.05 floor edge.
    """
    x0, x1 = POCKET['x']; y0, y1 = POCKET['y']
    z0, z1 = POCKET['floor_z'], POCKET['cut_top_z']
    r, relief = POCKET['endmill_radius'], POCKET['corner_relief_radius']
    def box(a, b, c, d):
        return cq.Solid.makeBox(b-a, d-c, z1-z0, cq.Vector(a, c, z0))
    def vertical(x, y, radius):
        return cq.Solid.makeCylinder(radius, z1-z0,
                                     cq.Vector(x, y, z0), cq.Vector(0, 0, 1))
    # Ø1 endmill perimeter offsets give the real four R0.5 internal XY corners.
    pieces = [box(x0+r, x1-r, y0, y1), box(x0, x1, y0+r, y1-r)]
    for x in (x0+r, x1-r):
        for y in (y0+r, y1-r):
            pieces.append(vertical(x, y, r))
    # Ø1 cutter circular-interpolates radius0.05 around each corner center.
    for x in (x0, x1):
        for y in (y0, y1):
            pieces.append(vertical(x, y, relief))
    tool = pieces[0]
    for item in pieces[1:]:
        tool = boolean('fuse', tool, item)
    tool = _one(tool.clean(), 'pocket tool before floor radius')
    bottom = [f for f in tool.Faces() if f.geomType() == 'PLANE'
              and _at(_bbox(f)[2], z0) and _at(_bbox(f)[5], z0)]
    if len(bottom) != 1:
        raise ValueError('One continuous pocket bottom face required after clean')
    bottom_edges = bottom[0].Edges()
    if len(bottom_edges) < 8:
        raise ValueError('Unexpected rounded-pocket/corner-relief perimeter')
    tool = tool.fillet(POCKET['floor_corner_radius'], bottom_edges)
    _one(tool, 'milled U pocket cutter')
    if not _at(_bbox(tool)[2], z0, 2e-6):
        raise ValueError('Pocket floor datum changed')
    meta = nut_pocket_spec()
    meta['floor_edge_count_before_fillet'] = len(bottom_edges)
    meta['cutter_volume_mm3'] = tool.Volume()
    return tool, meta


def nut_pocket_spec():
    # Analytic nominal polygon bounds only; do not replace saved-shape seats.
    hex_radius = 7. / math.sqrt(3.)
    cut_x = 4.2 - POCKET['corner_relief_radius']
    seat_before = math.sqrt(3.) * 7.**2 / 2. - math.pi*2.25**2
    tip_loss_bound = 2.*math.sqrt(3.)*(hex_radius-cut_x)**2
    nominal_rotation_limit = 30. - math.degrees(math.acos(7.2/(2.*hex_radius)))
    return dict(pocket_shape='rectangular_U_with_corner_relief',
        pocket_width_mm=8.4, pocket_depth_y_mm=3.4, capture_height_mm=7.2,
        floor_z_mm=-24.1, roof_z_mm=-16.9, nut_nominal_y_mm=[-91.3, -88.1],
        axial_seat_planes_y_mm=[-91.3, -87.9],
        rear_and_front_corner_relief_centers_xy_mm=[
            [x, y] for x in POCKET['x'] for y in POCKET['y']],
        corner_relief_diameter_mm=1.1, floor_corner_radius_mm=.05,
        machining_tool=dict(access=[0., 0., 1.], endmill_diameter_mm=1.,
            corner_radius_mm=.05, usable_diameter1_reach_min_mm=7.2,
            larger_shank_must_start_above_z_mm=-16.9,
            operation='Rough rounded rectangular U, circular-interpolate four corner escapes, finish floor and walls; deburr without changing datums',
            selected_supplier_tool=False),
        analytical_nominal_axial_seat_lower_bound_mm2=seat_before-tip_loss_bound,
        analytical_nominal_center_fixed_rotation_limit_deg=nominal_rotation_limit,
        analytical_scope='Ideal supplier-max regular hex, fixed center and parallel faces; excludes real nut chamfers, tolerances, lift/tilt, strength and actual machining errors',
        required_axial_seat_gate_mm2=20., required_lid_seat_gate_mm2=110.,
        supplemental_declaration=dict(kind='keyed_nut_pocket',
            nut='tension_adjuster_travelling_nut', pocket_part='drive_bench_baseplate',
            additional_pocket_parts=['tension_adjuster_nut_lid'], frame='carriage',
            axis_origin_mm=[-21., -91.3, -20.5], axis_direction=[0., 1., 0.],
            finite_span_y_mm=[-91.3, -88.1], nut_af_mm=7., pocket_af_mm=7.2,
            pocket_shape='rectangular_U_with_corner_relief', pocket_width_mm=8.4,
            capture_height_mm=7.2,
            description='pocket_af_mm is legacy capture-height metadata, NOT a generated hex pocket; use actual saved plate+lid for rotation'),
        required_checks=[
            'One positive valid final plate and lid; source input remains unchanged.',
            'Actual saved nut fits both axial float endpoints and top insertion.',
            'Both original20mm2 axial seats and original110mm2 lid seat remain, without lowering gates.',
            'Actual plate+lid fixed-center nut rotation and first-contact angles; no lift/tilt claim.',
            'M2 pilot/tap walls, common motion and stop faces, all prior assembly/delta checks.'
        ], geometry_built=False, manufacturing_approved=False)

"""Revision2 common support for an isolated thumb IP bench, IP-local millimetres.

Inert import: CadQuery/OCC are used only by build(cq, original_baseplate_IP_local,
foot_holes). Nothing is read, exported, regenerated, or executed on import.
The caller retains the original motor baseplate and replaces only its copied
candidate. Thirty-two other motor part files and the 120 mm axis spacing remain
outside this module's modification scope. Hardware uses supplier dimensions as
simplified envelopes, not supplier native CAD or an approved load specification.
"""
from __future__ import annotations

import math


MOTOR_HOLES = ((-50., -156.), (8., -156.), (-50., -99.), (8., -99.))
# Rear feet at Y=-140 avoid the underside M4 nuts/washers near Y=-156.
# Four corner feet at Y=-163 would overlap those fasteners below the plate.
TABLE_FEET = ((-58., -140.), (34., -140.), (-58., 4.), (34., 4.))
TABLE_SLOTS = ((-57., -70.), (34., -70.))
PLATE_BOUNDS = ((-69., -170., -36.), (44., 14., -28.))
SOCKET = dict(part_number='Wera 05004504001', designation='8790 HMA Deep 7 x 50mm',
    across_flats_mm=7., length_mm=50., front_diameter_mm=10.8,
    rear_diameter_mm=11.8, maximum_diameter_mm=11.8,
    source_url='https://hybris-media.wera.de/download/pdfgenerator-datasheets/en/05004504001.pdf',
    source_page_url='https://www.wera.de/de/werkzeuge/8790-hma-deep-steckschluesseleinsatz-mit-1-4-antrieb',
    interpretation='A full-length cylinder at the larger published diameter is used for external access; socket cavity, bolt protrusion accommodation, extension and ratchet remain separate checks.')
HARDWARE = dict(
    designation='Accu SSC-M4-20-A2/SSC-M4-25-A2, HPN-M4-A2 and HPW-M4-A2 dimension-derived envelopes',
    diameter_mm=4., pitch_mm=.7, head_diameter_mm=7., head_height_mm=4.,
    socket_across_flats_mm=3., socket_depth_mm=2.,
    nut_across_flats_mm=7., nut_height_mm=3.2,
    washer_od_mm=9., washer_id_mm=4.3, washer_thickness_mm=.9,
    washer_reference='Accu HPW-M4-A2 supplier PDF id72488: thickness0.9 +0/-0.2, ID4.3 +0.18/-0, OD9 +0/-0.3',
    washer_thickness_interpretation='Supplier maximum dimension; actual allowed thickness0.7..0.9mm. CAD stack uses0.9mm, not DIN125 nominal0.8.',
    screw_reference='SSC-M4-20-A2 product2828 and SSC-M4-25-A2 product2830: D7 +0/-0.22, H4 +0/-0.18, AF3, depth2, d4, pitch0.7, full thread',
    nut_reference='Accu HPN-M4-A2 PDF id7890: AF7 +0/-0.22, H3.2 +0/-0.3, pitch0.7',
    clearance_hole_diameter_mm=4.5,
    helical_threads=False, tip_chamfers_and_incomplete_threads_modelled=False,
    source_status='Official supplier dimensions checked by model_audit; caller must preserve/hash-bind originals copied from work/thumb_ip_bench_v1/hardware before publication.')
WASHER_T = HARDWARE['washer_thickness_mm']


def build(cq, original_baseplate_IP_local, foot_holes):
    """Return parts, intended contacts, assembly declarations and design scope.

    foot_holes: iterable of dictionaries containing xy=[x,y], underhead_z
    (INCLUDING the 0.9 mm upper washer), bolt_length_mm, and contact_part or part.
    The supplied parent foot must seat on Z=-28 and have its upper face at
    underhead_z-0.9. It is supplied by the saddle module, not reconstructed here.
    Parts have name/shape/body='fixed'/role/color. Contacts are required physical
    contacts, never collision exemptions. All operations use copied arguments.
    """
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_OUT
    from OCP.TopTools import TopTools_ListOfShape

    parts, contacts, assembly = [], [], []
    names = set()

    def facts(s):
        solids = s.Solids()
        positive = all(x.Volume() > 1e-7 for x in solids)
        bounded = True
        for solid in solids:
            classifier = BRepClass3d_SolidClassifier(solid.wrapped)
            classifier.PerformInfinitePoint(1e-7)
            bounded &= classifier.State() == TopAbs_OUT
        return dict(valid=s.isValid(), solid_count=len(solids),
                    volume_mm3=sum(x.Volume() for x in solids),
                    positive=positive, bounded=bool(bounded))

    def check(s, label):
        f = facts(s)
        if not (f['valid'] and f['solid_count'] == 1 and f['positive'] and f['bounded']):
            raise ValueError('Invalid, inverted or disconnected part: '+label+' '+str(f))
        return f

    def op(kind, left, *right):
        before = [facts(s)['volume_mm3'] for s in (left, *right)]
        operation = {'cut': BRepAlgoAPI_Cut, 'fuse': BRepAlgoAPI_Fuse}[kind]()
        aa, bb = TopTools_ListOfShape(), TopTools_ListOfShape()
        aa.Append(left.copy().wrapped)
        for tool in right:
            bb.Append(tool.copy().wrapped)
        operation.SetArguments(aa); operation.SetTools(bb)
        operation.SetNonDestructive(True); operation.SetRunParallel(False)
        operation.Build()
        if not operation.IsDone():
            raise ValueError('Incomplete '+kind)
        result = cq.Shape.cast(operation.Shape())
        after = [facts(s)['volume_mm3'] for s in (left, *right)]
        if any(abs(a-b) > 1e-8 for a, b in zip(before, after)):
            raise ValueError('Operation changed an input')
        check(result, kind+' output')
        if kind == 'cut' and facts(result)['volume_mm3'] > before[0]+1e-6:
            raise ValueError('Cut added material')
        return result

    def box(x0, x1, y0, y1, z0, z1):
        return cq.Solid.makeBox(x1-x0, y1-y0, z1-z0, cq.Vector(x0, y0, z0))

    def cylinder(x, y, radius, z0, z1):
        return cq.Solid.makeCylinder(radius, z1-z0, cq.Vector(x, y, z0), cq.Vector(0, 0, 1))

    def hex_prism(x, y, af, z0, z1):
        # Vertex circumradius=AF/sqrt(3), rather than treating AF as diameter.
        radius = af/math.sqrt(3.)
        points = [(x+radius*math.cos(i*math.pi/3), y+radius*math.sin(i*math.pi/3)) for i in range(6)]
        return cq.Workplane('XY').workplane(offset=z0).polyline(points).close().extrude(z1-z0, clean=False).val()

    def ring(x, y, outer, inner, z0, z1):
        return op('cut', cylinder(x, y, outer, z0, z1), cylinder(x, y, inner, z0-.01, z1+.01))

    def screw(x, y, underhead, length):
        body = op('fuse', cylinder(x, y, 2., underhead-length, underhead),
                  cylinder(x, y, 3.5, underhead, underhead+4.))
        return op('cut', body, hex_prism(x, y, 3., underhead+2., underhead+4.01))

    def nut(x, y, z0):
        return op('cut', hex_prism(x, y, 7., z0, z0+3.2), cylinder(x, y, 2., z0-.01, z0+3.21))

    def add(name, shape, role, color=(.62, .66, .70)):
        if name in names:
            raise ValueError('Duplicate part '+name)
        names.add(name)
        parts.append(dict(name=name, shape=shape, body='fixed', role=role,
                          color=list(color), geometry=check(shape, name)))
        return name

    def contact(a, b, area, description, z=None, xy=None):
        row = dict(a=a, b=b, minimum_area_mm2=area, description=description)
        if z is not None: row['plane_z_mm'] = z
        if xy is not None: row['patch_center_xy_mm'] = list(xy)
        if z is not None and xy is not None:
            row['plane_origin_mm'] = [xy[0], xy[1], z]
            row['normal'] = [0., 0., 1.]
        contacts.append(row)

    def screw_declaration(name, x, y, underhead, length, excluded, description):
        assembly.append(dict(kind='axial_screw', part=name, underhead=[x, y, underhead],
            axis=[0., 0., 1.], shaft_radius_mm=2., head_radius_mm=3.5,
            head_height_mm=4., length_mm=length, entry_travel_mm=20.,
            driver_radius_mm=2., driver_length_mm=40., socket_across_flats_mm=3.,
            stage_excluded=list(excluded), description=description))

    def hardware(prefix, x, y, underhead, length, nut_top, lower_washer=True,
                 excluded=(), description=''):
        if not all(math.isfinite(v) for v in (x, y, underhead, length, nut_top)) or length <= 0:
            raise ValueError('Invalid screw stack')
        nut_bottom = nut_top-3.2
        tip = underhead-length
        if not (-54. <= tip <= nut_bottom-.5):
            raise ValueError('Insufficient nominal nut projection or table clearance: '+prefix)
        sn = add(prefix+'_screw', screw(x, y, underhead, length), 'nominal_M4_socket_screw_major_envelope')
        uw = add(prefix+'_upper_washer', ring(x, y, 4.5, 2.15, underhead-WASHER_T, underhead), 'M4_flat_washer_supplier_maximum_thickness')
        nn = add(prefix+'_nut', nut(x, y, nut_bottom), 'nominal_M4_hex_nut_major_envelope')
        contact(sn, uw, 22., 'Socket head underside seats on upper washer', underhead, (x, y))
        lw = None
        if lower_washer:
            lw = add(prefix+'_lower_washer', ring(x, y, 4.5, 2.15, nut_top, nut_top+WASHER_T), 'M4_flat_washer_supplier_maximum_thickness')
            contact(nn, lw, 26., 'Nut upper face seats on lower washer', nut_top, (x, y))
        screw_declaration(sn, x, y, underhead, length, excluded, description)
        assembly.append(dict(kind='axial_nut', part=nn, mate_screw=sn,
            axis=[0., 0., -1.], upper_face=[x, y, nut_top], across_flats_mm=7.,
            height_mm=3.2, path_travel_mm=30., entry_travel_mm=30.,
            socket_max_dia_mm=11.8, socket_length_mm=50.,
            source_url=SOCKET['source_url'], socket_part_number=SOCKET['part_number'],
            stage='before_table_placement', stage_excluded=[nn, sn],
            description='Replay actual nut outer-prism insertion from below and a full-length OD11.8 x50 socket envelope against all other parts; exclude only this nut and its mating bolt. Before placing on the table. This declaration is not a pass result. Internal socket engagement/bolt-tip room and ratchet space remain unverified.'))
        return dict(screw=sn, upper=uw, lower=lw, nut=nn, tip_z_mm=tip,
                    nut_span_z_mm=[nut_bottom, nut_top], nominal_tip_projection_mm=nut_bottom-tip)

    check(original_baseplate_IP_local, 'original baseplate')
    old_volume = original_baseplate_IP_local.Volume()
    bb = original_baseplate_IP_local.BoundingBox()
    actual = (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax)
    expected = (-56., 14., -162.5, -92.5, -21.8, -18.8)
    if any(abs(a-b) > .02 for a, b in zip(actual, expected)):
        raise ValueError('Input must be unchanged motor baseplate already transformed into IP-local coordinates: '+str(actual))

    normalized = []
    for row in foot_holes:
        x, y = map(float, row['xy'])
        uz, length = float(row['underhead_z']), float(row['bolt_length_mm'])
        part = row.get('contact_part', row.get('part'))
        if not isinstance(part, str) or not part:
            raise ValueError('Each parent foot hole needs contact_part or part')
        if not all(math.isfinite(v) for v in (x, y, uz, length)) or uz-WASHER_T <= -28.:
            raise ValueError('Invalid parent-foot geometry')
        if not (-59. <= x <= 36. and -164. <= y <= 6.):
            raise ValueError('Parent foot hardware does not fit plate with washer edge margin')
        normalized.append(dict(xy=[x, y], underhead_z=uz, bolt_length_mm=length, contact_part=part))
    if not normalized or len({tuple(r['xy']) for r in normalized}) != len(normalized):
        raise ValueError('Parent foot holes must be nonempty and unique')

    common = box(-69., 44., -170., 14., -36., -28.)
    modified = original_baseplate_IP_local.copy()
    all_holes = list(MOTOR_HOLES)+list(TABLE_FEET)+[tuple(r['xy']) for r in normalized]
    for i, a in enumerate(all_holes):
        if any(math.dist(a, b) < 10. for b in all_holes[i+1:]):
            raise ValueError('Fastener centers too close for nominal washer/tool envelopes')
    for x, y in all_holes:
        common = op('cut', common, cylinder(x, y, 2.25, -36.01, -27.99))
    for x, y in MOTOR_HOLES:
        modified = op('cut', modified, cylinder(x, y, 2.25, -21.81, -18.79))
    # Two closed 18 x 6.6 slots for an external shop clamping fixture. Its actual
    # bolts/clamps/table are intentionally NOT represented as installed hardware.
    for x, y in TABLE_SLOTS:
        slot = cq.Workplane('XY').workplane(offset=-36.01).center(x, y).slot2D(18., 6.6, 90.).extrude(8.02, clean=False).val()
        common = op('cut', common, slot)
    common_name = add('bench_common_plate', common, 'common_plate_with_external_table_clamp_slots', (.24, .30, .36))
    motor_plate = add('drive_bench_baseplate', modified, 'modified_copy_four_additional_M4_clearance_holes')

    motor_stacks = []
    for i, (x, y) in enumerate(MOTOR_HOLES, 1):
        prefix = 'bench_motor_mount_'+str(i)
        spacer = add(prefix+'_spacer', ring(x, y, 5., 2.25, -28., -21.8), 'nominal_OD10_ID4p5_h6p2_spacer')
        hs = hardware(prefix, x, y, -18.8+WASHER_T, 25., -36.-WASHER_T, description='Install motor module on four spacers after table feet, before parent coupon and cords; access from +Z.')
        contact(spacer, common_name, 60., 'Spacer lower annular seat', -28., (x, y))
        contact(spacer, motor_plate, 60., 'Spacer upper annular seat against original plate underside', -21.8, (x, y))
        contact(hs['upper'], motor_plate, 45., 'Upper washer against modified motor baseplate', -18.8, (x, y))
        contact(hs['lower'], common_name, 45., 'Lower washer against common plate underside', -36., (x, y))
        motor_stacks.append(dict(xy=[x, y], spacer=spacer, **hs))

    table_stacks = []
    for i, (x, y) in enumerate(TABLE_FEET, 1):
        prefix = 'bench_table_foot_'+str(i)
        foot = box(x-9., x+9., y-9., y+9., -55., -36.)
        foot = op('cut', foot, cylinder(x, y, 2.25, -55.01, -35.99))
        foot = op('cut', foot, cylinder(x, y, 6.2, -55.01, -45.))
        fn = add(prefix, foot, 'removable_18x18x19_support_with_OD12p4_bottom_nut_tool_counterbore', (.30, .35, .40))
        hs = hardware(prefix, x, y, -28.+WASHER_T, 25., -45., lower_washer=False,
            excluded=['drive_bench_baseplate'],
            description='Preassemble feet to common plate before the motor module; insert nut and socket from underside before placing on table.')
        contact(fn, common_name, 300., 'Foot full upper seat on plate underside', -36., (x, y))
        contact(hs['upper'], common_name, 45., 'Upper washer on common plate', -28., (x, y))
        contact(hs['nut'], fn, 25., 'Nut seats on internal counterbore shoulder', -45., (x, y))
        table_stacks.append(dict(xy=[x, y], part=fn, counterbore_diameter_mm=12.4,
            foot_width_depth_height_mm=[18., 18., 19.],
            nominal_counterbore_radial_wall_mm=2.8,
            nominal_socket_radial_clearance_mm=.3,
            counterbore_span_z_mm=[-55., -45.], table_contact_plane_z_mm=-55., **hs))

    parent_stacks = []
    for i, row in enumerate(normalized, 1):
        x, y = row['xy']; uz = row['underhead_z']
        hs = hardware('bench_parent_mount_'+str(i), x, y, uz, row['bolt_length_mm'], -36.-WASHER_T,
            description='Clamp the detached coupon between split jaws first, then place and fasten the foot stack while the X-adjustment slots can align the contacts. Horizontal jaw hardware precedes foot mounting hardware. Positive-Z insertion and driver access still require scene audit; this proposed order is not approved.')
        contact(row['contact_part'], common_name, 150., 'Actual parent foot bottom must seat substantially on plate; area gate does not establish strength', -28., (x, y))
        contact(hs['upper'], row['contact_part'], 40., 'Upper washer on actual parent foot; supplied underhead includes supplier maximum0.9mm washer', uz-WASHER_T, (x, y))
        contact(hs['lower'], common_name, 45., 'Parent mounting lower washer on common plate', -36., (x, y))
        parent_stacks.append(dict(**row, **hs))

    if abs(original_baseplate_IP_local.Volume()-old_volume) > 1e-8:
        raise ValueError('Original motor baseplate changed')
    return dict(parts=parts, replace_names=['drive_bench_baseplate'], contacts=contacts,
        assembly=assembly, spec=dict(
            schema='thumb_ip_common_base.v2', units='IP-local millimetres',
            previous_source_sha256='549dd8fe9826c98ece44c414433dd14d04652743bd2c705b3363aa27cc5fd8dc',
            common_plate_bounds_mm=[list(v) for v in PLATE_BOUNDS],
            motor_mount_holes_xy_mm=[list(v) for v in MOTOR_HOLES],
            motor_spacer_od_id_height_mm=[10., 4.5, 6.2], motor_stacks=motor_stacks,
            parent_foot_stacks=parent_stacks, table_foot_stacks=table_stacks,
            table_plane_z_mm=-55., lowest_new_screw_tip_z_mm=min(r['tip_z_mm'] for r in motor_stacks+table_stacks+parent_stacks),
            nominal_table_tip_clearance_mm=55.+min(r['tip_z_mm'] for r in motor_stacks+table_stacks+parent_stacks),
            old_motor_bolt_tip_z_mm=-25., common_top_z_mm=-28.,
            nominal_old_bolt_to_plate_clearance_mm=3.,
            table_clamp_slots=[dict(center_xy_mm=list(p), length_mm=18., width_mm=6.6,
                                    angle_deg=90.) for p in TABLE_SLOTS],
            table_clamps_and_table_modelled=False, tabletop_attachment_complete=False,
            hardware_assumptions=dict(HARDWARE),
            socket_access_tool=dict(SOCKET),
            source_file_reuse='Original motor baseplate retained externally; only its copy receives four holes. Other 32 motor parts remain unchanged.',
            motor_axis_spacing_mm=120., original_baseplate_volume_mm3=old_volume,
            modified_baseplate_volume_mm3=modified.Volume(),
            operation_scope='Non-destructive shape construction only; no collision, assembly path, load or physical qualification is implied.',
            limitations=[
                'M4 hardware uses supplier nominal/maximum dimensions with smooth major-envelope threads; tolerance combinations and incomplete thread geometry are not simulated.',
                'Screw/nut insertion and external socket access entries request independent replay; this generator does not claim those paths passed.',
                'Published socket external dimensions do not specify the internal bore length available around a projecting screw tip; actual engagement and extension/ratchet space remain unverified.',
                'Feet provide physical bottom faces at Z=-55; actual table flatness, rocking, slip and overturning remain unmeasured.',
                'Two clamp slots are provided, but the external shop clamps/table attachment are absent and must be specified before powered use.',
                'The new saddle/base/fasteners must be checked against actual cords, moving IP assembly and motor before acceptance.',
                'Saddle-foot slot fit, preload, plate bending, foot counterbore strength and fastener retention are not qualified.'
            ]))

"""Inert CAD definition of a separate short, custom adjustment key.

Public API: ``build(cq) -> cq.Shape`` and ``metadata() -> dict``.
No CAD import, build, export, process, file access or hardware I/O occurs on
import. The caller supplies CadQuery and owns saved-STEP validation and audits.
Coordinates are the existing IP-local millimetre frame. This removable tool is
NOT one of the physical tension-bench assembly parts and is not motor hardware.
"""
from __future__ import annotations

from copy import deepcopy
import math


P = {
    'axis_x_mm': -21.0,
    'axis_z_mm': -20.5,
    'axis_reference_y_mm': -73.0,
    'socket_front_y_mm': -69.0,
    'socket_depth_mm': 2.0,
    'socket_across_flats_mm': 3.0,
    'tip_y_mm': -70.5,
    'hex_across_flats_mm': 2.9,
    'tip_lead_length_mm': 0.2,
    'tip_lead_angle_deg': 45.0,
    'knob_rear_y_mm': -59.0,
    'knob_front_y_mm': -53.0,
    'knob_diameter_mm': 14.0,
    # This is construction overlap inside the knob, not a separate component.
    'hex_into_knob_overlap_mm': 0.2,
    'entry_translation_mm': 12.0,
    'signed_travel_mm_per_revolution': -0.7,
}


def metadata():
    """Return serializable, nominal geometry and explicit unverified scope."""
    p = deepcopy(P)
    x, z = p['axis_x_mm'], p['axis_z_mm']
    af = p['hex_across_flats_mm']
    lead = p['tip_lead_length_mm']
    tip_af = af - 2.0 * lead * math.tan(math.radians(p['tip_lead_angle_deg']))
    return {
        'schema': 'thumb_ip_tension_custom_adjustment_key.v1',
        'name': 'tension_custom_adjustment_key',
        'units': 'mm, degree; existing IP-local frame',
        'api': 'build(cq) -> single Shape; metadata() -> independent dictionary',
        'body': 'adjuster_rotor',
        'role': 'separately_stored_removable_custom_tool',
        'included_in_physical_assembly': False,
        'included_in_assembly_mass': False,
        'storage_location': 'off fixture; not modelled',
        'intended_material': 'steel; alloy, hardness and heat treatment not selected',
        'supplier_cad': False,
        'parameters': p,
        'adjuster_axis': [x, p['axis_reference_y_mm'], z],
        'adjuster_axis_direction': [0.0, 1.0, 0.0],
        'travel_mm_per_revolution': p['signed_travel_mm_per_revolution'],
        'phase_reference_part': 'tension_adjuster_screw',
        'phase_zero': 'Hex vertices use (X,Z)=(x+r*cos(i*pi/3),z+r*sin(i*pi/3)), matching the actual socket definition.',
        'rotation_convention': 'Rotate tool by the same +Y-axis angle as the lead-screw head: q=(120-D)/(-0.7)*360 degrees. Do not reset phase independently.',
        'nominal_geometry': {
            'tip_y_mm': p['tip_y_mm'],
            'tip_across_flats_mm': tip_af,
            'lead_span_y_mm': [p['tip_y_mm'], p['tip_y_mm'] + lead],
            'full_hex_exposed_span_y_mm': [p['tip_y_mm'] + lead, p['knob_rear_y_mm']],
            'full_hex_corner_diameter_mm': 2.0 * af / math.sqrt(3.0),
            'knob_span_y_mm': [p['knob_rear_y_mm'], p['knob_front_y_mm']],
            'assembly_bbox_mm': [[x-7.0, p['tip_y_mm'], z-7.0],
                                [x+7.0, p['knob_front_y_mm'], z+7.0]],
            'tip_socket_penetration_mm': 1.5,
            'full_hex_socket_overlap_mm': 1.3,
            'tip_to_socket_floor_mm': 0.5,
            'nominal_per_flat_clearance_mm': (p['socket_across_flats_mm']-af)/2.0,
            'cap_passage_radius_mm': 2.1,
            'minimum_nominal_hex_corner_to_cap_radial_gap_mm': 2.1-af/math.sqrt(3.0),
            'knob_to_common_plate_top_nominal_z_gap_mm': 0.5,
            'knob_to_cap_front_nominal_y_gap_mm': 5.9,
        },
        'assembly': [{
            'kind': 'sampled_translation',
            'part': 'tension_custom_adjustment_key',
            'initial_translation_mm': [0.0, p['entry_translation_mm'], 0.0],
            'final_translation_mm': [0.0, 0.0, 0.0],
            'maximum_step_mm': 0.5,
            'stage': 'temporary_tool_after_fixture_assembly_and_with_carrier_locks_released',
            'stage_excluded': [],
            'access_frame': 'fixed',
            'orientation': 'Use actual lead-screw hex phase at each D; hold that orientation during axial insertion.',
            'withdrawal': 'Reverse the same path after adjustment; remove tool before service.',
            'nominal_start_tip_y_mm': -58.5,
            'nominal_start_knob_front_y_mm': -41.0,
            'description': 'Actual whole saved key, not a straight cylindrical driver surrogate; check against every assembled physical part including target screw and cap.',
        }],
        'required_checks': [
            'Export/reimport the key separately: positive signed volume, bounded solid, valid and exactly one connected solid; preserve source hashes.',
            'Compare actual hex faces/lead and finite overlap with actual screw socket; intentional empty socket clearance must not exempt a key/screw material collision.',
            'At every selected D, phase key with screw and check insertion +Y12 to0 against all assembled physical parts. Test same path in reverse.',
            'Check the entire key rotation while engaged; the circular knob is rotation invariant but its hex stem is not. Discrete phases alone are not continuous clearance.',
            'Repeat at lead-screw axial-play endpoints (nominal0/+0.1 mm) with the key following the screw; preserve insertion depth and measure all changed gaps.',
            'Measure minimum gap to parent saddle and common plate. The declared0.5 mm plate gap is nominal geometry, not tolerance approval.',
            'Verify practical finger access, controllable grip, tool removal, corner finish, torsional capacity, wear and actual supplier socket fit before physical use.',
        ],
        'manufacturing_plan': [
            'Separate one-piece custom steel tool; start from a round blank at least14 mm finished diameter, with machining allowance.',
            'Turn the6 mm long round knob and the reduced shaft blank coaxially; preserve enough radial stock for the2.9 mm across-flats hex.',
            'Index six side-milling operations at60 degrees (or three opposed-flat setups) to form the straight hex shaft. Do not model the shaft as a bonded insert.',
            'Machine the0.2 mm axial45-degree six-face nose lead; deburr without reducing the declared1.3 mm full-hex overlap further.',
            'The nominal hex-to-knob shoulder is sharp in this candidate. Select a realizable cutter/root radius and verify resulting geometry/strength before manufacture; this CAD is not a completed shop drawing.',
            'Measure actual AF, straightness, socket engagement and knob finish. Steel grade, heat treatment, tolerances and torque limit remain to be specified.',
        ],
        'limitations': [
            'Replaces neither the failed50 mm straight-driver evidence nor any purchased driver drawing. That failed access case must remain recorded.',
            'Separate tool only: do not add to the158 physical bench parts or leave installed as a motor/adjuster component.',
            'No human hand envelope, friction grip, torsional strength, fatigue, tool life, preload force or50/100 ms performance has been verified.',
            'No assumption that nominal0.05 mm per-flat clearance equals a selected supplier fit or manufacturing tolerance.',
            'No physical slip/preload control is established; unload as needed, release separate carriage locks, adjust, relock and remove tool in a separately verified procedure.',
        ],
        'build_executed_by_author': False,
        'physical_use_approved': False,
    }


def build(cq):
    """Build the one-piece tool only when explicitly called by the owner.

    Uses a ruled three-section hex loft for the short nose lead and straight
    shaft, then a real0.2 mm overlap into the cylindrical knob. The serial OCC
    fuse is not followed by clean(), shape fixing or absolute-volume masking.
    """
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_OUT
    from OCP.TopTools import TopTools_ListOfShape

    p = P
    x, z = p['axis_x_mm'], p['axis_z_mm']
    af, lead = p['hex_across_flats_mm'], p['tip_lead_length_mm']
    tip_af = af-2.0*lead*math.tan(math.radians(p['tip_lead_angle_deg']))
    if not 0.0 < tip_af < af or not p['tip_y_mm']+lead < p['knob_rear_y_mm']:
        raise ValueError('Invalid short-key dimensions')

    def hex_wire(y, across_flats):
        r = across_flats/math.sqrt(3.0)
        vertices = [cq.Vector(x+r*math.cos(i*math.pi/3.0), y,
                              z+r*math.sin(i*math.pi/3.0)) for i in range(6)]
        return cq.Wire.makePolygon(vertices+[vertices[0]])

    shaft = cq.Solid.makeLoft([
        hex_wire(p['tip_y_mm'], tip_af),
        hex_wire(p['tip_y_mm']+lead, af),
        hex_wire(p['knob_rear_y_mm']+p['hex_into_knob_overlap_mm'], af),
    ], ruled=True)
    knob = cq.Solid.makeCylinder(
        p['knob_diameter_mm']/2.0,
        p['knob_front_y_mm']-p['knob_rear_y_mm'],
        cq.Vector(x, p['knob_rear_y_mm'], z), cq.Vector(0.0, 1.0, 0.0))
    operation = BRepAlgoAPI_Fuse()
    arguments, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    arguments.Append(shaft.wrapped)
    tools.Append(knob.wrapped)
    operation.SetArguments(arguments)
    operation.SetTools(tools)
    operation.SetNonDestructive(True)
    operation.SetRunParallel(False)
    operation.Build()
    if not operation.IsDone():
        raise ValueError('Short-key fuse did not complete')
    result = cq.Shape.cast(operation.Shape())
    solids = result.Solids()
    if not result.isValid() or len(solids) != 1 or solids[0].Volume() <= 0.0:
        raise ValueError('Short key must be valid, positive and one connected solid')
    classifier = BRepClass3d_SolidClassifier(solids[0].wrapped)
    classifier.PerformInfinitePoint(1e-7)
    if classifier.State() != TopAbs_OUT:
        raise ValueError('Short key is not a bounded material solid')
    return result

"""One explicit stationary-terminal rotation, with a real bridge to its M3 seats."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('_arm056_prior', ROOT/'cad/radial_turnback_proposal_v1/model.py')
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n, np, cq, d = a.n, a.np, a.cq, a.d
terminal = n.m.drive.module('_arm056_terminal', ROOT/'cad/motor_tendon_terminal_v1/terminal_selected.py')
FRAME = 'thumb_ip_twin_sheath_terminal_frame'
NEW_FRAME = 'arm056_stationary_terminal_bridge'
ANGLE = 90.


def members():
    names = {FRAME, 'arm055_split_return_0', 'arm055_split_return_1'}
    for i in [0, 1]:
        names |= {'thumb_ip_barrel_adjuster_'+str(i), 'thumb_ip_barrel_top_nut_'+str(i),
                  'thumb_ip_barrel_bottom_nut_'+str(i), 'motor_housing_clamp_'+str(i)+'_jaw_1'}
        names |= {'motor_housing_clamp_'+str(i)+'_'+kind+'_'+str(j)
                  for kind in ['bolt', 'nut'] for j in [0, 1]}
    assert len(names) == 19
    return names


def context():
    current, base, retained, T, older, source = a.context()
    prior = n.p.read(ROOT/'cad/radial_turnback_proposal_v1/reviews/check.json')
    guides = n.c.load(n.p.read(ROOT/prior['scene'])['parts'], source)
    baseline = {k: list(v) for k, v in retained.items()}
    baseline['forearm'] += guides
    assert sum(map(len, baseline.values())) == 872
    fixed = {k: [z for z in v if z['name'] not in members()] for k, v in baseline.items()}
    packet = [z for v in baseline.values() for z in v if z['name'] in members()]
    assert len(packet) == 19 and sum(map(len, fixed.values())) == 853
    S0 = d.around(terminal.M[:3, 3], (0, 0, 1), ANGLE)
    S = T@S0@np.linalg.inv(T)
    for rel in ['cad/radial_turnback_proposal_v1/reviews/check.json',
                'cad/motor_tendon_terminal_v1/terminal_selected.py',
                'cad/motor_spool_coupling_v1/selected/reviews/check.json']:
        source[rel] = n.m.drive.sha(ROOT/rel)
    return current, baseline, fixed, packet, T, S0, S, prior, older, source


def annular_bridge(xy, centre, z):
    v = np.array(xy)-centre
    rad = np.linalg.norm(v)
    phi = np.arctan2(v[1], v[0])
    def point(r, t):
        return cq.Vector(*(centre+r*np.array([np.cos(t), np.sin(t)])), z)
    end = phi+np.pi/2
    edges = [cq.Edge.makeThreePointArc(point(rad+3., phi), point(rad+3., (phi+end)/2), point(rad+3., end)),
             cq.Edge.makeLine(point(rad+3., end), point(rad-3., end)),
             cq.Edge.makeThreePointArc(point(rad-3., end), point(rad-3., (phi+end)/2), point(rad-3., phi)),
             cq.Edge.makeLine(point(rad-3., phi), point(rad+3., phi))]
    return cq.Solid.extrudeLinear(cq.Wire.assembleEdges(edges), [], cq.Vector(0, 0, 1.))


def build(packet, T, S0, S):
    original = next(z for z in packet if z['name'] == FRAME)['shape']
    rotated = n.g.original.transform(original, S)
    bridge = rotated
    seat_points = []
    for y in [-287., -261.]:
        xy = np.array([-11., y])
        old_pad = terminal.cyl(3., -82., -81., xy)
        arm = annular_bridge(xy, terminal.M[:2, 3], -82.)
        for shape in [old_pad, arm]:
            bridge = bridge.fuse(n.g.original.transform(shape, T))
        hole = terminal.cyl(1.7, -82.01, -80.99, xy)
        bridge = bridge.cut(n.g.original.transform(hole, T))
        seat_points.append(dict(original_seat=(T@np.r_[xy, -81., 1.])[:3],
            rotated_frame_hole=(T@S0@np.r_[xy, -81., 1.])[:3],
            bridge_radius_mm=float(np.linalg.norm(xy-terminal.M[:2, 3])),
            axial_stack_mm=1., circumferential_width_mm=6.))
    bridge = bridge.clean()
    assert bridge.isValid() and len(bridge.Solids()) == 1
    lost = rotated.cut(bridge, tol=1e-6)
    assert not lost.Solids() or lost.Volume() < 1e-8
    new_frame = dict(name=NEW_FRAME, shape=bridge, body='forearm', replaces=FRAME,
        role='rotated_original_terminal_with_integral_stationary_M3_bridge', color=[.93, .48, .1])
    moved = [dict(z, shape=n.g.original.transform(z['shape'], S)) for z in packet if z['name'] != FRAME]
    assert len(moved) == 18
    design = dict(stationary_terminal_rotation_current_world_Z_deg=ANGLE,
        stationary_terminal_rotation_proposal_axis=(T[:3, :3]@np.array([0., 0., 1.])),
        transform_from_ARM055=S, transform_in_original_world=S0,
        moved_hardware_names=[z['name'] for z in moved], replaced_frame=FRAME,
        retained_original_frame_material_removed_mm3=lost.Volume() if lost.Solids() else 0.,
        actual_M3_seats=seat_points, bridge_width_thickness_mm=[6., 1.],
        original_M3_fasteners_moved=False, rotated_original_frame_holes_have_no_extra_fasteners=True,
        motor_installation_changed=False, motor_command_angle_deg=0.,
        nominal_reaction='Housing -> preserved integral jaw/opposite jaw/M2 -> captured adjuster -> rotated original R5 terminal frame -> integral two 90-degree annular bridges -> original M3 seats/bolts -> unchanged thumb adapter -> ARM054 carrier -> beam/roll flange. Strength/preload/friction/tools/assembly unverified.',
        assembly_proposal='Frame/adjuster/jaw kit must be mounted on the original M3 seats before the thumb motor/output and local cords. This ordering is a proposal only; neutral obstruction is tested before any service path. Loose kit requires manual support.',
        full_arm_or_terminal_adopted=False)
    return [new_frame]+moved, design


def cords(T, S0):
    rows, details = [], []
    route = terminal.route
    anchors = n.p.read(ROOT/'cad/motor_spool_coupling_v1/selected/reviews/check.json')['design']['anchors']
    def point(M, p):
        return (M@np.r_[p, 1.])[:3]
    for port, mx, side, ma, mb in zip(terminal.channels(), [4., 12.], [-1., 1.], [-210., 210.], [-180., 0.]):
        i = port['cord']
        aa, bb = np.deg2rad([ma, mb])
        A = route.circle(terminal.M[:3, 3], terminal.M[:3, :3], mx, 10., aa)
        B = route.circle(terminal.M[:3, 3], terminal.M[:3, :3], mx, 10., bb)
        tail = route.point(np.array(anchors[i]['tail_exit_mm']))
        wrap = side*(aa-bb)
        assert 0 < wrap < 2*np.pi
        old_exit = point(S0, port['exit'])
        assert np.linalg.norm(B-old_exit) < 1e-10
        pieces = [route.line(tail, A, 'unchanged_anchor_tail'),
                  route.arc(A, route.circle(terminal.M[:3, 3], terminal.M[:3, :3], mx, 10., (aa+bb)/2), B, 10.*wrap, 'new_R10_wrap')]
        # Rotate the actual former 18-mm tangent, R5 guide and adjuster bore together.
        old_pieces = [route.line(port['exit'], port['entry'], 'free_tangent'), *port['pieces'],
                      route.line(port['end'], [*port['xy'], -163.], 'adjuster_passage')]
        for j, seg in enumerate(pieces):
            rows.append(dict(name='arm056_cord_'+str(i)+'_'+str(j), physical_line=i,
                segment=j, shape=n.g.original.transform(terminal.sweep_piece(seg, .3), T),
                body='forearm', role=seg['label'], length_mm=seg['length_mm'], evidence_reference_not_hardware=True,
                color=[.97,.25,.1] if i == 0 else [.1,.65,.95]))
        for j, seg in enumerate(old_pieces, 2):
            rows.append(dict(name='arm056_cord_'+str(i)+'_'+str(j), physical_line=i,
                segment=j, shape=n.g.original.transform(terminal.sweep_piece(seg, .3), T@S0),
                body='forearm', role=seg['label'], length_mm=seg['length_mm'], evidence_reference_not_hardware=True,
                color=[.97,.25,.1] if i == 0 else [.1,.65,.95]))
        details.append(dict(line=i, groove_radius_mm=10., groove_canonical_axial_mm=mx,
            anchor_angle_deg=ma, actual_tangent_angle_deg=mb, wrap_angle_deg=float(np.rad2deg(wrap)),
            tangent_exit_world_mm=point(T, B), R5_entry_world_mm=point(T@S0, port['entry']),
            shoulder_world_mm=point(T@S0, [*port['xy'], -158.]),
            adjuster_mouth_world_mm=point(T@S0, [*port['xy'], -163.]),
            tangent_length_mm=18., R5_radius_mm=5.,
            local_cord_length_mm=sum(z['length_mm'] for z in rows if z['physical_line'] == i),
            previous_local_length_mm=sum(z['length_mm'] for z in terminal.cords() if z['cord'] == i),
            inherited_anchor_tail_kink_deg=26.565051177078,
            spool_phase_and_command_changed=False, full_rotation_or_long_material_motion_verified=False))
    assert len(rows) == 12 and all(z['shape'].isValid() and len(z['shape'].Solids()) == 1 for z in rows)
    return rows, details

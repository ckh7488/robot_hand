"""Separate two-station layout with complete packets and outward electrical hardware."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('_arm054_prior', ROOT/'cad/circumferential_layout_v1/model.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
n, np, cq, d = prior.n, prior.np, prior.cq, prior.d
ROWS = [-210., -140.]
AZIMUTHS = [45.*i for i in range(8)]
RADIUS = 55.
STATIONS = [-204.5, -162.5]
ORDER = [
    ['index_mcp', 'little_mcp', 'thumb_mcp', 'little_pip', 'little_dip', 'ring_pip', 'ring_dip', 'middle_pip'],
    ['thumb_ip', 'middle_dip', 'index_pip', 'index_dip', 'index_spread', 'middle_spread', 'ring_spread', 'little_spread'],
]


def placement(modules, grouped):
    lookup = {z['name']: z for z in modules}
    reference = np.array([-14.4, -7.5, -3.9])
    rows, contract = [], []
    for j, names in enumerate(ORDER):
        for bank, stem in enumerate(names):
            name = stem+'_actuator'
            theta = np.deg2rad(AZIMUTHS[bank])
            radial = np.array([np.cos(theta), 0., np.sin(theta)])
            axial = np.array([0., 1., 0.])
            tangent = np.cross(radial, axial)
            reverse = stem == 'thumb_ip'
            # Flip the entire thumb packet, including both R40 guides and M3 stack.
            rotation = np.column_stack([-axial if reverse else axial,
                                        tangent if reverse else -tangent, -radial])
            assert abs(np.linalg.det(rotation)-1.) < 1e-12
            centre = radial*RADIUS+np.array([0., ROWS[j], 0.])
            new = np.eye(4)
            new[:3, :3] = rotation
            new[:3, 3] = centre-rotation@reference
            old = np.array(lookup[name]['transform_world_mm'])
            delta = new@np.linalg.inv(old)
            contract.append(dict(module=name, bank=bank, azimuth_deg=AZIMUTHS[bank], row=j,
                old_installation_transform=old, proposed_installation_transform=new,
                transform_from_current=delta, proposed_centre_mm=centre,
                output_axis_world=rotation[:, 0], old_output_axis_world=old[:3, 0],
                installation_axis_changed=True, whole_packet_reversed=reverse,
                commanded_motor_angle_deg=0., rows=[z['name'] for z in grouped[name]]))
            rows += [dict(z, shape=n.g.original.transform(z['shape'], delta),
                          proposal_module=name) for z in grouped[name]]
    assert len(rows) == 409 and len(contract) == 16
    return rows, contract


def radial_box(lo, hi, angle):
    return n.g.original.transform(prior.prior.box(lo, hi), d.around((0, 0, 0), (0, 1, 0), -angle))


def cage():
    shapes = []
    for j, y in enumerate(ROWS):
        seat_y = y-22.
        shapes.append(prior.cyl_y(38., 3., seat_y).cut(prior.cyl_y(34., 3., seat_y)))
        for bank, stem in enumerate(ORDER[j]):
            angle = AZIMUTHS[bank]
            reversed_packet = stem == 'thumb_ip'
            sy = y+19. if reversed_packet else seat_y
            for tangent in [-13., 13.]:
                pad = radial_box([34., sy, tangent-4.], [75.4, sy+3., tangent+4.], angle)
                start = np.array([71., sy-1., tangent])
                start = (d.around((0, 0, 0), (0, 1, 0), -angle)@np.r_[start, 1.])[:3]
                pad = pad.cut(cq.Solid.makeCylinder(1.7, 5., cq.Vector(*start), cq.Vector(0, 1, 0)))
                shapes.append(pad)
                if reversed_packet:
                    shapes.append(radial_box([34., seat_y, tangent-4.], [38., sy+3., tangent+4.], angle))
    # Lower rails join both hubs to actual split collars; upper caps remain removable.
    for sign in [-1., 1.]:
        shapes.append(prior.prior.box([min(sign*24., sign*27.), -232., -27.],
                                      [max(sign*24., sign*27.), -159., -24.]))
    for y in STATIONS:
        shapes.append(prior.prior.collar(y, False))
    shape = shapes[0]
    for item in shapes[1:]:
        shape = shape.fuse(item)
    rows = [dict(name='arm054_eight_bank_carrier', shape=shape.clean(), body='forearm',
                 role='separate_proposal_carrier', color=[.3, .5, .75])]
    for j, y in enumerate(STATIONS):
        rows.append(dict(name='arm054_beam_cap_'+str(j), shape=prior.prior.collar(y, True),
            body='forearm', role='separate_proposal_cap', color=[.3, .5, .75]))
        for i, x in enumerate([-31., 31.]):
            bolt = prior.prior.cylinder(1.5, 12., [x, y, -4.]).fuse(prior.prior.cylinder(2.85, 2., [x, y, -6.])).clean()
            nut = cq.Workplane('XY').polygon(6, 6.35).extrude(2.4).val().translate((x, y, 4.))
            nut = nut.cut(prior.prior.cylinder(1.5, 4., [x, y, 3.])).clean()
            for kind, part in [('M3x12_envelope', bolt), ('M3_nut_envelope', nut)]:
                rows.append(dict(name='arm054_'+kind+'_'+str(j)+'_'+str(i), shape=part,
                    body='forearm', role='nominal_fastener_envelope', color=[.55, .58, .62]))
    assert len(rows) == 11
    assert all(z['shape'].isValid() and len(z['shape'].Solids()) == 1 for z in rows), [
        (z['name'], z['shape'].isValid(), len(z['shape'].Solids())) for z in rows]
    return rows


def assemble(frames):
    _, modules, grouped, rack = prior.prior.s.bundles(frames)
    moved, contract = placement(modules, grouped)
    new = cage()
    removed = {z['name'] for z in moved} | {rack['name']}
    unchanged = {key: [z for z in rows if z['name'] not in removed] for key, rows in frames.items()}
    assert sum(map(len, unchanged.values())) == 452
    return moved, new, unchanged, contract

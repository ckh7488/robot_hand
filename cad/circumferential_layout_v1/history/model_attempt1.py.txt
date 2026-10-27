"""Separate four-bank proposal with motors pointing distally along the forearm."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'cad/arm_layout_proposal_v1'))
import proposal as prior

n, np, cq, d = prior.n, prior.np, prior.cq, prior.d
ROWS = [-268., -226., -184., -142.]
AZIMUTHS = [0., 90., 180., 270.]
RADIUS = 71.6
STATIONS = [-204.5, -162.5]


def placement(modules, grouped):
    lookup = {z['name']: z for z in modules}
    reference = np.array([-14.4, -7.5, -3.9])
    rows, contract = [], []
    for j, names in enumerate(prior.ORDER):
        for bank, stem in enumerate(names):
            name = stem+'_actuator'
            angle = np.deg2rad(AZIMUTHS[bank])
            radial = np.array([np.cos(angle), 0., np.sin(angle)])
            axial = np.array([0., 1., 0.])
            tangential = np.cross(radial, axial)
            rotation = np.column_stack([axial, tangential, radial])
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
                output_axis_world=axial, old_output_axis_world=old[:3, 0],
                installation_axis_changed=True, commanded_motor_angle_deg=0.,
                rows=[z['name'] for z in grouped[name]]))
            rows += [dict(z, shape=n.g.original.transform(z['shape'], delta),
                proposal_module=name) for z in grouped[name]]
    assert len(rows) == 409 and len(contract) == 16
    return rows, contract


def cyl_y(r, length, y):
    return cq.Solid.makeCylinder(r, length, cq.Vector(0., y, 0.), cq.Vector(0., 1., 0.))


def bridge(sign, y):
    points = [(sign*21., -25.), (sign*26., -21.),
              (sign*48., -44.), (sign*44., -48.)]
    wire = cq.Wire.makePolygon([cq.Vector(x, y-4., z) for x, z in points], close=True)
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(0., 8., 0.))


def cage():
    shapes = []
    for y in ROWS:
        # Full outer ring connects four original flange-seat-sized pads.
        # Its added material is tested, including against existing electrical solids.
        shapes.append(cyl_y(68., 3., y-22.).cut(cyl_y(64., 3., y-22.)))
        for azimuth in AZIMUTHS:
            pad = prior.box([51.2, y-22., -20.], [68., y-19., 20.])
            pad = n.g.original.transform(pad, d.around((0, 0, 0), (0, 1, 0), -azimuth))
            for tangent in [-13., 13.]:
                start = np.array([55.6, y-24., tangent])
                T = d.around((0, 0, 0), (0, 1, 0), -azimuth)
                start = (T@np.r_[start, 1.])[:3]
                bore = cq.Solid.makeCylinder(1.7, 8., cq.Vector(*start), cq.Vector(0, 1, 0))
                pad = pad.cut(bore)
            shapes.append(pad)
    for sx in [-1., 1.]:
        for sz in [-1., 1.]:
            lo = [min(sx*44., sx*48.), -290., min(sz*44., sz*48.)]
            hi = [max(sx*44., sx*48.), -161., max(sz*44., sz*48.)]
            shapes.append(prior.box(lo, hi))
    for y in STATIONS:
        shapes += [prior.collar(y, False), bridge(-1., y), bridge(1., y)]
    sh = shapes[0]
    for shape in shapes[1:]:
        sh = sh.fuse(shape)
    sh = sh.clean()
    rows = [dict(name='arm053_four_bank_cage', shape=sh, body='forearm',
        role='separate_proposal_cage', color=[.3, .5, .75])]
    for j, y in enumerate(STATIONS):
        rows.append(dict(name='arm053_beam_cap_'+str(j), shape=prior.collar(y, True),
            body='forearm', role='separate_proposal_cap', color=[.3, .5, .75]))
        for i, x in enumerate([-31., 31.]):
            bolt = prior.cylinder(1.5, 12., [x, y, -4.]).fuse(prior.cylinder(2.85, 2., [x, y, -6.])).clean()
            nut = cq.Workplane('XY').polygon(6, 6.35).extrude(2.4).val().translate((x, y, 4.))
            nut = nut.cut(prior.cylinder(1.5, 4., [x, y, 3.])).clean()
            for kind, shape in [('M3x12_envelope', bolt), ('M3_nut_envelope', nut)]:
                rows.append(dict(name='arm053_'+kind+'_'+str(j)+'_'+str(i), shape=shape,
                    body='forearm', role='nominal_fastener_envelope', color=[.55, .58, .62]))
    assert len(rows) == 11
    assert all(z['shape'].isValid() and len(z['shape'].Solids()) == 1 for z in rows), [
        (z['name'], z['shape'].isValid(), len(z['shape'].Solids())) for z in rows]
    return rows


def assemble(frames):
    _, modules, grouped, rack = prior.s.bundles(frames)
    moved, contract = placement(modules, grouped)
    new = cage()
    removed = {z['name'] for z in moved} | {rack['name']}
    unchanged = {key: [z for z in rows if z['name'] not in removed] for key, rows in frames.items()}
    assert sum(map(len, unchanged.values())) == 452
    return moved, new, unchanged, contract

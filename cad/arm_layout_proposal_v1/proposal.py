"""A separate short/wide carrier proposal, never an installed replacement."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import module_survey as s

d, n, np, ROOT, HERE = s.d, s.n, s.np, s.ROOT, s.HERE
cq = n.cq
COLS = [-55., -15., 25., 65.]
ROWS = [-274., -232., -190., -148.]
ORDER = [
    ['index_mcp', 'little_mcp', 'thumb_mcp', 'thumb_ip'],
    ['little_pip', 'little_dip', 'ring_pip', 'ring_dip'],
    ['middle_pip', 'middle_dip', 'index_pip', 'index_dip'],
    ['index_spread', 'middle_spread', 'ring_spread', 'little_spread'],
]
STATIONS = [-202., -160.]


def box(lo, hi):
    return cq.Solid.makeBox(*(np.array(hi)-lo), cq.Vector(*lo))


def cylinder(r, height, start):
    return cq.Solid.makeCylinder(r, height, cq.Vector(*start), cq.Vector(0, 0, 1))


def placements(modules, grouped):
    lookup = {m['name']: m for m in modules}
    rows, contract = [], []
    for j, names in enumerate(ORDER):
        for i, stem in enumerate(names):
            name = stem+'_actuator'
            old = np.asarray(lookup[name]['transform_world_mm'], dtype=float)
            old_center = np.asarray(lookup[name]['module_envelope_center_mm'], dtype=float)
            delta = np.eye(4)
            delta[:3, 3] = np.array([COLS[i], ROWS[j], -95.])-old_center
            new = delta@old
            contract.append(dict(module=name, row=j, column=i, old_center_mm=old_center,
                proposed_center_mm=[COLS[i], ROWS[j], -95.], transform_from_current=delta,
                old_installation_transform=old, proposed_installation_transform=new,
                installation_rotation_changed=False, motor_command_angle_changed=False,
                rows=[z['name'] for z in grouped[name]]))
            rows += [dict(z, shape=n.g.original.transform(z['shape'], delta),
                          proposal_module=name, proposal_delta=delta) for z in grouped[name]]
    assert len(rows) == 409 and len(contract) == 16
    return rows, contract


def collar(y, upper):
    z0, z1 = (0., 25.) if upper else (-25., 0.)
    sh = box([-26., y-4., z0], [26., y+4., z1])
    iz0, iz1 = (-1., 21.) if upper else (-21., 1.)
    sh = sh.cut(box([-22., y-5., iz0], [22., y+5., iz1]))
    ez0, ez1 = (0., 4.) if upper else (-4., 0.)
    for x in [-31., 31.]:
        sh = sh.fuse(box([x-5., y-4., ez0], [x+5., y+4., ez1]))
        sh = sh.cut(cylinder(1.7, 12., [x, y, -6.]))
    return sh.clean()


def carrier():
    shapes = [box([x-14.4-6., -297., -76.], [x-14.4+6., -125., -73.]) for x in COLS]
    # Original42/40 pitch and flange/rail stack are retained in this separate layout.
    for y in [-295., -253., -211., -169., -127.]+STATIONS:
        shapes.append(box([-69.4, y-4., -76.], [50.6, y+4., -73.]))
    for y in STATIONS:
        shapes.append(collar(y, False))
        for x in [-25., 25.]:
            shapes.append(box([x-3., y-4., -73.], [x+3., y+4., -25.]))
    sh = shapes[0]
    for z in shapes[1:]:
        sh = sh.fuse(z)
    for y in ROWS:
        for x in COLS:
            for offset in [-13., 13.]:
                sh = sh.cut(cylinder(1.7, 10., [x-16., y+offset, -80.]))
    rows = [dict(name='arm052_short_wide_carrier', shape=sh.clean(), body='forearm',
                 role='separate_proposal_carrier', color=[.3, .5, .75])]
    for j, y in enumerate(STATIONS):
        rows.append(dict(name='arm052_beam_cap_'+str(j), shape=collar(y, True),
            body='forearm', role='separate_proposal_cap', color=[.3, .5, .75]))
        for i, x in enumerate([-31., 31.]):
            bolt = cylinder(1.5, 12., [x, y, -4.]).fuse(cylinder(2.85, 2., [x, y, -6.])).clean()
            nut = cq.Workplane('XY').polygon(6, 6.35).extrude(2.4).val().translate((x, y, 4.))
            nut = nut.cut(cylinder(1.5, 4., [x, y, 3.])).clean()
            for kind, shape in [('M3x12_envelope', bolt), ('M3_nut_envelope', nut)]:
                rows.append(dict(name='arm052_'+kind+'_'+str(j)+'_'+str(i), shape=shape,
                    body='forearm', role='nominal_fastener_envelope', color=[.55, .58, .62]))
    assert len(rows) == 11
    assert all(z['shape'].isValid() and len(z['shape'].Solids()) == 1 for z in rows)
    return rows


def bounds(rows):
    b = np.array([n.c.m.pack._bbox(z['shape']) for z in rows])
    return [b[:, 0, :].min(axis=0).tolist(), b[:, 1, :].max(axis=0).tolist()]


def assemble(frames):
    layout, modules, grouped, old_rack = s.bundles(frames)
    moved, contract = placements(modules, grouped)
    new = carrier()
    names = {z['name'] for z in moved} | {old_rack['name']}
    unchanged = {key: [z for z in rows if z['name'] not in names] for key, rows in frames.items()}
    assert sum(map(len, unchanged.values())) == 452
    return moved, new, unchanged, contract, old_rack


if __name__ == '__main__':
    raise SystemExit('Use check_proposal.py; do not replace the current installed candidate.')

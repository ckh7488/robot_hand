"""Change the two real bend planes, keeping their radii and attachment seats."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('_arm055_layout_basis', ROOT/'cad/eight_bank_layout_v1/model.py')
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
n, np, cq, d = p.n, p.np, p.cq, p.d
b = n.m.drive.module('_arm055_clamp_basis', ROOT/'cad/support_attachment_basis_v1/bend_model.py')
OLD_NAMES = ['compact_motor_turnback_0', 'compact_motor_turnback_1']


def context():
    frames, source = d.inputs()
    raw = n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')
    assert raw['completed'] and raw['current862_installation_replaced'] is False
    _, modules, grouped, rack = p.prior.prior.s.bundles(frames)
    moved, placements = p.placement(modules, grouped)
    for got, expected in zip(placements, raw['placements']):
        assert got['module'] == expected['module'] and got['rows'] == expected['rows']
        assert np.max(abs(np.array(got['transform_from_current'])-expected['transform_from_current'])) < 1e-12
    support = n.c.load(n.p.read(ROOT/raw['scene'])['parts'], source)
    removed = {z['name'] for z in moved} | {rack['name']}
    unchanged = {key: [z for z in rows if z['name'] not in removed] for key, rows in frames.items()}
    baseline = {key: list(rows) for key, rows in unchanged.items()}
    baseline['forearm'] += moved+support
    assert sum(map(len, baseline.values())) == 872
    retained = {key: [z for z in rows if z['name'] not in OLD_NAMES] for key, rows in baseline.items()}
    assert sum(map(len, retained.values())) == 870
    T = np.array(next(z for z in placements if z['module'] == 'thumb_ip_actuator')['transform_from_current'])
    for path in ['cad/eight_bank_layout_v1/reviews/check.json',
                 'cad/eight_bank_layout_v1/model.py',
                 'cad/compact_turnback_pack_v1/reviews/check.json',
                 'cad/support_attachment_basis_v1/bend_model.py']:
        source[path] = n.m.drive.sha(ROOT/path)
    return frames, baseline, retained, T, raw, source


def build(T, source):
    rel = 'cad/sheath_retention_v1/selected/generated/parts/scene.json'
    source[rel] = n.m.drive.sha(ROOT/rel)
    names = {'motor_housing_clamp_'+str(i)+'_jaw_-1' for i in [0, 1]}
    originals = n.c.load([z for z in n.p.read(ROOT/rel)['parts'] if z['name'] in names], source)
    rows, design = [], []
    for i in [0, 1]:
        Q = T@b.frame(i)
        desired = np.array([1., 0., 1. if i == 0 else -1.])/np.sqrt(2.)
        u = Q[:3, :3].T@desired
        assert abs(u[1]) < 1e-12 and abs(np.linalg.norm(u)-1.) < 1e-12
        A = np.array([0., -12., 0.])
        B = np.array([0., -24., 0.])
        M = np.array([0., -64., 0.])+40.*u
        D = np.array([0., -24., 0.])+80.*u
        edges = [cq.Edge.makeLine(cq.Vector(*A), cq.Vector(*B)),
                 cq.Edge.makeThreePointArc(cq.Vector(*B), cq.Vector(*M), cq.Vector(*D))]
        jawname = 'motor_housing_clamp_'+str(i)+'_jaw_-1'
        jaw = n.g.original.transform(next(z['shape'] for z in originals if z['name'] == jawname), T)
        tube = b.sweep(edges, 2.5).cut(b.sweep(edges, 1.6)).clean()
        shape = jaw.fuse(n.g.original.transform(tube, Q)).clean()
        lost = jaw.cut(shape, tol=1e-6)
        removed = lost.Volume() if lost.Solids() else 0.
        assert shape.isValid() and len(shape.Solids()) == 1 and removed < 1e-8
        rows.append(dict(name='arm055_split_return_'+str(i), shape=shape, body='forearm',
            physical_line=i, replaces=OLD_NAMES[i], preserved_original_jaw=jawname,
            role='separate_proposal_integral_jaw_and_return', color=[.93, .48, .1]))
        design.append(dict(line=i, real_jaw_frame=Q, original_jaw_removed_mm3=removed,
            bend_direction_world=desired, bend_angle_deg=180., centreline_radius_mm=40.,
            guide_OD_bore_mm=[5., 3.2], straight_lead_mm=12.,
            shoulder_world_mm=(Q@np.array([0., 5., 0., 1.]))[:3],
            free_port_world_mm=(Q@np.r_[D, 1.])[:3],
            free_tangent_world=Q[:3, :3]@np.array(edges[-1].tangentAt(1).toTuple()),
            shoulder_to_free_port_length_mm=17.+sum(z.Length() for z in edges),
            fixed_attachment_and_fastener_axes_unchanged=True,
            motor_command_angle_deg=0., motor_installation_changed_from_ARM054=False,
            nominal_reaction='Housing -> bore/integral original lower jaw -> existing opposite jaw/two M2 -> captured adjuster head -> existing terminal M3 support -> ARM054 adapter/spoke/hub/collar -> actual beam/roll flange. Friction/preload/strength/assembly unverified.'))
    return rows, design

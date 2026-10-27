"""Read actual rigid motor bundles before proposing a different carrier."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'cad/arm_packaging_audit_v1'))
import audit as d

n, np = d.n, d.np


def bundles(frames):
    layout = n.p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    modules = [z for z in layout['modules'] if z['rigid_body'] == 'forearm']
    rear = [z for z in frames['forearm'] if z['source_group'] != 'arm']
    rack = [z for z in rear if z['name'] == 'Forearm_twenty_axis_module_rack']
    grouped = {m['name']: [z for z in rear if z['name'].startswith(m['name']+'_')]
               for m in modules}
    assigned = {z['name'] for rows in grouped.values() for z in rows}
    extras = [z for z in rear if z['name'] not in assigned and z not in rack]
    expected = {
        'thumb_ip_twin_sheath_terminal_frame',
        'thumb_ip_stock_output_horn', 'thumb_ip_stock_central_retainer',
        'thumb_ip_stock_rear_idler', 'thumb_ip_two_tail_spool',
        'thumb_ip_tail_clamp_1', 'thumb_ip_tail_clamp_2',
        'compact_motor_turnback_0', 'compact_motor_turnback_1',
    }
    expected |= {'thumb_ip_spool_tap6_'+str(i) for i in range(1, 5)}
    for i in range(2):
        expected |= {'thumb_ip_barrel_'+s+'_'+str(i)
                     for s in ['adjuster', 'top_nut', 'bottom_nut']}
        expected |= {'motor_housing_clamp_'+str(i)+'_jaw_1'}
        expected |= {'motor_housing_clamp_'+str(i)+'_'+s+'_'+str(j)
                     for s in ['bolt', 'nut'] for j in range(2)}
    assert {z['name'] for z in extras} == expected and len(extras) == 29
    grouped['thumb_ip_actuator'] += extras
    assert len(rack) == 1 and len(modules) == 16
    assert sorted(map(len, grouped.values())) == [24]*15+[49]
    assert sum(map(len, grouped.values())) == 409
    return layout, modules, grouped, rack[0]


def bounds(rows):
    b = np.array([n.c.m.pack._bbox(z['shape']) for z in rows])
    return [b[:, :3].min(axis=0).tolist(), b[:, 3:].max(axis=0).tolist()]


def main():
    out = HERE/'reviews/module_survey.json'
    assert not out.exists()
    frames, source = d.inputs()
    layout, modules, grouped, rack = bundles(frames)
    source['cad/arm_layout_proposal_v1/module_survey.py'] = n.m.drive.sha(Path(__file__))
    source['cad/palm_motor_layout_v1/selected/generated/layout.json'] = n.m.drive.sha(
        ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    results = []
    for module in modules:
        rows = grouped[module['name']]
        inverse = np.linalg.inv(np.asarray(module['transform_world_mm']))
        canonical = [dict(z, shape=n.g.original.transform(z['shape'], inverse)) for z in rows]
        results.append(dict(name=module['name'], old_transform=module['transform_world_mm'],
            canonical_bbox_mm=bounds(canonical), rows=[dict(name=z['name'],
            source_group=z['source_group'], bbox_mm=n.c.m.pack._bbox(c['shape']))
            for z, c in zip(rows, canonical)]))
    arm = {z['name']: n.c.m.pack._bbox(z['shape']) for rows in frames.values()
           for z in rows if z['source_group'] == 'arm'}
    n.a.save(out, dict(completed=True, source_sha256=source, bundles=results,
        rows=409, remote_modules=16, replaced_rack=rack['name'], arm_bbox_mm=arm,
        generic_thumb_IP_ownership='Both motor_housing_clamp line0/1 and both turnbacks belong to the same representative thumb_ip_actuator, not two different motors. Terminal022/RETAIN026/PACK049 and DRIVE018 scene ownership.',
        installation_geometry_or_position_changed=False, new_native_tests=0,
        input_hash_guard=all(n.m.drive.sha(ROOT/f) == h for f, h in source.items())))
    print('bundle bounds', [(z['name'], z['canonical_bbox_mm']) for z in results], flush=True)


if __name__ == '__main__':
    main()

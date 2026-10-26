"""Localize the unresolved tilted-guide/rack insertion material."""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_assembly_block_render', HERE/'render.py')
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
r, poses = a.r, a.poses


def main():
    out = HERE/'images'
    if (out/'rack_block.png').exists():
        raise FileExistsError('Preserve rack obstruction image')
    saved = r.read_json(ROOT/'cad/wire_guides_v1/generated/scene.json')
    report = r.read_json(HERE/'reviews/rack_order.json')
    if not report['completed']:
        raise ValueError('Incomplete rack study')
    name = 'middle_pip_actuator'
    record = report['paths'][name]['samples'][4]
    assert record['translation_mm'] == [0., 0., -7.]
    source = {}
    def pin(path):
        source[path.relative_to(ROOT).as_posix()] = r.sha(path)
        return path
    for path in [HERE/'render_block.py', HERE/'render.py', HERE/'reviews/rack_order.json', ROOT/'cad/wire_guides_v1/generated/scene.json',
                 ROOT/'cad/gesture_hand_v3/render_gestures.py', ROOT/'cad/gesture_hand_v3/poses.py', ROOT/'cad/functional_motion_v3/generated/catalog.json']:
        pin(path)
    evidence = HERE/'generated/visual_evidence'
    rackrow = next(row for row in saved['retained_packaging'] if row['name'] == 'Forearm_twenty_axis_module_rack')
    rack = cq.importers.importStep(str(pin(ROOT/rackrow['step']))).val()
    crop = rack.intersect(cq.Solid.makeBox(85, 56, 70, cq.Vector(-85, -176, -112)), tol=1e-6)
    cropmesh = evidence/'rack_35_local_visual_only.stl'
    cq.exporters.export(crop, str(cropmesh), tolerance=.03, angularTolerance=.1)
    pin(cropmesh)
    rows = [row for row in saved['new_parts']+saved['retained_packaging']
            if row.get('module') == name and '_rack_nut_' not in row['name']]
    common_parts = []
    common_values = []
    for hit in record['native']['overlaps']:
        if hit['part'] != rackrow['name']:
            continue
        row = next(row for row in rows if row['name'] == hit['lead'])
        moving = cq.importers.importStep(str(pin(ROOT/row['step']))).val().translate((0, 0, -7))
        common = moving.intersect(rack, tol=1e-6)
        if abs(common.Volume()-hit['material_intersection_mm3']) > 1e-5:
            raise ValueError('Wrong rack overlap')
        common_parts.append(common)
        common_values.append(dict(part=row['name'], material_intersection_mm3=common.Volume()))
    common = cq.Compound.makeCompound(common_parts)
    path = evidence/'actual_35_guide_rack_overlap.stl'
    cq.exporters.export(common, str(path), tolerance=.02, angularTolerance=.1)
    cq.exporters.export(common, str(path.with_suffix('.step')))
    pin(path); pin(path.with_suffix('.step'))
    d = r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral = poses.forward_kinematics(d, np.zeros(20))['bodies']
    assemblies = []
    for dz in [-7, 0]:
        parts = []
        for row in rows:
            mesh = pin(ROOT/row['mesh'])
            shift = np.eye(4); shift[2, 3] = dz
            parts.append(dict(name=row['name'], body='palm', mesh_path=mesh, points=r.read_stl_points(mesh),
                              color=row.get('color', [.6, .64, .67])[:3], neutral_inverse=np.linalg.inv(neutral['palm'])@shift))
        parts.append(dict(name='rack_local_visual_only', body='palm', mesh_path=cropmesh, points=r.read_stl_points(cropmesh),
                          color=[.35, .43, .52], neutral_inverse=np.linalg.inv(neutral['palm'])))
        if dz:
            parts.append(dict(name='actual_common_only', body='palm', mesh_path=path, points=r.read_stl_points(path),
                              color=[1., .08, .08], neutral_inverse=np.linalg.inv(neutral['palm'])))
        assemblies.append(dict(parts=parts, design=d))
    camera = r.view_fit(assemblies, [dict(angles_deg=np.zeros(20))], np.array([-.8, -.85, .65]), 1.)
    original = r.make_renderer
    def transparent(assembly, *args, **kwargs):
        state = original(assembly, *args, **kwargs)
        if assembly is assemblies[0]:
            for i, part in enumerate(assembly['parts']):
                if part['name'] == 'rack_local_visual_only' or part['name'].endswith(('_wire_guide_base', '_wire_guide_lid')):
                    state['model'].geom_rgba[i, 3] = .24
        return state
    r.make_renderer = transparent
    try:
        tiles = []
        for assembly, label in zip(assemblies, ['아직7mm 아래: 가이드/덮개가 랙 가로대에 걸림', '완성 위치: 겹침0, 중간 조립 경로는 미해결']):
            pixels, bounds = r.render_one(assembly, np.zeros(20), 'local', {'local': camera}, 1000, mujoco)
            tiles.append((label, pixels))
        r.sheet(tiles, 2, 1000, '남은 실제 조립 걸림 · 손 외형 수정 대상이 아님',
                '빨강=실제 겹친 체적만 표시 / 왼쪽 일부 반투명 / 랙은 그림에서만 국부 표시, 검사는 전체 랙·다른 부품 사용 / 삽입 순서·자세를 더 검토할 후보').save(out/'rack_block.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:
                assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/rel) == h for rel, h in source.items()):
        raise ValueError('Changed block render input')
    r.write_json(out/'rack_block_report.json', dict(completed=True, input_sha256=source,
                 output_sha256=r.sha(out/'rack_block.png'), actual_common_material=common_values,
                 moving_translation_mm=[0, 0, -7], visual_only_crop_mm=[[-85, -176, -112], [0, -120, -42]],
                 whole_geometry_used_for_material=True, robot_geometry_changed=False))


if __name__ == '__main__':
    main()

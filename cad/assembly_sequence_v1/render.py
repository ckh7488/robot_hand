"""Actual overlap material and a separate screw-first assembly illustration."""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT/'cad/gesture_hand_v3'))
import poses
spec = importlib.util.spec_from_file_location('_assembly_render', ROOT/'cad/gesture_hand_v3/render_gestures.py')
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


def main():
    out = HERE/'images'
    if out.exists():
        raise FileExistsError('Preserve actual CAD images')
    report = r.read_json(HERE/'reviews/base_hardware.json')
    if not report['completed']:
        raise ValueError('Incomplete original insertion study')
    out.mkdir()
    evidence = HERE/'generated/visual_evidence'
    evidence.mkdir(parents=True)
    saved = r.read_json(ROOT/'cad/wire_guides_v1/generated/scene.json')
    name = 'index_spread_actuator'
    rows = [row for row in saved['new_parts']+saved['retained_packaging'] if row.get('module') == name and '_rack_nut_' not in row['name']]
    source = {}
    def pin(path):
        source[path.relative_to(ROOT).as_posix()] = r.sha(path)
        return path
    for path in [HERE/'render.py', HERE/'reviews/base_hardware.json', ROOT/'cad/wire_guides_v1/generated/scene.json',
                 ROOT/'cad/gesture_hand_v3/render_gestures.py', ROOT/'cad/gesture_hand_v3/poses.py',
                 ROOT/'cad/functional_motion_v3/generated/catalog.json']:
        pin(path)
    d = r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral = poses.forward_kinematics(d, np.zeros(20))['bodies']
    case = next(row for row in rows if row['name'].endswith('_motor'))
    screw = next(row for row in rows if row['name'].endswith('_base_screws'))
    a = cq.importers.importStep(str(pin(ROOT/screw['step']))).val().translate((4, 0, 0))
    bx = cq.importers.importStep(str(pin(ROOT/case['step']))).val()
    common = a.intersect(bx, tol=1e-6)
    want = next(x['material_intersection_mm3'] for x in report['paths']['base_screws_after_nuts']['samples'][2]['native']['overlaps']
                if x['lead'] == screw['name'] and x['part'] == case['name'])
    if abs(common.Volume()-want) > 1e-5:
        raise ValueError('Wrong common material')
    mesh = evidence/'actual_screw_case_overlap.stl'
    cq.exporters.export(common, str(mesh), tolerance=.025, angularTolerance=.1)
    step = evidence/'actual_screw_case_overlap.step'
    cq.exporters.export(common, str(step))
    pin(mesh); pin(step)
    assemblies = []
    for mode in ['after_motor_blocked', 'screw_first']:
        parts = []
        for row in rows:
            path = pin(ROOT/row['mesh'])
            shift = np.eye(4)
            if mode == 'after_motor_blocked' and row['name'].endswith('_base_screws'):
                shift[0, 3] = 4
            if mode == 'screw_first' and row['name'].endswith(('_motor', '_rotor', '_body_screws')):
                shift[0, 3] = 20
            color = [.15, .68, .62] if row.get('kind') == 'guide_base' else row.get('color', [.65, .68, .7])[:3]
            if row['name'].endswith('_base_screws'):
                color = [.15, .45, .95]
            parts.append(dict(name=row['name'], body='palm', mesh_path=path, points=r.read_stl_points(path),
                              color=color, neutral_inverse=np.linalg.inv(neutral['palm'])@shift))
        if mode == 'after_motor_blocked':
            parts.append(dict(name='actual_overlap_only', body='palm', mesh_path=mesh, points=r.read_stl_points(mesh),
                              color=[1., .08, .08], neutral_inverse=np.linalg.inv(neutral['palm'])))
        assemblies.append(dict(parts=parts, design=d))
    camera = r.view_fit(assemblies, [dict(angles_deg=np.zeros(20))], np.array([.95, -.65, .5]), 1.)
    # Transparent vendor case is a display aid; collision used the full solid.
    original = r.make_renderer
    def transparent(assembly, *args, **kwargs):
        state = original(assembly, *args, **kwargs)
        if assembly is assemblies[0]:
            for i, part in enumerate(assembly['parts']):
                if part['name'].endswith(('_motor', '_rotor')):
                    state['model'].geom_rgba[i, 3] = .18
        return state
    r.make_renderer = transparent
    try:
        labels = ['모터를 붙인 뒤 나사 삽입: 중간에 몸체를 관통', '순서 후보: 나사 먼저 넣고 모터를 안착']
        tiles = []
        for assembly, label in zip(assemblies, labels):
            pixels, bounds = r.render_one(assembly, np.zeros(20), 'local', {'local': camera}, 1000, mujoco)
            tiles.append((label, pixels))
        r.sheet(tiles, 2, 1000, '형상을 깎기 전에 조립 순서 확인',
                '빨강은 실제 나사/모터 겹침만 표시 / 왼쪽 몸체는 내부를 보여주기 위해 반투명 / 오른쪽 모터 이동20mm는 조립 설명, 운전 자세가 아님').save(out/'assembly_order.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:
                assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/path) == h for path, h in source.items()):
        raise ValueError('Changed render inputs')
    r.write_json(out/'report.json', dict(completed=True, input_sha256=source,
                 outputs={'assembly_order.png': r.sha(out/'assembly_order.png')}, actual_saved_geometry=True,
                 actual_overlap_volume_mm3=common.Volume(), simulation_steps=0,
                 geometry_changed=False, explanatory_motor_translation_mm=[20, 0, 0]))


if __name__ == '__main__':
    main()

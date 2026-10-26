"""Actual saved CAD and sphere, with common-scale approach/release views."""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
import mujoco
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT/'cad/gesture_hand_v3'))
import poses
spec = importlib.util.spec_from_file_location('_pinch_render', ROOT/'cad/gesture_hand_v3/render_gestures.py')
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


def main():
    output = HERE/'images'
    if output.exists():
        raise FileExistsError('Preserve prior images')
    report = r.read_json(HERE/'reviews/audit.json')
    search = r.read_json(HERE/'generated/search.json')
    if not report['completed'] or not report['input_hash_guard']:
        raise ValueError('Incomplete native audit')
    scene = r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    d = search['design']
    sources = {p:r.sha(ROOT/p) for p in ['cad/object_pinch_v2/render.py',
        'cad/object_pinch_v2/reviews/audit.json', 'cad/object_pinch_v2/generated/search.json',
        'cad/gesture_hand_v3/render_gestures.py', 'cad/gesture_hand_v3/poses.py']}
    selected = report['selected'][0] if report['selected'] else None
    case = next(c for c in report['cases'] if c['id'] == selected['case']) if selected else report['cases'][0]
    path = next(p for p in report['paths'] if p['id'] == selected['path']) if selected else next(iter(report['paths']), None)
    neutral = poses.forward_kinematics(d, np.zeros(20))['bodies']
    parts = []
    for row in scene['hand']:
        if r.sha(ROOT/row['mesh']) != row['sha256_stl']:
            raise ValueError('Changed hand mesh')
        sources[row['mesh']] = row['sha256_stl']
        parts.append(dict(name=row['name'], body=row['body'], mesh_path=ROOT/row['mesh'],
            points=r.read_stl_points(ROOT/row['mesh']), color=row['color'][:3],
            neutral_inverse=np.linalg.inv(neutral[row['body']])))
    sources[case['object_step']] = case['object_sha256']
    obj = cq.importers.importStep(str(ROOT/case['object_step'])).val()
    mesh = (ROOT/case['object_step']).with_suffix('.stl')
    cq.exporters.export(obj, str(mesh), tolerance=.025, angularTolerance=.1)
    sources[mesh.relative_to(ROOT).as_posix()] = r.sha(mesh)
    parts.append(dict(name='supported_sphere25', body='palm', mesh_path=mesh,
        points=r.read_stl_points(mesh), color=[.96, .57, .13], neutral_inverse=np.linalg.inv(neutral['palm'])))
    assembly = dict(parts=parts, design=d)
    samples = path['forward_observations'] if path else []
    qs = [s['native']['hand']['angles_deg'] for s in samples] or [np.zeros(20).tolist(), case['angles_deg']]
    allposes = [dict(angles_deg=q) for q in qs]
    cameras = {'oblique':r.view_fit([assembly], allposes, np.array([.72, -.4, 1.]), 1.)}
    output.mkdir()
    records = []
    try:
        labels = ['접근 시작', '두 손가락 함께 접근', '25mm 구 양쪽 손끝 도달']
        indices = [0, len(qs)//2, len(qs)-1]
        tiles = []
        for label, index in zip(labels, indices):
            img, bounds = r.render_one(assembly, qs[index], 'oblique', cameras, 680, mujoco)
            tiles.append((label, img))
            records.append(dict(index=index, bounds_px=bounds))
        footer = '주황: 고정 지지된 시험 구 / 같은 손364부품 / 접촉 기하 검사이며 들어 올리기·힘 검증은 아님'
        r.sheet(tiles, 3, 680, 'CAD 수정 없이 · 검지와 엄지 자세를 함께 조정', footer).save(output/'approach.png')
        assembly.pop('_render_state')['renderer'].close()
        if path and path['status'] == 'sampled_approach_release_clear':
            seq = qs + list(reversed(qs[:-1]))
            frames = []
            for index in range(0, len(seq), 2):
                img, bounds = r.render_one(assembly, seq[index], 'oblique', cameras, 640, mujoco)
                panel = Image.new('RGB', (640, 720), r.BG)
                panel.paste(img, (0, 40))
                draw = ImageDraw.Draw(panel)
                draw.text((12, 7), '25mm 구 접근 → 손끝 도달 → 풀기', font=r.font(22), fill=r.INK)
                draw.text((12, 683), '고정 지지된 구 · 재생 시간은 실물 속도가 아님', font=r.font(17), fill=r.INK)
                frames.append(panel)
            frames[0].save(output/'approach_release.gif', save_all=True, append_images=frames[1:], duration=140, loop=0)
            assembly.pop('_render_state')['renderer'].close()
    finally:
        if '_render_state' in assembly:
            assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/p) == h for p,h in sources.items()):
        raise ValueError('Changed render input')
    r.write_json(output/'report.json', dict(completed=True, input_sha256=sources,
        outputs={p.name:r.sha(p) for p in output.iterdir() if p.is_file()},
        records=records, actual_saved_meshes=True, hand_parts_visible=364,
        object_parts_visible=1, simulation_steps=0, mujoco_version=mujoco.__version__))


if __name__ == '__main__':
    main()

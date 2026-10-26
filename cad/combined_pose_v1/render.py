"""Render unchanged saved hand and the additional combined example."""
import importlib.util
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
spec = importlib.util.spec_from_file_location('_combined_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


def main():
    out = HERE/'images'
    if out.exists():
        raise FileExistsError('Preserve earlier images')
    result = r.read_json(HERE/'reviews/audit.json')
    if not result['completed'] or not result['input_hash_guard']:
        raise ValueError('Incomplete native results')
    scene = r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    d = r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    inputs = {p:r.sha(ROOT/p) for p in ['cad/combined_pose_v1/render.py','cad/combined_pose_v1/reviews/audit.json',
        'cad/motion_clearance_v2/generated/assembly_catalog.json','cad/functional_motion_v3/generated/catalog.json',
        'cad/gesture_hand_v3/render_gestures.py','cad/gesture_hand_v3/poses.py']}
    neutral = poses.forward_kinematics(d,np.zeros(20))['bodies']
    parts = []
    for row in scene['hand']:
        if r.sha(ROOT/row['mesh']) != row['sha256_stl']:
            raise ValueError('Hand mesh changed')
        inputs[row['mesh']] = row['sha256_stl']
        parts.append(dict(name=row['name'],body=row['body'],mesh_path=ROOT/row['mesh'],
            points=r.read_stl_points(ROOT/row['mesh']),color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral[row['body']])))
    obj = Path(result['object']['step']).with_suffix('.stl')
    inputs[obj.as_posix()] = r.sha(ROOT/obj)
    parts.append(dict(name='sphere25',body='palm',mesh_path=ROOT/obj,points=r.read_stl_points(ROOT/obj),
        color=[.96,.57,.13],neutral_inverse=np.linalg.inv(neutral['palm'])))
    assembly = dict(parts=parts,design=d)
    case = result['cases'][0]
    path = next((p for p in result['paths'] if p['status']=='sampled_approach_release_clear'),None)
    qs = [s['native']['hand']['angles_deg'] for s in path['forward_observations']] if path else [np.zeros(20).tolist(),case['angles_deg']]
    cameras = {'oblique':r.view_fit([assembly],[dict(angles_deg=q) for q in qs],np.array([.72,-.4,1.]),1.)}
    out.mkdir()
    try:
        tiles = []
        for label,q in [('편 손',qs[0]),('중지 펴기 · 약지/새끼 접기 · 엄지/검지 접근',qs[-1])]:
            image,bounds = r.render_one(assembly,q,'oblique',cameras,760,mujoco)
            tiles.append((label,image))
        r.sheet(tiles,2,760,'추가 손모양 한 가지 · 손가락별로 서로 다른 자세',
            '같은 손 CAD364부품 / 주황 구는 고정 지지된 시험 물체 / 실제 파지력·들어 올리기 검증 아님').save(out/'example.png')
        assembly.pop('_render_state')['renderer'].close()
        if path:
            seq = qs+list(reversed(qs[:-1]))
            frames = []
            for i in sorted(set(range(0,len(seq),2))|{len(qs)-1,len(seq)-1}):
                image,_ = r.render_one(assembly,seq[i],'oblique',cameras,640,mujoco)
                panel = Image.new('RGB',(640,720),r.BG)
                panel.paste(image,(0,40))
                draw = ImageDraw.Draw(panel)
                draw.text((12,7),'중지 펴기 + 두 손가락 접기 + 손끝 접근',font=r.font(22),fill=r.INK)
                draw.text((12,683),'같은 CAD · 고정 구 · 재생 시간은 실물 속도가 아님',font=r.font(16),fill=r.INK)
                frames.append(panel)
            frames[0].save(out/'example.gif',save_all=True,append_images=frames[1:],duration=140,loop=0)
            assembly.pop('_render_state')['renderer'].close()
    finally:
        if '_render_state' in assembly:
            assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/p)==h for p,h in inputs.items()):
        raise ValueError('Render inputs changed')
    r.write_json(out/'report.json',dict(completed=True,input_sha256=inputs,
        outputs={p.name:r.sha(p) for p in out.iterdir() if p.is_file()},hand_parts_visible=364,
        object_parts_visible=1,simulation_steps=0,actual_saved_meshes=True))


if __name__=='__main__':
    main()

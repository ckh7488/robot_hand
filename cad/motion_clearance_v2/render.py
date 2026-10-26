"""Render the actual saved palm and index shell, before and after local relief."""
import argparse, copy, hashlib, importlib.util, json, sys
from pathlib import Path
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
    spec=importlib.util.spec_from_file_location('_motion002_renderer',ROOT/'cad/gesture_hand_v3/render_gestures.py')
    r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
    design=read(HERE/'generated/design.json')
    neutral=r.poses.forward_kinematics(design,np.zeros(20))['bodies']
    q=np.zeros(20);q[1]=-20
    original=read(ROOT/'cad/thumb_ip_drive_v1/generated/scene.json')
    candidate=read(HERE/'generated/scene.json')
    inputs={};assemblies=[]
    names={'Palm_tapered_chassis','index_segment1_segment_shell'}
    for idx,scene in enumerate([original,candidate]):
        parts=[]
        for row in scene['hand']:
            if row['name'] not in names:continue
            path=ROOT/row['mesh'] if row['location']=='repo' else ROOT/'cad/thumb_ip_drive_v1/generated'/row['mesh']
            if sha(path)!=row['sha256_stl']:raise ValueError('Changed mesh')
            inputs[path.relative_to(ROOT).as_posix()]=sha(path)
            color=([.80,.23,.20] if idx==0 else [.38,.54,.59]) if row['name']=='Palm_tapered_chassis' else [.19,.43,.77]
            parts.append(dict(name=row['name'],body=row['body'],mesh_path=path,points=r.read_stl_points(path),
                color=color,neutral_inverse=np.linalg.inv(neutral[row['body']])))
        assemblies.append(dict(parts=parts,design=design))
    camera={'detail':r.view_fit(assemblies,[dict(angles_deg=q)],np.array([.8,-.4,1.]),1)}
    tiles=[];bounds=[]
    for idx,assembly in enumerate(assemblies):
        im,bb=r.render_one(assembly,q,'detail',camera,700,mujoco)
        assembly['_render_state']['renderer'].close();bounds.append(bb)
        tiles.append(('기존 · 안쪽 턱과 겹침' if idx==0 else '수정 · 안쪽 턱을 낮춰 이동 공간 확보',im))
    a.output.mkdir(parents=True)
    r.sheet(tiles,2,700,'MOTION002 · 검지 뿌리를 뒤로 20° 젖힌 실제 CAD',
        '두 부품만 표시 · 같은 자세/카메라/축척 · 뿌리 받침의 다른 간섭과 강도·공차는 아직 미해결').save(a.output/'palm_relief_comparison.png')
    (a.output/'render_report.json').write_text(json.dumps(dict(source_sha256=inputs,angles_deg=q.tolist(),
        shown_parts=sorted(names),other_parts_hidden=True,synthetic_geometry=False,
        cameras=camera,foreground_bounds=bounds,mujoco=mujoco.__version__,simulation_steps=0),indent=2),encoding='utf-8')


if __name__=='__main__':main()

"""Actual unchanged364-part CAD, posed for functional visual inspection."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
spec=importlib.util.spec_from_file_location('_function_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError('Preserve previous visual evidence')
    cat=read(HERE/'generated/catalog.json');scene=read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    inputs={p:sha(ROOT/p) for p in ['cad/functional_motion_v3/render.py','cad/functional_motion_v3/generated/catalog.json','cad/gesture_hand_v3/render_gestures.py','cad/gesture_hand_v3/poses.py']}
    neutral=poses.forward_kinematics(cat['design'],np.zeros(20))['bodies'];parts=[]
    for row in scene['hand']:
        if sha(ROOT/row['mesh'])!=row['sha256_stl']:raise ValueError('Changed mesh')
        inputs[row['mesh']]=row['sha256_stl']
        parts.append(dict(name=row['name'],body=row['body'],mesh_path=ROOT/row['mesh'],points=r.read_stl_points(ROOT/row['mesh']),
            color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral[row['body']])))
    assembly=dict(parts=parts,design=cat['design']);cases={c['id']:c for c in cat['cases']}
    groups=[('cross', ['cross_old_reference','cross_middle_14','cross_index_14'], '실제 CAD · 교차 후보 비교, 가운데·오른쪽은 간섭 검사 전 후보'),
        ('contacts',['pinch_index','pinch_middle','pinch_ring','pinch_little'], '실제 CAD · 손끝 접촉 탐색, 접촉·물체 유지 성공 판정 전'),
        ('functions',['four_hook','four_tabletop','four_flat_fist','fan_half','OK','finger_heart','L','shaka'], '실제 CAD · 추가 기능 표본, 이름만으로 기능 성공 판정하지 않음')]
    a.output.mkdir(parents=True);records=[];outputs=[]
    try:
        for group,ids,title in groups:
            selected=[cases[i] for i in ids]
            # One shared scale in each comparison; all364 installed parts shown.
            cameras={v:r.view_fit([assembly],selected,n,1.) for v,n in [('front',np.array([0.,0.,1.])),('side',np.array([.9,-.1,.65]))]}
            if '_render_state' in assembly:
                assembly.pop('_render_state')['renderer'].close()
            for view in cameras:
                tiles=[]
                for c in selected:
                    img,bounds=r.render_one(assembly,c['angles_deg'],view,cameras,600,mujoco)
                    tiles.append((c['label_ko'],img));records.append(dict(id=c['id'],view=view,foreground_bounds_px=bounds))
                filename=group+'_'+view+'.png'
                r.sheet(tiles,3 if group=='cross' else 2 if group=='contacts' else 4,600,title,
                    '현재364부품·같은 비교 축척 / 관절만 이동, 형상 변경·숨긴 부품 없음 / 실물 성능 미검증').save(a.output/filename)
                outputs.append(filename)
    finally:
        if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    if not all(sha(ROOT/p)==h for p,h in inputs.items()):raise ValueError('Render inputs changed')
    (a.output/'report.json').write_text(json.dumps(dict(completed=True,input_sha256=inputs,part_count=364,
        no_parts_hidden=True,simulation_steps=0,mujoco_version=mujoco.__version__,records=records,
        output_sha256={p:sha(a.output/p) for p in outputs}),indent=2),encoding='utf-8')
    print(json.dumps(dict(completed=True,outputs=outputs)),flush=True)


if __name__=='__main__':main()

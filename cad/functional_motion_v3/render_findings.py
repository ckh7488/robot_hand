"""Render actual whole-hand failures and one tested pose correction."""
import argparse
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
spec=importlib.util.spec_from_file_location('_findings_renderer',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError('Preserve earlier renders')
    cat=r.read_json(HERE/'generated/catalog.json');audit=r.read_json(HERE/'reviews/audit.json');fix=r.read_json(HERE/'reviews/motion_fix.json')
    scene=r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    inputs={p:r.sha(ROOT/p) for p in ['cad/functional_motion_v3/render_findings.py','cad/functional_motion_v3/generated/catalog.json',
        'cad/functional_motion_v3/reviews/audit.json','cad/functional_motion_v3/reviews/motion_fix.json',
        'cad/gesture_hand_v3/render_gestures.py','cad/gesture_hand_v3/poses.py']}
    neutral=poses.forward_kinematics(cat['design'],np.zeros(20))['bodies'];parts=[]
    for row in scene['hand']:
        if r.sha(ROOT/row['mesh'])!=row['sha256_stl']:raise ValueError('Changed mesh')
        inputs[row['mesh']]=row['sha256_stl']
        parts.append(dict(name=row['name'],body=row['body'],mesh_path=ROOT/row['mesh'],points=r.read_stl_points(ROOT/row['mesh']),color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral[row['body']])))
    lookup={c['id']:c for c in cat['cases']};result={c['id']:c for c in audit['cases']}
    a.output.mkdir(parents=True);outputs=[];records=[]
    try:
        defects=[]
        for ident,label in [('cross_middle_18','가까운 교차: 첫마디끼리 실제 겹침'),('pinch_index','손끝만 맞춘 집기: 엄지가 첫마디 관통')]:
            q=lookup[ident]['angles_deg'];bad={n for row in result[ident]['native']['collision_details'] for n in [row['a'],row['b']]}
            assembly=dict(design=cat['design'],parts=[dict(p,color=[.9,.16,.12] if p['name'] in bad else p['color']) for p in parts])
            camera={'oblique':r.view_fit([assembly],[dict(angles_deg=q)],np.array([.8,-.4,1.]),1.)}
            img,bounds=r.render_one(assembly,q,'oblique',camera,720,mujoco);assembly['_render_state']['renderer'].close()
            defects.append((label,img));records.append(dict(id=ident,highlighted_parts=sorted(bad),bounds_px=bounds))
        r.sheet(defects,2,720,'전체 기능에서 드러난 문제 · 빨강은 실제 재료 겹침이 나온 부품',
            '실제364부품 모두 표시 / 선택한 후보의 실패이며, 다른 자세로도 불가능하다는 증명은 아님').save(a.output/'functional_failures.png')
        outputs.append('functional_failures.png')
        poses_to_render=[p['native']['angles_deg'] for p in fix['cases']];assembly=dict(design=cat['design'],parts=parts)
        cameras={'front':r.view_fit([assembly],[dict(angles_deg=q) for q in poses_to_render],np.array([.5,-.3,1.]),1.)}
        tiles=[]
        for p in fix['cases']:
            img,bounds=r.render_one(assembly,p['native']['angles_deg'],'front',cameras,720,mujoco)
            label=('수정 전' if p['id'].endswith('original') else '엄지 자세 수정 후')+' · 겹침 '+str(p['native']['collision_count'])+'쌍'
            tiles.append((label,img));records.append(dict(id=p['id'],bounds_px=bounds))
        assembly['_render_state']['renderer'].close()
        title='검지만 펴는 동작 · 네 손가락 목표와 CAD 형상 유지'
        footer='완성 자세와 접근·풀기'+str(fix['summary']['sampled_path_configurations'])+'표본 검사 / 연속·실물 성능 승인은 아님'
        r.sheet(tiles,2,720,title,footer).save(a.output/'count_one_correction.png');outputs.append('count_one_correction.png')
        observations=fix['paths'][0]['observations'];indices=sorted(set(range(0,len(observations),3))|{len(observations)-1})
        assembly=dict(design=cat['design'],parts=parts)
        cameras={'oblique':r.view_fit([assembly],[dict(angles_deg=p['native']['angles_deg']) for p in observations],np.array([.5,-.3,1.]),1.)}
        frames=[]
        from PIL import Image,ImageDraw
        for ix in indices:
            pose=observations[ix];img,bounds=r.render_one(assembly,pose['native']['angles_deg'],'oblique',cameras,600,mujoco)
            panel=Image.new('RGB',(600,680),r.BG);panel.paste(img,(0,45));draw=ImageDraw.Draw(panel)
            draw.text((15,8),'검지만 펴기 · 접근/풀기 · 실제 CAD',font=r.font(23),fill=r.INK)
            draw.text((15,650),'재생 시간은 실물 속도가 아님 · 표본 경로',font=r.font(19),fill=r.INK)
            frames.append(panel)
        assembly['_render_state']['renderer'].close()
        frames[0].save(a.output/'count_one_motion.gif',save_all=True,append_images=frames[1:],duration=140,loop=0)
        outputs.append('count_one_motion.gif')
    finally:
        pass
    if not all(r.sha(ROOT/p)==h for p,h in inputs.items()):raise ValueError('Render input changed')
    r.write_json(a.output/'report.json',dict(completed=True,input_sha256=inputs,outputs={p:r.sha(a.output/p) for p in outputs},
        records=records,all364parts_visible=True,simulation_steps=0,actual_saved_triangles=True))


if __name__=='__main__':main()

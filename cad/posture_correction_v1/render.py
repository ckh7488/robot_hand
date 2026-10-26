"""Same saved CAD before/after posture correction, with geometric path GIFs."""
import argparse
import importlib.util
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
spec=importlib.util.spec_from_file_location('_posture_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError('Preserve earlier renders')
    report=r.read_json(HERE/'reviews/audit.json');search=r.read_json(HERE/'generated/search.json')
    scene=r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');d=search['design']
    inputs={p:r.sha(ROOT/p) for p in ['cad/posture_correction_v1/render.py','cad/posture_correction_v1/generated/search.json',
        'cad/posture_correction_v1/reviews/audit.json','cad/gesture_hand_v3/render_gestures.py','cad/gesture_hand_v3/poses.py']}
    neutral=poses.forward_kinematics(d,np.zeros(20))['bodies'];parts=[]
    for row in scene['hand']:
        if r.sha(ROOT/row['mesh'])!=row['sha256_stl']:raise ValueError('Changed mesh')
        inputs[row['mesh']]=row['sha256_stl'];parts.append(dict(name=row['name'],body=row['body'],mesh_path=ROOT/row['mesh'],
            points=r.read_stl_points(ROOT/row['mesh']),color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral[row['body']])))
    byid={c['id']:c for c in report['cases']};selected={s['group']:s for s in report['selected']};assembly=dict(parts=parts,design=d)
    pairs=[]
    for before,group,label in [('before_cross_middle_18','cross_front_middle','교차'),('before_pinch_index','index_thumb_distal_contact','손끝 모으기')]:
        choices=[c for c in report['cases'] if c['group']==group]
        after=byid[selected[group]['case']] if group in selected else next((c for c in choices if not c['native']['collision_count']),choices[0])
        pairs.append((byid[before],after,label))
    poses_to_fit=[dict(angles_deg=c['angles_deg']) for p in pairs for c in p[:2]]
    cameras={'oblique':r.view_fit([assembly],poses_to_fit,np.array([.72,-.4,1.]),1.)}
    a.output.mkdir(parents=True);records=[];outputs=[]
    try:
        tiles=[]
        for before,after,label in pairs:
            for c,when in [(before,'교정 전'),(after,'각도 교정 후')]:
                img,bounds=r.render_one(assembly,c['angles_deg'],'oblique',cameras,720,mujoco)
                tiles.append((label+' '+when+' · 겹침'+str(c['native']['collision_count'])+'쌍',img))
                records.append(dict(id=c['id'],bounds_px=bounds,part_count=364))
        r.sheet(tiles,2,720,'같은 CAD · 부품/축/길이 유지 · 자세와 움직이는 순서만 교정',
            '좌: 이전 실패 후보 / 우: 교정 후보 / 손끝은 빈손 근접 자세, 물체 집기·실물 성능 승인은 아님').save(a.output/'posture_comparison.png')
        outputs.append('posture_comparison.png')
        assembly.pop('_render_state')['renderer'].close()
        for group,filename,title in [('cross_front_middle','cross_motion.gif','가위 → 교차 → 가위'),('index_thumb_distal_contact','tip_motion.gif','보 → 손끝 모으기 → 보')]:
            if group not in selected:continue
            path=next(p for p in report['paths'] if p.get('id')==selected[group]['path']);forward=path['forward_observations']
            seq=forward+list(reversed(forward[:-1]));cameras={'oblique':r.view_fit([assembly],[dict(angles_deg=p['native']['angles_deg']) for p in seq],np.array([.72,-.4,1.]),1.)}
            indices=sorted(set(range(0,len(seq),2))|{len(seq)-1});frames=[]
            for i in indices:
                img,bounds=r.render_one(assembly,seq[i]['native']['angles_deg'],'oblique',cameras,620,mujoco)
                panel=Image.new('RGB',(620,700),r.BG);panel.paste(img,(0,40));draw=ImageDraw.Draw(panel)
                draw.text((12,7),title+' · 같은 CAD',font=r.font(23),fill=r.INK)
                draw.text((12,667),'검사한 각도 경로 / 재생 시간은 실물 속도가 아님',font=r.font(17),fill=r.INK);frames.append(panel)
            frames[0].save(a.output/filename,save_all=True,append_images=frames[1:],duration=140,loop=0);outputs.append(filename)
            assembly.pop('_render_state')['renderer'].close()
    finally:
        if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/p)==h for p,h in inputs.items()):raise ValueError('Render inputs changed')
    r.write_json(a.output/'report.json',dict(completed=True,input_sha256=inputs,outputs={p:r.sha(a.output/p) for p in outputs},records=records,
        actual_saved_meshes=True,no_parts_hidden=True,geometry_changed=False,simulation_steps=0,mujoco_version=mujoco.__version__))


if __name__=='__main__':main()

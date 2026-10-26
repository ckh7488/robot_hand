"""Clarify saved collision locations without recoloring entire hand parts."""
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
spec=importlib.util.spec_from_file_location('_explain_renderer',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError('Preserve earlier picture')
    cat=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json');results=r.read_json(ROOT/'cad/functional_motion_v3/reviews/audit.json')
    scene=r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');design=cat['design']
    neutral=poses.forward_kinematics(design,np.zeros(20))['bodies'];parts=[]
    sources={p:r.sha(ROOT/p) for p in ['cad/object_contact_v1/explain_red.py','cad/functional_motion_v3/generated/catalog.json','cad/functional_motion_v3/reviews/audit.json','cad/gesture_hand_v3/render_gestures.py','cad/gesture_hand_v3/poses.py']}
    for row in scene['hand']:
        if r.sha(ROOT/row['mesh'])!=row['sha256_stl']:raise ValueError('Changed saved mesh')
        sources[row['mesh']]=row['sha256_stl']
        parts.append(dict(name=row['name'],body=row['body'],mesh_path=ROOT/row['mesh'],points=r.read_stl_points(ROOT/row['mesh']),
            color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral[row['body']])))
    selected=[next(c for c in cat['cases'] if c['id']==ident) for ident in ['cross_middle_18','pinch_index']]
    assembly=dict(parts=parts,design=design);cameras={'oblique':r.view_fit([assembly],selected,np.array([.8,-.4,1.]),1.)}
    tiles=[];marks=[]
    try:
        for case,label in zip(selected,['교차 후보: 두 첫마디가 겹치는 위치','집기 후보: 엄지가 검지에 겹치는 위치']):
            result=next(c for c in results['cases'] if c['id']==case['id'])
            collision=max(result['native']['collision_details'],key=lambda c:c['material_intersection_mm3'])
            image,bounds=r.render_one(assembly,case['angles_deg'],'oblique',cameras,720,mujoco)
            cam=cameras['oblique'];point=np.array(collision['intersection_center_mm']);delta=point-cam['focus']
            factor=720/(2*cam['parallel_scale_mm']);x=360+float(delta@cam['right'])*factor;y=360-float(delta@cam['up'])*factor
            draw=ImageDraw.Draw(image);draw.ellipse((x-19,y-19,x+19,y+19),outline=(235,35,35),width=5)
            # Circle is a location marker, not a scale drawing of overlap size.
            anchor=(x-125,y-70);draw.line((anchor[0]+15,anchor[1]+25,x-14,y-14),fill=(220,30,30),width=4)
            draw.rounded_rectangle((anchor[0]-15,anchor[1]-5,anchor[0]+92,anchor[1]+28),radius=5,fill='white')
            draw.text(anchor,'걸리는 위치',font=r.font(19),fill=(185,25,25))
            tiles.append((label,image));marks.append(dict(case=case['id'],world_point_mm=point.tolist(),pixel=[x,y],source_pair=[collision['a'],collision['b']]))
    finally:
        if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    a.output.mkdir(parents=True)
    r.sheet(tiles,2,720,'빨간 원 = 실제 겹침이 계산된 위치 · 부품 색은 원래대로',
        '이 후보 자세에서는 걸림 / 자연스러운 자세·이동 순서로 먼저 해결 / 빨간 원의 크기는 겹친 체적 크기가 아님').save(a.output/'collision_locations.png')
    if not all(r.sha(ROOT/p)==h for p,h in sources.items()):raise ValueError('Inputs changed')
    r.write_json(a.output/'report.json',dict(completed=True,source_sha256=sources,markers=marks,all364parts_visible=True,
        simulation_steps=0,output_sha256=r.sha(a.output/'collision_locations.png'),interpretation='Projected material-intersection centroid markers, not cutout targets or overlap-volume outlines'))


if __name__=='__main__':main()

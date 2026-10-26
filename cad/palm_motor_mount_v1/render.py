"""Show the missing retention path and the proposed real CAD split collars."""
import importlib.util
from pathlib import Path
import sys

import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
spec=importlib.util.spec_from_file_location('_mount_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)


def main():
    out=HERE/'images';out.mkdir(exist_ok=False)
    source_scene=r.read_json(ROOT/'cad/palm_motor_layout_v1/selected/generated/scene.json')
    saved=r.read_json(HERE/'selected3/generated/scene.json')
    report=r.read_json(HERE/'selected3/reviews/check.json')
    assert report['completed'] and report['input_hash_guard']
    hand=r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand']
    design=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral=poses.forward_kinematics(design,np.zeros(20))['bodies']
    sources={z:r.sha(ROOT/z) for z in ['cad/palm_motor_mount_v1/render.py','cad/palm_motor_mount_v1/selected3/generated/scene.json','cad/palm_motor_mount_v1/selected3/reviews/check.json']}
    records=[source_scene['changed_parts']+source_scene['retained_packaging_guide'],saved['new_parts']+saved['retained_parts']]
    assemblies=[]
    for k,rows in enumerate(records):
        parts=[]
        def add(row,color,alpha):
            path=ROOT/row['mesh'];assert r.sha(path)==row['sha256_stl'];sources[row['mesh']]=row['sha256_stl']
            parts.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,alpha=alpha,
                              neutral_inverse=np.linalg.inv(neutral['palm'])))
        for row in hand:
            if row['body']=='palm' and row['name']!='Palm_shaped_front_cover':add(row,[.62,.69,.75],.2)
        for row in rows:
            if row.get('body')!='palm':continue
            if row['name']=='Palm_four_motor_carrier':color,alpha=[.95,.5,.1],.95
            elif row.get('kind')=='removable_collar_cap':color,alpha=[.12,.64,.4],1.
            elif row['name'].startswith('Palm_collar_'):color,alpha=[.28,.32,.37],1.
            else:color,alpha=[.13,.42,.7],.4
            add(row,color,alpha)
        assemblies.append(dict(parts=parts,design=design))
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([.15,-.35,1.]),1.)
    images=[]
    try:
        for assembly in assemblies:
            state=r.make_renderer(assembly,{'mount':camera},1100,mujoco);assembly['_render_state']=state
            for i,row in enumerate(assembly['parts']):state['model'].geom_rgba[i,3]=row['alpha']
            image,_=r.render_one(assembly,np.zeros(20),'mount',{'mount':camera},1100,mujoco);images.append(image)
        r.sheet([('이전: 네 받침이 바닥에만 닿음 · 기둥과 고정 연결 없음',images[0]),
                 ('후보: 주황 받침 + 초록 분리 덮개로 기존 기둥 두 곳을 죔',images[1])],2,1100,
            '손·모터 배치 유지 · 실제 고정 경로를 추가하는 후보',
            '덮개를 숨긴 고정 구조 보기 · 볼트는 명목 외곽 · 마찰/체결력/강도·실물 설치 승인은 아님').save(out/'mount.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    assert all(r.sha(ROOT/z)==h for z,h in sources.items())
    r.write_json(out/'report.json',dict(completed=True,input_sha256=sources,outputs={'mount.png':r.sha(out/'mount.png')},
        actual_saved_meshes=True,simulation_steps=0,view='Fixed palm cutaway;hand audit still includes all364parts.'))


if __name__=='__main__':main()

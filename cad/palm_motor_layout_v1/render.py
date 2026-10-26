"""Native saved meshes with actual common volumes, not whole-part red paint."""
import importlib.util
from pathlib import Path
import sys

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
spec=importlib.util.spec_from_file_location('_palm_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)


def main():
    output=HERE/'images/placement.png'
    if output.exists():raise FileExistsError('Keep the original picture')
    saved=r.read_json(HERE/'selected/generated/scene.json')
    study=r.read_json(HERE/'reviews/check2.json')
    assert study['completed'] and study['input_hash_guard']
    scene=r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    d=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral=poses.forward_kinematics(d,np.zeros(20))['bodies']
    old=r.read_json(ROOT/'cad/wiring_layout_v1/generated/packaging.json')['parts']
    after=saved['changed_parts']+saved['retained_packaging_guide']
    evidence=r.read_json(ROOT/'cad/wiring_layout_v1/reviews/overlap_locations.json')['items']
    sources={v:r.sha(ROOT/v) for v in ['cad/palm_motor_layout_v1/render.py',
        'cad/palm_motor_layout_v1/selected/generated/scene.json','cad/palm_motor_layout_v1/reviews/check2.json',
        'cad/wiring_layout_v1/reviews/overlap_locations.json','cad/gesture_hand_v3/render_gestures.py']}
    assemblies=[]
    for version,records in [('before',old),('after',after)]:
        parts=[]
        def add(row,color,alpha=1.):
            path=ROOT/row['mesh']
            if r.sha(path)!=row['sha256_stl']:raise ValueError('Changed render input')
            sources[row['mesh']]=row['sha256_stl']
            parts.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
                              color=color,alpha=alpha,neutral_inverse=np.linalg.inv(neutral['palm'])))
        for row in scene['hand']:
            if row['body']!='palm' or row['name']=='Palm_shaped_front_cover':continue
            add(row,[.62,.69,.75],.22 if row['name']=='Palm_tapered_chassis' else .4)
        for row in records:
            if row.get('body')=='palm':
                add(row,[.13,.42,.7],.55)
        if version=='before':
            for i,row in enumerate(evidence):
                add(dict(name='actual_common_'+str(i),mesh=row['evidence_mesh'],sha256_stl=row['sha256_stl']),[.96,.06,.04])
        assemblies.append(dict(parts=parts,design=d))
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([.15,-.25,1.]),1.)
    images=[]
    try:
        for assembly in assemblies:
            state=r.make_renderer(assembly,{'detail':camera},1000,mujoco)
            assembly['_render_state']=state
            for i,part in enumerate(assembly['parts']):state['model'].geom_rgba[i,3]=part['alpha']
            tile,_=r.render_one(assembly,np.zeros(20),'detail',{'detail':camera},1000,mujoco)
            images.append(tile)
        labels=['이전: 빨강 8곳은 실제 겹친 재료',
                '후보: 모터 위치·방향만 조정 · 겹침 0 / 바닥 접촉 확인' if study['nominal_placement_clear'] and study['changed_feet_seated'] else '후보: 추가 겹침 또는 바닥 접촉 미해결']
        r.sheet(list(zip(labels,images)),2,1000,'손 외형을 유지한 손바닥 모터 배치 · 실제 CAD 내부 보기',
                '파랑: 모터/받침 · 회색 반투명: 손바닥 고정 구조 · 덮개만 숨김 · 체결·구동 연결/설치 승인은 아님').save(output)
        r.sheet([(labels[0],images[0])],1,1000,'빨강은 실제 공통 체적 8곳',
                '부품 전체를 빨갛게 칠한 것이 아님 · 손을 깎기 전에 모터 위치/방향부터 조정').save(HERE/'images/problem.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    assert all(r.sha(ROOT/path)==h for path,h in sources.items())
    r.write_json(HERE/'images/report.json',dict(completed=True,input_sha256=sources,
        outputs={path.name:r.sha(path) for path in [output,HERE/'images/problem.png']},
        actual_native_meshes=True,actual_overlap_volumes=8,simulation_steps=0,
        visualization_only_transparency=True,hidden_cover=True,
        scope='Fixed palm detail only. No packaging or hand parts excluded from the native audit.'))


if __name__=='__main__':main()

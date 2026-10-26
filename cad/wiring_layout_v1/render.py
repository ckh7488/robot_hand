"""Actual saved hand/motor CAD; explicit cover-only cutaway for packaging."""
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
spec=importlib.util.spec_from_file_location('_wire_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


def main():
    out=HERE/'images'
    if out.exists():
        raise FileExistsError('Keep prior renders')
    initial=r.read_json(HERE/'reviews/screen.json')
    refined=r.read_json(HERE/'reviews/refined.json') if (HERE/'reviews/refined.json').exists() else None
    report=refined or initial
    if not report['completed'] or not report['input_hash_guard']:
        raise ValueError('Incomplete geometry screen')
    scene=r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    d=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral=poses.forward_kinematics(d,np.zeros(20))['bodies']
    sources={p:r.sha(ROOT/p) for p in ['cad/wiring_layout_v1/render.py','cad/wiring_layout_v1/reviews/screen.json',
        'cad/wiring_layout_v1/generated/packaging.json','cad/motion_clearance_v2/generated/assembly_catalog.json',
        'cad/form_drive_v1/generated/packaging/scene.json','cad/gesture_hand_v3/render_gestures.py','cad/gesture_hand_v3/poses.py']}
    chosen_metadata=HERE/'generated/refined_packaging.json' if refined and refined.get('selected') else HERE/'generated/packaging.json'
    if refined:
        sources['cad/wiring_layout_v1/reviews/refined.json']=r.sha(HERE/'reviews/refined.json')
        sources[chosen_metadata.relative_to(ROOT).as_posix()]=r.sha(chosen_metadata)
    assemblies=[]
    changed={'index_mcp_actuator','little_mcp_actuator'}
    if refined and refined.get('selected'):
        changed.update(['thumb_base_sweep_actuator','thumb_opposition_actuator','ring_mcp_actuator'])
    for version in ['before','after']:
        parts=[]
        for row in scene['hand']:
            sources[row['mesh']]=row['sha256_stl']
            if row['name']=='Palm_shaped_front_cover':
                continue
            parts.append(dict(name=row['name'],body=row['body'],mesh_path=ROOT/row['mesh'],points=r.read_stl_points(ROOT/row['mesh']),
                color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral[row['body']])))
        if version=='before':
            rows=r.read_json(ROOT/'cad/form_drive_v1/generated/packaging/scene.json')['parts']
            motorrows=r.read_json(ROOT/'cad/form_drive_v1/generated/packaging/placements_mass_inertia.json')['parts']
            modules={p['name']:p.get('module') for p in motorrows}
            records=[dict(row,mesh='cad/form_drive_v1/generated/packaging/'+row['mesh'],module=modules[row['name']]) for row in rows]
        else:
            records=r.read_json(chosen_metadata)['parts']
        for row in records:
            path=ROOT/row['mesh']
            if r.sha(path)!=row['sha256_stl']:
                raise ValueError('Changed saved packaging mesh')
            sources[row['mesh']]=row['sha256_stl']
            color=[.98,.57,.13] if row.get('module') in changed else row['color'][:3]
            parts.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
                color=color,neutral_inverse=np.linalg.inv(neutral['palm'])))
        assemblies.append(dict(parts=parts,design=d))
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([.55,-.15,1.]),1.)
    out.mkdir()
    try:
        tiles=[]
        after_label='후보: 두 모터를 전완으로 · 세 모터는 손바닥 안 재배치' if refined and refined.get('selected') else '후보: 손바닥4 / 원격16 · 간섭은 상세 결과 확인'
        labels=['이전: 손바닥6 / 원격14 · 현재 손과 관통',after_label]
        for assembly,label in zip(assemblies,labels):
            image,bounds=r.render_one(assembly,np.zeros(20),'oblique',{'oblique':camera},1100,mujoco)
            tiles.append((label,image))
        r.sheet(tiles,2,1100,'배선 전 모터 배치 검토 · 손364부품 형상/관절축/길이 유지',
            '주황은 위치를 조정한 모듈 전체 / 덮개1개만 숨긴 내부 보기 / 손 형상 절삭·모터 설치·줄·전선 완료 아님').save(out/'placement.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:
                assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/p)==h for p,h in sources.items()):
        raise ValueError('Render input changed')
    r.write_json(out/'report.json',dict(completed=True,input_sha256=sources,
        outputs={'placement.png':r.sha(out/'placement.png')},actual_saved_meshes=True,simulation_steps=0,
        hand_visual_parts=363,hidden_for_cutaway=['Palm_shaped_front_cover'],
        audit_hand_parts=364,hidden_from_audit=[],packaging_parts=len(records),motor_count=20,
        interpretation='Provisional motor packaging at fixed hand/forearm reference, no installed motor or wiring claim.'))


if __name__=='__main__':
    main()

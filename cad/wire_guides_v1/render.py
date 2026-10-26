"""Saved local motor/guide before, after and lid-removal illustrations."""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
spec=importlib.util.spec_from_file_location('_guide_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


def main():
    out=HERE/'images'
    if out.exists():
        raise FileExistsError('Keep prior images')
    check=r.read_json(HERE/'reviews/check.json')
    if not check['completed'] or not check['final_nominal_clear'] or not check['all_mounts_nominal_seated']:
        raise ValueError('Incomplete/nonseated guide candidate')
    saved=r.read_json(HERE/'generated/scene.json')
    d=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral=poses.forward_kinematics(d,np.zeros(20))['bodies']
    source={}
    def pin(path):
        source[path.relative_to(ROOT).as_posix()]=r.sha(path)
        return path
    for path in [HERE/'render.py',HERE/'reviews/check.json',HERE/'generated/scene.json',ROOT/'cad/remote_motor_lead_v2/generated/scene.json',
        ROOT/'cad/wiring_layout_v1/generated/packaging.json',ROOT/'cad/gesture_hand_v3/render_gestures.py',
        ROOT/'cad/gesture_hand_v3/poses.py',ROOT/'cad/functional_motion_v3/generated/catalog.json']:
        pin(path)
    out.mkdir()
    evidence=HERE/'generated/visual_evidence';evidence.mkdir()
    old=r.read_json(ROOT/'cad/wiring_layout_v1/generated/packaging.json')['parts']
    wires=r.read_json(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts']
    rackrow=next(row for row in old if row['name']=='Forearm_twenty_axis_module_rack')
    rack=cq.importers.importStep(str(pin(ROOT/rackrow['step']))).val()
    crop=rack.intersect(cq.Solid.makeBox(85,43,72,cq.Vector(-85,-44,-112)),tol=1e-6)
    path=evidence/'rack_local_visual_only.stl';cq.exporters.export(crop,str(path),tolerance=.05,angularTolerance=.1);pin(path)
    rackvis=dict(name='rack_local_visual_only',mesh=path.relative_to(ROOT).as_posix(),color=rackrow['color'])
    assemblies=[]
    name='index_spread_actuator'
    for version in ['before','after','open']:
        rows=old if version=='before' else saved['retained_packaging']+saved['new_parts']
        records=[row for row in rows if row.get('module')==name]+[row for row in wires if row.get('module')==name]+[rackvis]
        parts=[]
        for row in records:
            if version=='open' and row.get('kind')=='M2x6_envelope':
                continue
            path=pin(ROOT/row['mesh'])
            if row.get('role')=='nominal_electrical_wire_space':
                color=[.98,.65,.12]
            elif row.get('role')=='catalog_outer_box_unverified_mating':
                color=[.82,.83,.7]
            else:
                color=row.get('color',[.7,.72,.64])[:3]
            shift=np.eye(4)
            if version=='open' and row.get('kind')=='removable_lid':
                shift[2,3]=12.
            parts.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,
                neutral_inverse=np.linalg.inv(neutral['palm'])@shift))
        assemblies.append(dict(parts=parts,design=d))
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([-.85,-.65,.65]),1.)
    try:
        tiles=[]
        for assembly,label in zip(assemblies,['이전: 열린 전선 끝 · 고정 가이드 없음','후보: 기존 볼트 자리로 가이드/덮개 장착','덮개 나사2개 제거 후12mm 들어 올린 모습']):
            pixels,bounds=r.render_one(assembly,np.zeros(20),'local',{'local':camera},1000,mujoco)
            tiles.append((label,pixels))
        r.sheet(tiles[:2],2,1000,'전선 가이드 후보 · 손/모터/랙/전선 경로는 그대로',
            '16모터 중1개 국부 보기 / 새 가이드와 체결품만 추가 / 옆으로 빠짐 억제용, 축 방향 당김·마찰·강도는 미검증').save(out/'guides.png')
        r.sheet([tiles[1],tiles[2]],2,1000,'전선 교체를 위한 분리형 덮개 · 지지대에 붙은 아래 가이드는 유지',
            '오른쪽은 분리 위치 설명 / 실제 공급 공구·전체 조립 순서·인장완화 성능 승인은 아님').save(out/'lid_access.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:
                assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/path)==h for path,h in source.items()):
        raise ValueError('Render input changed')
    r.write_json(out/'report.json',dict(completed=True,input_sha256=source,outputs={name:r.sha(out/name) for name in ['guides.png','lid_access.png']},
        actual_saved_meshes=True,simulation_steps=0,local_module='index_spread_actuator',full_rack_used_in_audit=True,
        visual_only_rack_crop_mm=[[-85,-44,-112],[0,-1,-40]],open_lid_translation_mm=[0,0,12],open_view_removed_kind='M2x6_envelope'))


if __name__=='__main__':
    main()

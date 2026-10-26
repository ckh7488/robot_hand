"""Local before/after views of real saved motor and electrical-tail geometry."""
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
spec=importlib.util.spec_from_file_location('_lead_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


def main():
    out=HERE/'images'
    if out.exists():
        raise FileExistsError('Preserve previous images')
    report=r.read_json(HERE/'reviews/corrected.json')
    if not report['completed'] or not report['nominal_leads_clear'] or not report['input_hash_guard']:
        raise ValueError('No completed geometric tail candidate')
    sources={}
    def pin(path):
        sources[path.relative_to(ROOT).as_posix()]=r.sha(path)
        return path
    for path in [HERE/'render.py',HERE/'reviews/corrected.json',HERE/'generated/scene.json',
        ROOT/'cad/wiring_layout_v1/generated/packaging.json',ROOT/'cad/gesture_hand_v3/render_gestures.py',
        ROOT/'cad/gesture_hand_v3/poses.py',ROOT/'cad/functional_motion_v3/generated/catalog.json']:
        pin(path)
    d=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral=poses.forward_kinematics(d,np.zeros(20))['bodies']
    out.mkdir()
    evidence=HERE/'generated/visual_evidence'
    evidence.mkdir()
    common=[]
    shown=[]
    metadata=r.read_json(ROOT/'cad/wiring_layout_v1/generated/packaging.json')['parts']
    for row in metadata:
        if row.get('module')!=report['motor_module']:
            continue
        path=pin(ROOT/row['mesh'])
        common.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
            color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral['palm'])))
        shown.append(row['name'])
    rack=next(row for row in metadata if row['name']=='Forearm_twenty_axis_module_rack')
    original=cq.importers.importStep(str(pin(ROOT/rack['step']))).val()
    # A local cutaway for readability only. Collision checks use the whole rack.
    clip=cq.Solid.makeBox(80,43,50,cq.Vector(-80,-44,-112))
    local=original.intersect(clip,tol=1e-6)
    path=evidence/'rack_local_visual_only.stl'
    cq.exporters.export(local,str(path),tolerance=.05,angularTolerance=.1)
    pin(path)
    common.append(dict(name='rack_local_visual_only',body='palm',mesh_path=path,points=r.read_stl_points(path),
        color=rack['color'][:3],neutral_inverse=np.linalg.inv(neutral['palm'])))
    leads=[]
    for row in r.read_json(HERE/'generated/scene.json')['parts']:
        path=pin(ROOT/row['mesh'])
        if row['name']=='provisional_plug_outer_envelope':
            color=[.8,.82,.69]
        else:
            color=[[.92,.22,.16],[.22,.55,.95],[.94,.7,.08]][int(row['name'][-1])-1]
        leads.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
            color=color,neutral_inverse=np.linalg.inv(neutral['palm'])))
    assemblies=[dict(parts=common,design=d),dict(parts=common+leads,design=d)]
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([-.8,-.5,.7]),1.)
    try:
        tiles=[]
        for assembly,label in zip(assemblies,['이전: 전선이 없는 원격 모터 배치 후보','부분 배선: 모터1개 · 3선 · 임시 끝점']):
            pixels,bounds=r.render_one(assembly,np.zeros(20),'local',{'local':camera},1000,mujoco)
            tiles.append((label,pixels))
        r.sheet(tiles,2,1000,'전선 공간을 먼저 넣은 국부 보기 · 손 형상/모터 위치 변경 없음',
            '전선 외경1.9mm·굽힘 반경10mm는 가정 / 색은 선 구분 / 랙만 국부 표시 / 커넥터 실물 적합·고정·전체 배선은 미완').save(out/'remote_lead.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:
                assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/path)==h for path,h in sources.items()):
        raise ValueError('Render input changed')
    r.write_json(out/'report.json',dict(completed=True,input_sha256=sources,
        outputs={'remote_lead.png':r.sha(out/'remote_lead.png')},actual_saved_meshes=True,simulation_steps=0,
        shown_packaging=shown,visual_only_rack_crop_mm=[[-80,-44,-112],[0,-1,-62]],
        audit_full_packaging_parts=225,audit_full_hand_parts=364,hidden_from_audit=[],
        interpretation='Local illustration of one provisional motor and3electrical tails, no installed motor or full harness claim.'))


if __name__=='__main__':
    main()

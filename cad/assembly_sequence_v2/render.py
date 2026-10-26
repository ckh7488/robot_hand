"""Actual carrier-first and pitched motor-unit entry illustrations."""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_sequence_render_prior',ROOT/'cad/assembly_sequence_v1/render.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
r,poses=a.r,a.poses
spec=importlib.util.spec_from_file_location('_sequence_render_final',HERE/'final_sequence.py')
f=importlib.util.module_from_spec(spec);spec.loader.exec_module(f)


def main():
    out=HERE/'images'
    if out.exists():raise FileExistsError('Preserve prior actual CAD illustrations')
    report=r.read_json(HERE/'reviews/final_sequence.json')
    if not report['completed'] or not report['sampled_order_clear']:raise ValueError('No clear staged entry to illustrate')
    saved,new,retained,arm=f.c.load_scene()
    parts=f.q.split(saved,new,retained)
    name='middle_pip_actuator'
    group=next(row for row in saved['groups'] if row['module']==name)
    entry=next(row for row in f.configurations(saved,new,retained) if row['id']=='unit_'+name)
    pose=entry['poses'][5]
    assert pose==dict(x_mm=4.,pitch_deg=3.)
    source={}
    def pin(path):
        source[path.relative_to(ROOT).as_posix()]=r.sha(path);return path
    for path in [HERE/'render.py',HERE/'final_sequence.py',HERE/'entry_probe.py',HERE/'reviews/final_sequence.json',
        ROOT/'cad/wire_guides_v1/generated/scene.json',ROOT/'cad/gesture_hand_v3/render_gestures.py',
        ROOT/'cad/gesture_hand_v3/poses.py',ROOT/'cad/functional_motion_v3/generated/catalog.json']:
        pin(path)
    out.mkdir();evidence=HERE/'generated/visual_evidence';evidence.mkdir(parents=True)
    rackrow=parts['rack'][0];pin(ROOT/rackrow['step'])
    crop=rackrow['shape'].intersect(cq.Solid.makeBox(85,56,70,cq.Vector(-85,-176,-112)),tol=1e-6)
    mesh=evidence/'rack_local_visual_only.stl';cq.exporters.export(crop,str(mesh),tolerance=.03,angularTolerance=.1);pin(mesh)
    rackvis=dict(name='rack_local_visual_only',mesh=mesh.relative_to(ROOT).as_posix(),color=[.35,.43,.52])
    carrier=[r for r in parts['carriers']+parts['bolts']+parts['racknuts'] if r['module']==name]
    unit=[]
    for row in entry['moving']:
        pin(ROOT/row['step']);pin(ROOT/row['mesh'])
        shape=f.move(row,pose,group)['shape']
        path=evidence/(row['name']+'_entry_visual.stl')
        cq.exporters.export(shape,str(path),tolerance=.025,angularTolerance=.1);pin(path)
        unit.append(dict(row,mesh=path.relative_to(ROOT).as_posix()))
    d=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral=poses.forward_kinematics(d,np.zeros(20))['bodies']
    assemblies=[]
    for rows in [carrier+[rackvis],carrier+unit+[rackvis]]:
        records=[]
        for row in rows:
            path=pin(ROOT/row['mesh'])
            records.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
                color=row.get('color',[.65,.68,.7])[:3],neutral_inverse=np.linalg.inv(neutral['palm'])))
        assemblies.append(dict(parts=records,design=d))
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([-.8,-.85,.65]),1.)
    try:
        tiles=[]
        for assembly,label in zip(assemblies,['가이드·어댑터를 먼저 랙에 고정','모터·크래들 묶음을3° 기울여 안착']):
            pixels,bounds=r.render_one(assembly,np.zeros(20),'local',{'local':camera},1000,mujoco)
            tiles.append((label,pixels))
        r.sheet(tiles,2,1000,'같은 형상 · 설치 순서와 진입 자세만 교정',
            '그림은 중지PIP 국부 예시 / 오른쪽은+X4mm 진입 중 / 드럼은 뒤에 배치,구동 결합·지그·공구·전체 조립은 미검증').save(out/'carrier_first.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/rel)==digest for rel,digest in source.items()):raise ValueError('Changed render inputs')
    r.write_json(out/'report.json',dict(completed=True,input_sha256=source,
        output_sha256=r.sha(out/'carrier_first.png'),actual_saved_geometry=True,robot_geometry_changed=False,
        displayed_pose=pose,visual_only_rack_crop_mm=[[-85,-176,-112],[0,-120,-42]],simulation_steps=0))


if __name__=='__main__':main()

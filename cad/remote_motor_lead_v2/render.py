"""Transparent rack reveals actual common material, never a cutting target."""
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
spec=importlib.util.spec_from_file_location('_post_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')
r=importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


def main():
    mode=sys.argv[1]
    if mode not in ['before','compare']:
        raise ValueError('before or compare required')
    image=HERE/'images'/('problem.png' if mode=='before' else 'reroute.png')
    if image.exists():
        raise FileExistsError('Preserve prior image')
    image.parent.mkdir(parents=True,exist_ok=True)
    sources={}
    def pin(path):
        sources[path.relative_to(ROOT).as_posix()]=r.sha(path)
        return path
    for path in [HERE/'render.py',ROOT/'cad/gesture_hand_v3/render_gestures.py',ROOT/'cad/gesture_hand_v3/poses.py',
        ROOT/'cad/functional_motion_v3/generated/catalog.json',ROOT/'cad/wiring_layout_v1/generated/packaging.json',
        ROOT/'cad/remote_motor_lead_v1/generated/batch_scene.json']:
        pin(path)
    d=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral=poses.forward_kinematics(d,np.zeros(20))['bodies']
    folder=HERE/'generated/visual_evidence'
    folder.mkdir(parents=True,exist_ok=True)
    common=[]
    def add(parts,name,path,color,alpha=1.):
        pin(path)
        parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,alpha=alpha,
            neutral_inverse=np.linalg.inv(neutral['palm'])))
    metadata=r.read_json(ROOT/'cad/wiring_layout_v1/generated/packaging.json')['parts']
    for row in metadata:
        if row.get('module')=='middle_pip_actuator':
            add(common,row['name'],ROOT/row['mesh'],row['color'][:3])
    rackrow=next(row for row in metadata if row['name']=='Forearm_twenty_axis_module_rack')
    rack=cq.importers.importStep(str(pin(ROOT/rackrow['step']))).val()
    visual=folder/'rack_crop.stl'
    if not visual.exists():
        crop=rack.intersect(cq.Solid.makeBox(80,43,72,cq.Vector(-80,-171,-112)),tol=1e-6)
        cq.exporters.export(crop,str(visual),tolerance=.05,angularTolerance=.1)
    add(common,'rack_crop_visual_only',visual,[.25,.43,.65],.23)
    oldrows=r.read_json(ROOT/'cad/remote_motor_lead_v1/generated/batch_scene.json')['parts']
    assemblies=[]
    intersection_rows=[]
    for version in (['before'] if mode=='before' else ['before','after']):
        parts=list(common)
        if version=='before':
            rows=oldrows
        else:
            report=r.read_json(pin(HERE/'reviews/check.json'))
            if not report['completed'] or not report['all_nominal_leads_clear']:
                raise ValueError('No completed all-lead route')
            rows=r.read_json(pin(HERE/'generated/scene.json'))['parts']
        for row in rows:
            if row['module']!='middle_pip_actuator':
                continue
            iswire=row['role']=='nominal_electrical_wire_space'
            add(parts,row['name'],ROOT/row['mesh'],[.95,.65,.16] if iswire else [.8,.82,.7])
            if version=='before' and iswire:
                shape=cq.importers.importStep(str(pin(ROOT/row['step']))).val()
                overlap=shape.intersect(rack,tol=1e-6)
                vol=float(overlap.Volume())
                if vol>.001:
                    path=folder/(row['name']+'_actual_common.stl')
                    if not path.exists():
                        cq.exporters.export(overlap,str(path),tolerance=.02,angularTolerance=.06)
                    add(parts,row['name']+'_actual_common',path,[.94,.05,.09])
                    intersection_rows.append(dict(wire=row['name'],actual_common_mm3=vol))
        assemblies.append(dict(parts=parts,design=d))
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([-.8,-.5,.65]),1.)
    try:
        tiles=[]
        for assembly,label in zip(assemblies,['이전: 전선2개가 고정 기둥을 관통','수정: 전선 방향만 돌려 기둥 옆으로 우회']):
            assembly['_render_state']=r.make_renderer(assembly,{'local':camera},950,mujoco)
            state=assembly['_render_state']
            for i,part in enumerate(assembly['parts']):
                state['model'].geom_rgba[i,3]=part['alpha']
            pixels,bounds=r.render_one(assembly,np.zeros(20),'local',{'local':camera},950,mujoco)
            tiles.append((label,pixels))
        r.sheet(tiles,len(tiles),950,'원격 모터 전선 경로 · 손과 지지대 형상은 그대로',
            '빨강=실제 겹친 재료만 / 기둥은 반투명 / 4모터 중1개 국부 보기 / 선 규격·고정·전체 배선은 미완').save(image)
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:
                assembly['_render_state']['renderer'].close()
    if not all(r.sha(ROOT/path)==h for path,h in sources.items()):
        raise ValueError('Render source changed')
    r.write_json(image.with_suffix('.json'),dict(completed=True,input_sha256=sources,output_sha256=r.sha(image),
        actual_saved_meshes=True,actual_common_material=intersection_rows,transparent_visual_only=['rack_crop_visual_only'],
        full_rack_in_audit=True,simulation_steps=0))


if __name__=='__main__':
    main()

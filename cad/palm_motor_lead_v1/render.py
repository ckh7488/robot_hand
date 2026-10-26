"""Actual unchanged palm parts and registered lead geometry in two CAD views."""
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


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def main():
    g=module('_palm_lead_draw_model',HERE/'model.py');p,b,m=g.p,g.b,g.m
    r=module('_palm_lead_renderer',ROOT/'cad/gesture_hand_v3/render_gestures.py')
    out=HERE/'images'
    if (out/'palm_leads.png').exists():raise FileExistsError('Preserve existing figure')
    out.mkdir(exist_ok=True)
    saved=p.read(HERE/'generated/scene.json');mount=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json')
    report=p.read(HERE/'reviews/assessment.json');assert report['completed'] and not report['nominal_leads_clear']
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand']
    design=p.read(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    neutral=poses.forward_kinematics(design,np.zeros(20))['bodies'];sources={}
    motors=[z for z in p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')['modules'] if z['zone']=='palm']
    baseline=[]
    for motor in motors:
        for row in g.wires(motor,radius=10.,stub=5.,straight=18.,angle=0.):
            mesh=out/(row['name']+'_copied.stl');cq.exporters.export(row['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
            baseline.append(dict(name=row['name'],mesh=mesh.relative_to(ROOT).as_posix(),sha256_stl=b.sha(mesh),body='palm',color=row['color']))
    assemblies=[]
    for leads in [baseline,[z for z in saved['parts'] if 'lead' in z['name']]]:
        parts=[]
        def add(row,color,alpha):
            path=ROOT/row['mesh'];assert b.sha(path)==row['sha256_stl'];sources[row['mesh']]=row['sha256_stl']
            parts.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,alpha=alpha,neutral_inverse=np.linalg.inv(neutral['palm'])))
        for row in hand:
            if row['body']=='palm' and row['name']!='Palm_shaped_front_cover':add(row,[.64,.68,.72],.12)
        for row in mount['new_parts']+mount['retained_parts']:
            if row.get('body')=='palm':add(row,[.3,.48,.65] if row.get('kind')!='four_adapter_carrier' else [.7,.52,.3],.25)
        for row in leads:add(row,row['color'],1.)
        assemblies.append(dict(parts=parts,design=design))
    # Motor exits are underneath; look from the back through a transparent floor.
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([.15,-.35,-1.]),1.)
    images=[]
    try:
        for assembly in assemblies:
            state=r.make_renderer(assembly,{'lead':camera},1100,mujoco);assembly['_render_state']=state
            for i,z in enumerate(assembly['parts']):state['model'].geom_rgba[i,3]=z['alpha']
            im,_=r.render_one(assembly,np.zeros(20),'lead',{'lead':camera},1100,mujoco);images.append(im)
        r.sheet([('그대로 복사: 바닥을 뚫는 원격 모터용 경로',images[0]),('우회 연구: 받침과 겹침이 남아 미채택',images[1])],2,1100,
            '손·모터·고정부 유지 · 손바닥 4모터 전선 출구',
            '투명 바닥 뒤쪽 보기 · 외경1.9/반경4mm는 가정 · 손 절삭 없이 보류 · 실제 케이블/커넥터 확인 필요').save(out/'palm_leads.png')
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    assert all(b.sha(ROOT/z)==h for z,h in sources.items())
    p.write(out/'report.json',dict(completed=True,source_mesh_sha256=sources,outputs={'palm_leads.png':b.sha(out/'palm_leads.png')},
        actual_CAD=True,simulation_steps=0,cutaway_visual_only=True,hand_geometry_modified=False))


if __name__=='__main__':main()

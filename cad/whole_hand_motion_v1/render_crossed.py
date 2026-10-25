"""Two actual saved-mesh views of the photo-requested crossing candidate."""
import argparse,importlib.util,json,sys
from pathlib import Path
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--audit',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--clearance',type=Path);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=HERE.parents[1];report=read(a.audit)
    if not report['completed'] or not report['input_hash_guard']['passed']:raise ValueError('Incomplete audit')
    sys.path.insert(0,str(root/'cad/gesture_hand_v3'));p=root/'cad/gesture_hand_v3/render_gestures.py';s=importlib.util.spec_from_file_location('_cross_renderer',p);r=importlib.util.module_from_spec(s);s.loader.exec_module(r)
    for k,h in report['source_sha256'].items():
        if r.sha(root/k)!=h:raise ValueError('Changed audited input')
    fallback=min(report['cases'],key=lambda c:c['result']['collision_count'])
    case=next((c for c in report['cases'] if c['id']==report['selected']),fallback);design=report['design'];neutral=r.poses.forward_kinematics(design,np.zeros(20))['bodies']
    scene=read(root/'cad/thumb_ip_drive_v1/generated/scene.json');parts=[];bad={c[k] for c in case['result']['collision_details'] for k in ['a','b']}
    for row in scene['hand']:
        mesh=root/row['mesh'] if row['location']=='repo' else root/'cad/thumb_ip_drive_v1/generated'/row['mesh']
        if r.sha(mesh)!=row['sha256_stl']:raise ValueError('Changed STL')
        parts.append(dict(name=row['name'],body=row['body'],mesh_path=mesh,points=r.read_stl_points(mesh),
            neutral_inverse=np.linalg.inv(neutral[row['body']]),color=[.9,.16,.12] if row['name'] in bad else row['color'][:3]))
    assembly=dict(parts=parts,design=design);views={'front':np.array([0.,0.,1.]),'oblique':np.array([.8,-.35,1.])}
    cameras={n:r.view_fit([assembly],[case],normal,1) for n,normal in views.items()};tiles=[]
    for view,label in [('front','정면'),('oblique','사선 · 두 손가락의 앞뒤 간격')]:
        im,_=r.render_one(assembly,case['angles_deg'],view,cameras,700,mujoco);tiles.append((label,im))
    assembly['_render_state']['renderer'].close()
    footer='목표 자세와 접근 경로 표본에서 보고 기준 이상 겹침0. 실물 구동·공차·연속운동은 미검증.' if report['selected'] else '검사한 후보에서 간섭이 남음. 빨강은 충돌 부품이며 가능한 자세로 승인하지 않음.'
    if a.clearance:
        gap=read(a.clearance)
        if not gap['completed'] or gap['source_audit_sha256']!=r.sha(a.audit):raise ValueError('Mismatched clearance report')
        if gap['meets_sampled_half_mm_goal']:footer='11표본: 겹침 기준 통과, 검지/중지 간격0.5mm 이상. 연속운동·공차·실물 미검증.'
        else:footer='겹침량 보고 기준과 별개로 검지-중지 간격0.5mm 목표 미달. 움직임 승인 아님.'
    a.output.parent.mkdir(parents=True,exist_ok=True);r.sheet(tiles,2,700,'검지·중지 교차 · 실제364부품 CAD',footer).save(a.output)
    a.output.with_suffix('.json').write_text(json.dumps(dict(source_audit_sha256=r.sha(a.audit),case=case['id'],angles_deg=case['angles_deg'],red_parts=sorted(bad),mesh_scene='cad/thumb_ip_drive_v1/generated/scene.json',views=['front','oblique']),indent=2),encoding='utf-8')
if __name__=='__main__':main()

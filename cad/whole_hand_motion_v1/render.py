"""Render hash-matched saved meshes at audited poses; red denotes interference."""
import argparse,copy,hashlib,importlib.util,json,sys
from pathlib import Path
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def mod(name,p):
    s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,default=HERE.parents[1]);ap.add_argument('--audit',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=a.repo;report=read(a.audit)
    if not report['completed'] or report['errors'] or not report['input_hash_guard']['passed']:raise ValueError('Incomplete audit')
    for k,h in report['source_sha256'].items():
        if sha(root/k)!=h:raise ValueError('Stale '+k)
    sys.path.insert(0,str(root/'cad/gesture_hand_v3'))
    r=mod('_motion_renderer',root/'cad/gesture_hand_v3/render_gestures.py')
    design=report['requests']['design'];neutral=r.poses.forward_kinematics(design,np.zeros(20))['bodies']
    scene=read(root/'cad/manufacturing_v2/generated/scene.json');parts=[]
    for row in scene['hand']:
        mesh=root/'cad/manufacturing_v2'/row['mesh']
        if sha(mesh)!=row['sha256_stl']:raise ValueError('Changed mesh')
        parts.append(dict(name=row['name'],body=row['body'],mesh_path=mesh,points=r.read_stl_points(mesh),
            color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral[row['body']])))
    assembly=dict(parts=parts,design=design)
    cases={c['id']:c for c in report['cases']}
    wanted=[('gesture_thumb_up','수용한 엄지척'),('four_fingers_hook','갈고리 모양'),
            ('index_ordinary_extension_mcp','검지 뿌리 뒤로20°'),('four_fingers_extension','네 손가락 젖힘 -25/-15/-8°'),
            ('index_pip_+135','검지 가운데 관절 135°'),('four_fingers_curl','네 손가락 접기 85/110/80°')]
    selected=[(cases[k],title) for k,title in wanted if k in cases]
    cameras={'oblique':r.view_fit([assembly],[c for c,_ in selected],np.array([.8,-.45,1.]),1)}
    state=r.make_renderer(assembly,cameras,600,mujoco);assembly['_render_state']=state
    tiles=[];metadata=[]
    for c,title in selected:
        collision_names={d[k] for d in c['result']['collision_details'] for k in ['a','b']}
        for i,p in enumerate(parts):state['model'].geom_rgba[i,:3]=[.9,.16,.12] if p['name'] in collision_names else p['color']
        im,bounds=r.render_one(assembly,c['angles_deg'],'oblique',cameras,600,mujoco)
        count=c['result']['collision_count'];tiles.append((title+f' · 간섭 {count}쌍',im));metadata.append(dict(id=c['id'],red_parts=sorted(collision_names),bounds=bounds))
    state['renderer'].close();assembly.pop('_render_state')
    a.output.mkdir(parents=True)
    r.sheet(tiles,3,600,'손 전체 실제 CAD · 넓힌 동작 검사',
        '빨강 = 해당 자세에서 실제 STEP 재료가 겹치는 부품. 0쌍도 연속 운동·제작 승인 아님.').save(a.output/'motion_findings.png')
    # Actual conflicting part-pair details, without rendering other parts over it.
    detail_tiles=[]
    for c,title in selected:
        hits=c['result']['collision_details']
        if not hits:continue
        hit=max(hits,key=lambda d:d['material_intersection_mm3']);named=[hit['a'],hit['b']]
        sub=copy.copy(assembly);sub['parts']=[dict(p,color=[.85,.16,.12] if p['name']==named[0] else [.17,.42,.75]) for p in parts if p['name'] in named]
        cam={'detail':r.view_fit([sub],[c],np.array([.8,-.4,1.]),1)}
        im,_=r.render_one(sub,c['angles_deg'],'detail',cam,600,mujoco)
        sub['_render_state']['renderer'].close()
        detail_tiles.append((title+f" · {hit['material_intersection_mm3']:.2f} mm³",im))
        metadata.append(dict(detail_id=c['id'],parts=named,overlap_mm3=hit['material_intersection_mm3']))
    if detail_tiles:r.sheet(detail_tiles,2,600,'간섭 부품 확대 · 나머지 부품은 숨김',
        '빨강/파랑은 실제 충돌한 두 부품. 부품명·자세·겹침량은 render_metadata.json에 기록.').save(a.output/'collision_details.png')
    (a.output/'render_metadata.json').write_text(json.dumps(dict(audit_sha256=sha(a.audit),panels=metadata),indent=2),encoding='utf-8')
if __name__=='__main__':main()

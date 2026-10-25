"""Show the saved candidate hand and its separate motor development module.

No geometry reconstruction or dynamics simulation. All meshes are checked
against the build manifest. Isolated/bench views explicitly hide other parts.
"""
from pathlib import Path
import argparse,hashlib,importlib.util,json
import numpy as np
from PIL import Image,ImageDraw


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2]);ap.add_argument('--generated',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();repo=a.repo.resolve();root=a.generated.resolve();out=a.output.resolve()
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True)
    def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
    hashes={}
    def bind(p,expected=None):
        p=Path(p).resolve();key=p.relative_to(repo).as_posix();value=digest(p)
        if expected is not None and expected!=value:raise ValueError('Stale render input '+key)
        if key in hashes and hashes[key]!=value:raise ValueError('Render input changed '+key)
        hashes[key]=value;return p
    def module(n,p):
        s=importlib.util.spec_from_file_location(n,bind(p));m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
    bind(__file__)
    h=module('_drive_render_helper',repo/'cad/manufacturing_v1/render.py');inputs=h.Inputs(repo);profile,r=h.read_profile(repo,inputs)
    scene=json.loads(bind(root/'scene.json').read_text());build=json.loads(bind(root/'build_report.json').read_text());design=json.loads(bind(root/'design.json').read_text())
    if not build['completed'] or build['errors'] or not build['input_hash_guard']['passed']:raise ValueError('Incomplete build')
    for k,v in build['source_sha256'].items():
        bind(repo/k,v)
    for k,v in build['generated_sha256'].items():
        bind(root/k,v)
    neutral=profile['poses'].forward_kinematics(design,np.zeros(20))['bodies'];parts=[]
    for row in scene['hand']:
        p=(root if row['location']=='generated' else repo)/row['mesh']
        bind(p,row['sha256_stl'])
        parts.append(dict(name=row['name'],body=row['body'],mesh_path=p,points=r.read_stl_points(p),color=row['color'][:3],neutral_inverse=np.linalg.inv(neutral[row['body']])))
    assembly=dict(parts=parts,design=design,label='엄지 줄 구동 후보')
    gestures=[profile['document']['gestures'][0],profile['document']['gestures'][2]]
    cameras={label:r.view_fit([assembly],gestures,np.array(normal,dtype=float),1) for label,normal in [('front',[0,0,1]),('iso',[.7,-.3,1.])]}
    import mujoco
    tiles=[]
    for label,g,view in [('손 편 상태',gestures[0],'front'),('따봉 · 엄지 끝 -15°',gestures[1],'front'),('따봉 · 사선',gestures[1],'iso')]:
        tile,_=r.render_one(assembly,g['angles_deg'],view,cameras,850,mujoco);tiles.append((label,tile))
    r.sheet(tiles,3,850,'손 비율·관절 위치 유지 · 엄지 줄 고정부를 추가한 후보','저장 CAD 렌더 · 모터는 별도 시험 모듈 / 전체 손 구동·제작·지연 승인 전').save(out/'hand_candidate.png')
    assembly['_render_state']['renderer'].close()
    # Detail is the actual local geometry with the rest of the hand hidden.
    ip_inv=np.linalg.inv(neutral['thumb_ip']);selected=[]
    for p in parts:
        if p['body'] in ('thumb_mcp','thumb_ip'):
            selected.append(dict(p,body='palm',neutral_inverse=ip_inv))
    detail=dict(parts=selected,design=design,label='엄지 관절 상세')
    camera={'iso':r.view_fit([detail],[dict(angles_deg=np.zeros(20))],np.array([1.,-.4,.65]),1)}
    joint,_=r.render_one(detail,np.zeros(20),'iso',camera,1000,mujoco);detail['_render_state']['renderer'].close()
    motor_parts=[]
    for row in scene['motor_bench']:
        p=root/row['mesh'];color=([.98,.48,.08] if row['name']=='drive_dual_groove_spool' else [.35,.43,.52])
        bind(p,row['sha256_stl'])
        if row['name']=='drive_metal_horn':color=[.72,.76,.8]
        motor_parts.append(dict(name=row['name'],body='palm',mesh_path=p,points=r.read_stl_points(p),color=color,neutral_inverse=np.eye(4)))
    motor=dict(parts=motor_parts,design=design,label='별도 모터 모듈')
    camera={'iso':r.view_fit([motor],[dict(angles_deg=np.zeros(20))],np.array([1.,-.8,.7]),1)}
    im,_=r.render_one(motor,np.zeros(20),'iso',camera,1000,mujoco);motor['_render_state']['renderer'].close()
    r.sheet([('엄지 끝관절 · 나머지 손 숨김',joint),('두 줄 스풀 · 별도 모터 시험 모듈',im)],2,1000,
        '실제 부품 CAD · 서로 다른 축척의 국부 확대','주황: 새 줄 고정부와 스풀 / 중앙 혼 고정·장력 조절·완성 시험대는 미완료').save(out/'drive_details.png')
    for k,v in inputs.hashes.items():bind(repo/k,v)
    end={k:digest(repo/k) for k in hashes};guard=hashes==end
    images={p.name:digest(p) for p in out.glob('*.png')}
    (out/'render_report.json').write_text(json.dumps(dict(completed=guard,status='completed' if guard else 'input_changed',errors=[] if guard else ['Input changed'],
        inputs_unchanged=guard,source_sha256=hashes,input_hash_guard=dict(start=hashes,end=end,passed=guard),
        saved_mesh_only=True,whole_hand_parts=len(parts),detail_parts=len(selected),motor_rows=len(motor_parts),
        hand_pose_ids=[g['id'] for g in gestures],build_sha256=digest(root/'build_report.json'),images=images,output_sha256=images),indent=2)+'\n')
    if not guard:raise ValueError('Render inputs changed')
    print('Saved CAD render complete',flush=True)


if __name__=='__main__':main()

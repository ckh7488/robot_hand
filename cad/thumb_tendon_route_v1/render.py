"""Saved actual rope/hand meshes and the still-failed root routing location."""
import importlib.util,sys,types
from pathlib import Path
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from PIL import ImageDraw
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.dont_write_bytecode=True;sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
s=importlib.util.spec_from_file_location('_tendon_render',ROOT/'cad/gesture_hand_v3/render_gestures.py');r=importlib.util.module_from_spec(s);s.loader.exec_module(r)


class SerialModel:
    @staticmethod
    def from_xml_string(xml):
        node=r.ET.fromstring(xml);node.find('compiler').set('usethread','false')
        return mujoco.MjModel.from_xml_string(r.ET.tostring(node,encoding='unicode'))


engine=types.SimpleNamespace(MjModel=SerialModel,MjData=mujoco.MjData,Renderer=mujoco.Renderer,mj_forward=mujoco.mj_forward)


def main():
    out=HERE/'images';out.mkdir(exist_ok=False)
    report=r.read_json(HERE/'reviews/calibrated.json');first=r.read_json(HERE/'reviews/audit.json');interface=r.read_json(HERE/'reviews/interfaces.json')
    assert report['completed'] and interface['completed'];source={}
    design=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design'];neutral=poses.forward_kinematics(design,np.zeros(20))['bodies']
    hand=r.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand']
    mount=r.read_json(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json')
    wires=r.read_json(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts']
    evidence=r.read_json(HERE/'generated/visual_evidence/scene.json')['parts']
    layout=r.read_json(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    M=np.asarray(next(z['transform_world_mm'] for z in layout['modules'] if z['name']=='thumb_ip_actuator'))
    def assembly(case,zoom=False,evidence_case=None):
        parts=[]
        def add(row,body,color,alpha,T):
            path=ROOT/row['mesh'];assert r.sha(path)==row['sha256_stl'];source[row['mesh']]=row['sha256_stl']
            parts.append(dict(name=row['name'],body=body,mesh_path=path,points=r.read_stl_points(path),color=list(color),alpha=alpha,neutral_inverse=T))
        for z in hand:
            if zoom and not z['body'].startswith('thumb'):continue
            add(z,z['body'],[.63,.68,.73],.22 if zoom else .3,np.linalg.inv(neutral[z['body']]))
        if not zoom:
            angle=case['kinematics']['motor_angle_deg'];T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(M[:3,0]*np.deg2rad(angle)).as_matrix();T[:3,3]=M[:3,3]-T[:3,:3]@M[:3,3]
            for z in mount['retained_parts']+mount['parts']:add(z,'palm',[.3,.36,.43],.12,T if z.get('moving') else np.eye(4))
            for z in wires:add(z,'palm',[.38,.43,.5],.3,np.eye(4))
        for z in r.read_json(ROOT/case['scene'])['parts']:
            if zoom and int(z['name'].split('_')[2])<7:continue
            add(z,'palm',z['color'],1.,np.eye(4))
        if evidence_case:
            for z in evidence:
                if z['source_case']==evidence_case:add(z,'palm',[.95,.03,.06],1.,np.eye(4))
        return dict(parts=parts,design=design)
    open_case=next(z for z in report['cases'] if z['id']=='open');current=next(z for z in report['cases'] if z['id']=='parent_mid');old=next(z for z in first['cases'] if z['id']=='parent_mid')
    overview=assembly(open_case);before=assembly(old,True,'initial_parent_mid');after=assembly(current,True,'calibrated_parent_mid')
    close_camera=r.view_fit([before,after],[dict(angles_deg=current['angles_deg'])],np.array([.4,-.7,1.]),1.)
    overview_camera=r.view_fit([overview],[dict(angles_deg=open_case['angles_deg'])],np.array([.3,-.25,1.]),1.)
    images=[]
    try:
        for a,case,camera in [(overview,open_case,overview_camera),(before,old,close_camera),(after,current,close_camera)]:
            a['_render_state']=r.make_renderer(a,{'view':camera},1000,engine)
            for i,z in enumerate(a['parts']):a['_render_state']['model'].geom_rgba[i,3]=z['alpha']
            image,_=r.render_one(a,case['angles_deg'],'view',{'view':camera},1000,engine)
            if a is not overview:
                tag='initial_parent_mid' if a is before else 'calibrated_parent_mid';cloud=np.concatenate([r.read_stl_points(ROOT/z['mesh']) for z in evidence if z['source_case']==tag])
                center=cloud.mean(axis=0)-camera['focus'];factor=1000/(2*camera['parallel_scale_mm']);x=500+np.dot(center,camera['right'])*factor;y=500-np.dot(center,camera['up'])*factor
                draw=ImageDraw.Draw(image);draw.ellipse((x-30,y-30,x+30,y+30),outline=(220,20,35),width=4)
                draw.text((max(8,x-100),y+34),'실제 겹침 위치',font=r.font(24),fill=(185,20,30))
            images.append(image)
        r.sheet([('편 자세: 모터에서 엄지 끝까지 두 줄',images[0]),('엄지 뿌리 함께 굽힘: 걸림이 남음',images[2])],2,1000,
            '두 구동줄의 실제 CAD 경로 후보 · 손/모터 위치 유지',
            '주황/파랑: 구동줄 · 회색: 기존 손/모터/전기선 공간 모델 · 가이드 고정부/장력 구조 미완').save(out/'route_status.png')
        r.sheet([('첫 경로: 엄지 중간 마디를 가로지름',images[1]),('바깥 경로 교정: 뿌리 쪽 추가 우회 필요',images[2])],2,1000,
            '시작과 끝의 통과만으로 움직임 성공이라고 할 수 없음',
            '빨강은 실제 공통 재료 · 손을 깎지 않고 줄/가이드 경로를 교정할 근거 · 두 후보 모두 전체 미채택').save(out/'route_before_after.png')
    finally:
        for a in [overview,before,after]:
            if '_render_state' in a:a['_render_state']['renderer'].close()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='C:/Windows/Fonts/malgun.ttf');plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(2,1,figsize=(10,8),layout='constrained')
    for item,color,label in [(first,'#82909c','첫 자유 경로'),(report,'#e27424','부모 바깥 경로 후보')]:
        rows=item['parent_length_study'];ax[0].plot([z['fraction']*100 for z in rows],[z['kinematics']['common_length_residual_mm'] for z in rows],marker='o',color=color,label=label)
    ax[0].axhline(0,color='#445',lw=.8);ax[0].set(xlabel='기존 공 접근 자세의 진행 비율 (%)',ylabel='두 줄이 함께 변하는 길이 (mm)',title='모터 회전으로 두 줄의 차이를 맞춰도 공통 길이 변화는 남음');ax[0].legend();ax[0].grid(alpha=.2)
    for cord,color in [(0,'#e27424'),(1,'#2796c0')]:
        rows=[z for z in interface['guide_constraint_diagnostics'] if z['cord']==cord];ax[1].plot([z['guide_index'] for z in rows],[z['max_center_change_in_body_mm'] for z in rows],marker='o',color=color,label='구동줄 '+str(cord+1))
    ax[1].set(xlabel='모터→엄지 방향의 가이드 예정 위치',ylabel='해당 몸체에 대한 중심 변화 (mm)',title='곡선의 가이드 중심도 움직이므로 실제 고정부 설계가 필요');ax[1].grid(alpha=.2);ax[1].legend()
    fig.suptitle('자유 곡선 후보의 계산값이며 텐셔너 규격·실제 고정 가이드 검증은 아님',fontsize=12);fig.savefig(out/'length_and_guides.png',dpi=150);plt.close(fig)
    assert all(r.sha(ROOT/z)==h for z,h in source.items())
    r.write_json(out/'report.json',dict(completed=True,input_sha256=source,actual_saved_meshes=True,simulation_steps=0,mujoco_asset_compile_threads=False,
        outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},scope='Current one-motor two-cord layout and actual failed root material intersection.Original MCP failure preserved.Free-guide/length charts are not installed pulley/force dynamics or complete hand drive evidence.'))


if __name__=='__main__':main()

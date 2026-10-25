"""Explicit thumb range/pose layer over unchanged GESTURE002 native geometry."""
from pathlib import Path
import copy,importlib,json,sys
import numpy as np

HERE=Path(__file__).resolve().parent
def load(repo=None):
    repo=Path(repo or HERE.parents[1]).resolve()
    specification=json.loads((HERE/'profile.json').read_text(encoding='utf-8'))
    base=repo/specification['base_candidate']
    sys.path.insert(0,str(base))
    poses=importlib.import_module('poses')
    catalog=poses.load_catalog(base/'poses.json')
    original=json.loads((base/'design.json').read_text(encoding='utf-8'))
    design=copy.deepcopy(original)
    if original['joint_ranges_deg']['thumb_ip']!=[0,85]:
        raise ValueError('Unexpected baseline thumb IP range')
    if specification['target_thumb_deg']!=[0,0,0,-15] or specification['exploratory_thumb_ip_range_deg']!=[-20,85]:
        raise ValueError('This version explicitly tests the wrist-directed 15-degree candidate')
    design['joint_ranges_deg']['thumb_ip']=specification['exploratory_thumb_ip_range_deg']
    opened=np.zeros(20)
    folded=np.array(catalog['reference']['authored_fist_deg'],float);folded[16:]=0
    selected=folded.copy();selected[16:]=specification['target_thumb_deg']
    reference=catalog['reference']['authored_fist_path']
    split=specification['finger_closing_phase_end']
    if len(reference)!=33 or not 0<split<1:raise ValueError('Expected accepted33-frame finger path')
    path=[]
    for index,row in enumerate(reference):
        q=np.array(row['angles_deg'],float);q[16:]=0
        path.append(dict(fraction=float(row['fraction'])*split,angles_deg=q.tolist(),
                         phase='four_fingers_close_thumb_fixed',authored_finger_frame=index))
    if path[0]['angles_deg']!=opened.tolist() or path[-1]['angles_deg']!=folded.tolist():
        raise ValueError('Finger path does not match declared endpoints')
    path.append(dict(fraction=1.,angles_deg=selected.tolist(),phase='thumb_tip_toward_wrist'))
    gestures=[dict(id='open',label_ko='손을 쫙 편 상태',angles_deg=opened.tolist()),
              dict(id='fingers_folded',label_ko='엄지 유지 · 네 손가락만 쥠',angles_deg=folded.tolist()),
              dict(id='thumb_wrist',label_ko='끝마디만 팔목 쪽으로15°',angles_deg=selected.tolist())]
    document=dict(joint_names=poses.NAMES,gestures=gestures,kinematic_design=design,
        transitions=[dict(id='open__thumb_wrist',source='open',target='thumb_wrist',
                         waypoints=path,status='candidate',method='accepted_finger_path_then_thumb_ip_only')])
    for g in gestures:poses.validate_angles(g['angles_deg'],design)
    for row in path:poses.validate_angles(row['angles_deg'],design)
    wrist_plate=next(row for row in json.loads((base/'generated/scene.json').read_text(encoding='utf-8'))['hand']
                     if row['name']=='Wrist_bridge_plate')
    bbox=np.asarray(wrist_plate['bbox_mm'],float)
    wrist=[float((bbox[0,0]+bbox[1,0])/2),float((bbox[0,1]+bbox[1,1])/2),float(bbox[0,2])]
    metrics=[]
    for g in gestures[1:]:
        fk=poses.forward_kinematics(design,g['angles_deg']);tip=fk['chains']['thumb'][-1]
        metrics.append(dict(id=g['id'],tip_world_mm=tip.tolist(),
            tip_longitudinal_y_from_wrist_mm=float(tip[1]-wrist[1]),distance_to_wrist_reference_mm=float(np.linalg.norm(tip-wrist))))
    delta=np.array(metrics[1]['tip_world_mm'])-np.array(metrics[0]['tip_world_mm'])
    if not delta[1]<0 or not metrics[1]['distance_to_wrist_reference_mm']<metrics[0]['distance_to_wrist_reference_mm']:
        raise ValueError('Candidate does not move toward the wrist reference')
    direction=dict(wrist_reference_mm=wrist,
        reference_meaning='Centre XY of the saved Wrist_bridge_plate bounding box, lower mounting face Z; a mechanical reference, not an anatomical rotation centre.',
        wrist_direction_world=[0,-1,0],tip_delta_world_mm=delta.tolist(),poses=metrics,
        interpretation='Tip moves toward wrist in hand-longitudinal projection and slightly closer to mounting reference. It does not directly point at the wrist.')
    return dict(repo=repo,base=base,poses=poses,catalog=catalog,original_design=original,design=design,
                specification=specification,document=document,direction=direction)

import json,math,numpy as np
from pathlib import Path
D=Path(__file__).resolve().parent;M=json.loads((D/'cad_manifest.json').read_text(encoding='utf-8'));S=json.loads((D/'scene.json').read_text(encoding='utf-8'))['Open'];cfg=json.loads((D.parent.parent/'robot_config.json').read_text(encoding='utf-8'));rows=[]
for it in S:
 n=it['name'];a=M['assets'][it['asset']];mass=0;pos=None
 if it['role']=='vendor_motor':mass=.018;pos=np.array(it['pos_mm'])+np.array(it['R'])@np.array([-15,-7.5,0])
 elif it['role']=='vendor_rotor':continue
 elif it['role']=='arm_motor_nominal':
  mass=cfg['arm']['motors'][n.removeprefix('AK_')]['mass_kg'];pos=np.array(a['com_mm'])
 else:
  mass=a['volume_mm3']*(7.85e-6 if it['role']=='fastener' else 1.25e-6);pos=np.array(it['pos_mm'])+np.array(it['R'])@np.array(a['com_mm'])
 rows.append(dict(name=n,mass_kg=mass,com_mm=pos.tolist(),role=it['role']))
hand=[r for r in rows if not r['role'].startswith('arm_')];hm=sum(r['mass_kg'] for r in hand);hp=sum((r['mass_kg']*np.array(r['com_mm']) for r in hand),np.zeros(3))/hm
elbow_y=M['axis_locations_y_mm']['elbow'];flex_y=M['axis_locations_y_mm']['wrist_flex'];moving=[r for r in rows if r['name'] not in ['AK_elbow_flex','Elbow_fixed_cradle']];wrist=[r for r in rows if r in hand or r['name'] in ['AK_wrist_deviation','Wrist_box_carrier','Palm_wrist_saddle']]
def torque(rows,p):
 moment=sum((r['mass_kg']*(np.array(r['com_mm'])-p)/1000 for r in rows),np.zeros(3));return 9.81*np.linalg.norm(moment[1:])
out={'status':'Illustrative full-solid mass screen, NOT validated material or measured mass','assumed_density_g_cm3':1.25,'frame_mass_assumption':'Vendor H101/S101 volumes also assigned same illustrative density; true masses unknown','fasteners_included':True,'wires_included':False,'fastener_density_g_cm3':7.85,'hand_mass_kg':hm,'hand_com_mm':hp.tolist(),'elbow_max_neutral_gravity_about_X_Nm':torque(moving,np.array([0,elbow_y,0])),'wrist_flex_max_neutral_gravity_about_X_Nm':torque(wrist,np.array([0,flex_y,0])),'hand_printed_solid_volume_cm3':sum(M['assets'][it['asset']]['volume_mm3']/1000 for it in S if it['role']=='printed'),'items':rows,'limitations':['No fatigue, stiffness, printed-layer material data or motor acceleration simulation.','Only modeled steel-envelope fastener masses included; harness and unmodeled arm flange fasteners omitted.','Neutral configuration first moment only, not a full joint-range gravity search.','Use measured printed part and purchased frame masses before motor decisions.']}
(D/'mass_screen.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k!='items'},indent=2))

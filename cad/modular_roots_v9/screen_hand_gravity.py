"""CAD mass first moments. Orientation-wise worst static gravity, not a motor response simulation."""
import json,numpy as np
from kinematics_numpy import C,D,META,forward,rigid_body
M=json.loads((D/'cad_manifest.json').read_text(encoding='utf-8'));out={'source':'CAD volumes and COM with1.25g/cm3 polymer,7.85g/cm3 hardware,18g per XL330','motor_reference':{'source_url':'https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/','at_5V_stall_Nm':.215,'stall_current_A':1.47,'stall_ratio_Nm_per_A':.146,'no_load_rpm':383,'caution':'Stall values are not continuous ratings. Current-scaled torque is a rough screen, not a calibrated actuator model.'},'joints':[]}
for f in C['hand']['fingers']:
 n=f['name'];best={}
 for pname,p in C['poses'].items():
  mat=forward(p);idx={it['name']:i for i,it in enumerate(META)}
  for j,tag in enumerate(['root','mcp','pip','dip']):
   axisbody=n+'_'+tag+'_motor';a=mat[idx[axisbody]];origin=a[:3,3];axis=a[:3,0];allowed=[n+'_'+s for s in ['spread','mcp','pip','dip'][j:]];moment=np.zeros(3);movingmass=0.;inertia=0.
   for it,t in zip(META,mat):
    if rigid_body(it['name']) not in allowed:continue
    spec=M['assets'][it['asset']]
    if it['role']=='vendor_motor':mass=.018;com=np.array([-15,-7.5,0.])
    elif it['role']=='vendor_rotor':continue
    else:mass=spec['volume_mm3']*(7.85e-6 if it['role']=='fastener' else 1.25e-6);com=np.array(spec['com_mm'])
    pcom=t[:3,:3]@com+t[:3,3];rv=(pcom-origin)/1000;moment+=mass*rv;inertia+=mass*np.linalg.norm(np.cross(axis,rv))**2;movingmass+=mass
   torque=9.81*np.linalg.norm(np.cross(axis,moment));row={'joint':n+'_'+tag,'pose':pname,'worst_hand_orientation_gravity_Nm':float(torque),'downstream_kg':movingmass,'rough_current_for_static_A':float(torque/.146),'fraction_5V_stall':float(torque/.215),'point_mass_inertia_lower_bound_kg_m2':float(inertia),'inertia_only_90deg_150ms_minjerk_peak_Nm':float(inertia*(10/np.sqrt(3))*np.pi/2/.15**2)}
   if tag not in best or best[tag]['worst_hand_orientation_gravity_Nm']<torque:best[tag]=row
 out['joints']+=list(best.values())
out['limitations']=['Worst gravity direction is optimized separately per joint; these maxima need not occur at the same hand orientation.','No acceleration, transmission friction, motor rotor reflected inertia, cable load or continuous thermal rating.','Motor COM is an approximate full-motor box; vendor frame mass density is assumed.','Full-solid CAD mass; slicer infill and actual purchased masses require updates.']
(D/'reviews/hand_joint_gravity.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out['joints'],indent=2))

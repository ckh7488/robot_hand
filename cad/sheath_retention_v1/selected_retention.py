"""The unchanged upper clamp rolled about its own housing axis."""
from retention_model import *
import retention_model as original
def build():
    rows,design=original.build();probe=p.read(HERE/'reviews/roll_probe.json');angle=probe['selected_roll_deg'];assert angle in [90.,-90.]
    port=next(z for z in design['ports'] if z['tag']=='distal_housing_clamp_1');Q=np.asarray(port['Q']);a=np.radians(angle);T=np.eye(4);T[:3,:3]=[[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]];W=Q@T@np.linalg.inv(Q)
    rows=[dict(z,shape=g.original.transform(z['shape'],W)) if z['name'].startswith('distal_housing_clamp_1_') else z for z in rows]
    design=dict(design,upper_clamp_roll_deg=angle,upper_clamp_roll_is_hardware_installation_orientation_not_motor_or_hand_angle=True)
    design['ports']=[dict(z,Q=(Q@T).tolist()) if z['tag']=='distal_housing_clamp_1' else z for z in design['ports']]
    return rows,design

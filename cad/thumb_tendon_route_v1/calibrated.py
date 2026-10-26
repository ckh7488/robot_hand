"""Motor output rest phase adjustment, with the SAME finger inputs and mounts."""
from follow_parent2 import *
import follow_parent2 as previous
CALIBRATION_DEG=-60.


def coordinated(q,entry_y=-8.,turn_y=5.,rise_y=25.):
    neutral=previous.geometry(np.zeros(20),entry_y,CALIBRATION_DEG,turn_y,rise_y)
    baseline=previous.geometry(q,entry_y,CALIBRATION_DEG,turn_y,rise_y)
    changes=np.array([a['length_mm']-z['length_mm'] for a,z in zip(baseline,neutral)])
    travel=math.degrees((changes[0]-changes[1])/20.);angle=CALIBRATION_DEG+travel
    rows=previous.geometry(q,entry_y,angle,turn_y,rise_y)
    residual=[a['length_mm']-z['length_mm'] for a,z in zip(rows,neutral)];assert abs(residual[0]-residual[1])<1e-7
    return rows,dict(motor_angle_deg=angle,motor_neutral_phase_deg=CALIBRATION_DEG,motor_differential_travel_deg=travel,
        motor_installation_changed=False,hand_angles_changed=False,
        neutral_lengths_mm=[z['length_mm'] for z in neutral],final_lengths_mm=[z['length_mm'] for z in rows],
        common_length_residual_mm=float(np.mean(residual)),differential_residual_mm=float(residual[0]-residual[1]),
        motor_rotation_alone_keeps_both_lengths=bool(max(abs(np.array(residual)))<1e-5),
        tensioner_installed=False,guide_loci_are_installed=False,force_equilibrium_solved=False,
        calibration_meaning='Engineering output-shaft rest phase and separate initial cord length settings,not a finger-angle limit,motor mounting rotation or physical calibration test.')

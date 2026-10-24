"""Frozen-posture gravity budget screen; not physical motor sizing or validation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'sim/remote_drive_v1'))
import model
from evaluate import scenario

assert Path(model.__file__).resolve() == ROOT/'sim/remote_drive_v1/model.py'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report_file = ROOT/'sim/remote_drive_v1/results/report.json'
    nominal = json.loads(report_file.read_text(encoding='utf-8'))
    hashes = dict(nominal['source_hashes'])
    for name in ['tools/screen_opposition_static_load.py', 'sim/remote_drive_v1/results/report.json']:
        hashes[name] = hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
    for name, expected in hashes.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == expected, name
    system = model.build_model('remote20')
    m, d, cfg = system['model'], system['data'], system['config']
    axis = model.NAMES.index('thumb_opposition')
    assert axis == 17 and axis not in system['cable']
    expected_column = np.zeros(20); expected_column[axis] = 1
    assert np.array_equal(system['A'][:, axis], expected_column)
    current_proxy_cap = cfg['stall_torque_Nm']*cfg['current_limit_A']/cfg['stall_current_A']
    available_at_zero_speed = min(cfg['stall_torque_Nm'], current_proxy_cap)
    epsilon = 1e-6

    def posture(q):
        mujoco.mj_resetData(m, d)
        d.qpos[:20] = q
        for motor, index in system['rotor_index'].items():
            d.qpos[index] = (system['A']@q)[motor]
        mujoco.mj_forward(m, d)

    def potential():
        return float(-np.sum(m.body_mass*(d.xipos@m.opt.gravity)))

    unused_time, unused_commands, moves = scenario('rapid_reversal', cfg['dt_s'])
    peak = np.asarray(moves[0]['q1'], dtype=float)
    assert peak[axis] > 0
    angles = np.linspace(0, peak[axis], 10)
    low, high = np.rad2deg(m.jnt_range[axis])
    groups = {
        'isolated_requested_range': [np.eye(20)[axis]*a for a in angles],
        'whole_hand_command_postures': [peak*(a/peak[axis]) for a in angles],
        'isolated_declared_joint_range': [np.eye(20)[axis]*a for a in np.linspace(low, high, 25)],
    }
    rows = []
    for group, poses in groups.items():
        for degrees in poses:
            q = np.deg2rad(degrees)
            posture(q)
            bias = float(d.qfrc_bias[axis])
            direction = np.eye(20)[axis]*epsilon
            posture(q+direction); up = potential()
            posture(q-direction); down = potential()
            gradient = (up-down)/(2*epsilon)
            error = abs(gradient-bias)
            assert error < 1e-8, (group, degrees.tolist(), error)
            rows.append(dict(group=group, joint_angles_deg=degrees.tolist(),
                required_gravity_torque_Nm=bias, potential_gradient_Nm=gradient,
                gradient_difference_Nm=error,
                zero_speed_proxy_margin_Nm=available_at_zero_speed-abs(bias),
                exceeds_proxy_even_with_local_friction_allowance=bool(
                    abs(bias) > available_at_zero_speed+cfg['joint_coulomb_Nm'])))
    summaries = {}
    for group in groups:
        members = [row for row in rows if row['group'] == group]
        worst = max(members, key=lambda row: abs(row['required_gravity_torque_Nm']))
        summaries[group] = dict(samples=len(members),
            maximum_abs_gravity_torque_Nm=abs(worst['required_gravity_torque_Nm']),
            worst_joint_angles_deg=worst['joint_angles_deg'],
            minimum_zero_speed_proxy_margin_Nm=min(row['zero_speed_proxy_margin_Nm'] for row in members),
            samples_exceeding_proxy_even_with_local_friction=sum(
                row['exceeds_proxy_even_with_local_friction_allowance'] for row in members))
    for name, expected in hashes.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == expected, name
    result = dict(schema_version=1, status='Uncalibrated static budget screen; no physical motor or latency pass',
        source_hashes=hashes, input_hash_guard_passed=True, engine='MuJoCo '+mujoco.__version__,
        axis=axis, joint=model.NAMES[axis], joint_names=model.NAMES,
        posture_rule='10 uniformly spaced opposition angles across the rapid-reversal request, isolated and whole-hand request postures; 25 isolated angles across the declared joint range',
        parameters={key:cfg[key] for key in ['gravity_m_s2','stall_torque_Nm','stall_current_A','current_limit_A','joint_coulomb_Nm']},
        current_proxy_cap_Nm=current_proxy_cap, zero_speed_electrical_envelope_Nm=available_at_zero_speed,
        potential_gradient_epsilon_rad=epsilon,
        maximum_gradient_difference_Nm=max(row['gradient_difference_Nm'] for row in rows),
        summaries=summaries, samples=rows,
        limitations=[
            'All joints are held at declared postures by hypothetical external holding torques; this is not the actual PD/cable equilibrium.',
            'The opposition transmission column is direct drive; other motor rows do not supply its generalized holding torque in this model.',
            'Manufacturer supply current is used only as the existing uncalibrated torque proxy, not measured phase current or a continuous torque rating.',
            'Local modeled Coulomb friction is a permissive static allowance, not verified mechanical holding capability.',
            'No acceleration, speed-dependent torque loss, PWM rise, thermal derating, supply sag or contact load is included.',
            'Sampled postures do not establish continuous extrema, full-range collision clearance, or robot-side 50/100ms performance.',
        ])
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text+'\n', encoding='utf-8')
    else:
        print(text)


if __name__ == '__main__':
    main()

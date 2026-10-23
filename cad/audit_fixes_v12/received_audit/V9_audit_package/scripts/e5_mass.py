"""E5: independent mass / gravity / inertia / speed re-computation (my code, their density assumptions unless stated).
Mass: mesh volume x density (printed & vendor frames 1.25, fasteners 7.85 g/cm3), XL330 18 g (rotor inside), AK masses
read from mass_screen.json (READ, not verified). Inertia: full mesh inertia tensors (not point masses)."""
import sys, json, math, itertools, numpy as np, trimesh
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); D = sys.argv[1]; OUT = sys.argv[3]
ms = {r['name']: r for r in json.load(open(D + '/mass_screen.json'))['items']}
z = np.load(D + '/collision_meshes.npz'); props = {}
for a in g.models:
    m = trimesh.Trimesh(z[a + '_vertices'], z[a + '_faces'], process=True)
    if not m.is_watertight: trimesh.repair.fix_normals(m)
    m.density = 1.0; props[a] = (m.volume, m.center_mass.copy(), m.moment_inertia.copy())  # unit density, mm
parts = []
for i, p in enumerate(g.meta):
    r = p['role']; a = p['asset']; vol, com, I1 = props[a]
    if r == 'vendor_rotor': continue
    if r == 'vendor_motor': m = 0.018
    elif r == 'arm_motor_nominal': m = ms[p['name']]['mass_kg']
    elif r == 'fastener': m = vol * 7.85e-6
    else: m = vol * 1.25e-6
    scale = m / vol  # kg per mm3 -> inertia kg mm2
    parts.append(dict(i=i, name=p['name'], role=r, m=m, com=com, I=I1 * scale))
hand = [q for q in parts if not q['role'].startswith('arm_')]
cat = {}
for q in hand: cat[q['role']] = cat.get(q['role'], 0) + q['m']
out = {'hand_mass_kg': sum(q['m'] for q in hand), 'hand_mass_by_role_kg': cat}
print(out)
A = g.C['arm']; wy = -A['wrist_to_palm'] - A['wrist_axis_offset']; ey = wy - A['elbow_to_wrist_flex']
axes = {'elbow_flex': ((0, ey, 0), (1, 0, 0), {'elbow_link', 'forearm_link', 'wrist_link', 'hand'}),
        'forearm_roll': ((0, ey + A['elbow_to_roll'], 0), (0, 1, 0), {'forearm_link', 'wrist_link', 'hand'}),
        'wrist_flex': ((0, wy, 0), (1, 0, 0), {'wrist_link', 'hand'}),
        'wrist_deviation': ((A['wrist_lateral_offset'], -A['wrist_to_palm'], 0), (0, 0, 1), {'hand'})}
def world(qs, M, ac):
    pass
def axis_world(name, arm):
    _, ac = g.fk(g.C['poses']['Open'], arm)
    up = {'elbow_flex': np.eye(4), 'forearm_roll': ac['elbow_link'], 'wrist_flex': ac['forearm_link'], 'wrist_deviation': ac['wrist_link']}[name]
    p, a, _ = axes[name]; return up[:3, :3] @ np.array(p, float) + up[:3, 3], up[:3, :3] @ np.array(a, float)
def grav_and_J(pose, arm):
    M = g.place(pose, arm); res = {}
    for jn, (_, _, links) in axes.items():
        p, a = axis_world(jn, arm); S = np.zeros(3); J = 0.
        for q in parts:
            if g.armlink[q['i']] not in links: continue
            R = M[q['i'], :3, :3]; c = R @ q['com'] + M[q['i'], :3, 3]; r = (c - p) / 1000
            S += q['m'] * r
            Ic = R @ q['I'] @ R.T / 1e6; rp = r - (r @ a) * a
            J += a @ Ic @ a + q['m'] * (rp @ rp)
        res[jn] = (9.81 * np.linalg.norm(np.cross(a, S)), J)
    return res
best = {k: (0, None) for k in axes}; Jmax = {k: 0 for k in axes}
for ps in g.poses:
    for roll, pitch, yaw in itertools.product(range(-80, 81, 20), range(-35, 56, 5), range(-20, 21, 5)):
        r = grav_and_J(g.C['poses'][ps], (0, roll, pitch, yaw))
        for k, (t, J) in r.items():
            if t > best[k][0]: best[k] = (t, (ps, roll, pitch, yaw))
            Jmax[k] = max(Jmax[k], J)
out['gravity_max_Nm'] = {k: [round(v[0], 3), v[1]] for k, v in best.items()}
out['inertia_max_kgm2'] = {k: round(v, 4) for k, v in Jmax.items()}
print(out['gravity_max_Nm']); print(out['inertia_max_kgm2'])
# their grid (roll +-60) for comparison
bt = {k: 0 for k in axes}
for ps in g.poses:
    for roll, pitch, yaw in itertools.product([-60, 0, 60], range(-35, 56, 5), range(-20, 21, 5)):
        for k, (t, J) in grav_and_J(g.C['poses'][ps], (0, roll, pitch, yaw)).items(): bt[k] = max(bt[k], t)
out['gravity_max_their_grid_Nm'] = {k: round(v, 3) for k, v in bt.items()}; print('their grid', out['gravity_max_their_grid_Nm'])
# ---------------- middle MCP speed bound ----------------
M = g.place(g.C['poses']['Open']); fr, _ = g.fk(g.C['poses']['Open'])
F = fr['middle_1']; p = F[:3, 3]; a = F[:3, 0]
Jp = Jf = 0.; S = np.zeros(3)
for q in parts:
    b = g.body[q['i']]
    if b not in ('middle_1', 'middle_2', 'middle_3') or q['role'] == 'vendor_rotor': continue
    R = M[q['i'], :3, :3]; c = R @ q['com'] + M[q['i'], :3, 3]; r = (c - p) / 1000; rp = r - (r @ a) * a
    Jp += q['m'] * rp @ rp; Jf += a @ (R @ q['I'] @ R.T / 1e6) @ a + q['m'] * rp @ rp; S += q['m'] * r
G = 9.81 * np.linalg.norm(np.cross(a, S))
def tmin(J, Gt, tau_stall, omega0, delta=math.pi / 2, tau_cap=None):
    u = np.linspace(0, 1, 2001); hd = 30 * u**2 - 60 * u**3 + 30 * u**4; hdd = 60 * u - 180 * u**2 + 120 * u**3
    def ok(T):
        v = delta * hd / T; acc = delta * hdd / T**2; av = tau_stall * np.maximum(0, 1 - abs(v) / omega0)
        if tau_cap is not None: av = np.minimum(av, tau_cap)
        return np.all(abs(J * acc) + Gt <= av)
    lo, hi = 0.01, 5.
    if not ok(hi): return None
    for _ in range(50):
        mid = (lo + hi) / 2; lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi
w0 = 383 * 2 * math.pi / 60; arm = 2e-5  # armature = CAD agent's earlier isaac_actuation value (reflected rotor), READ
sc = {'their: point mass, stall 0.215': (Jp, 0.215, None), 'mesh inertia': (Jf, 0.215, None), '+ reflected rotor 2e-5': (Jf + arm, 0.215, None),
      '+ 0.7A current cap (0.102 N.m)': (Jf + arm, 0.215, 0.215 * 0.7 / 1.47)}
out['middle_mcp'] = {'J_point_kgm2': Jp, 'J_mesh_kgm2': Jf, 'gravity_worst_Nm': G, 'rows': {}}
for k, (J, ts, cap) in sc.items():
    out['middle_mcp']['rows'][k] = {'no_gravity_s': tmin(J, 0, ts, w0, tau_cap=cap), 'worst_gravity_s': tmin(J, G, ts, w0, tau_cap=cap)}
print(json.dumps(out['middle_mcp'], indent=1))
# ---------------- elbow dynamic need ----------------
Je = out['inertia_max_kgm2']['elbow_flex']
for T in (0.4, 0.6, 1.0):
    amax = 5.7735 * (math.pi / 2) / T**2; vmax = 1.875 * (math.pi / 2) / T
    out.setdefault('elbow_90deg_minjerk', []).append({'T_s': T, 'peak_acc_rad_s2': round(amax, 1), 'peak_vel_rad_s': round(vmax, 2),
        'inertial_Nm': round(Je * amax, 2), 'gravity_plus_inertial_upper_Nm': round(Je * amax + best['elbow_flex'][0], 2)})
print(out['elbow_90deg_minjerk'])
json.dump(out, open(OUT, 'w'), indent=1, default=float)

"""Claude's independent V9 geometry engine (no imports from the submitted code).
- Placements for the 6 named poses come from scene.json (FreeCAD export).
- For other poses: my own FK from design.json; each motion cluster (derived from scene.json motion,
  not names) is attached to the FK frame that reproduces all 6 exported poses (validated, error printed).
Units mm/deg."""
import json, collections, numpy as np, fcl

def rot(axis, deg):
    a = np.deg2rad(deg); c, s = np.cos(a), np.sin(a)
    return [np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])][axis]
def T(p=(0, 0, 0), r=None):
    m = np.eye(4); m[:3, 3] = p
    if r is not None: m[:3, :3] = r
    return m
def about(p, axis, deg): return T(p, rot(axis, deg)) @ T(-np.asarray(p, float))

class Geo:
    def __init__(self, D):
        self.D = D
        self.C = json.load(open(D + '/design.json')); self.S = json.load(open(D + '/scene.json'))
        self.poses = list(self.S); self.meta = self.S['Open']; self.N = N = len(self.meta)
        self.names = [p['name'] for p in self.meta]; self.idx = {n: i for i, n in enumerate(self.names)}
        self.Tw = {}
        for ps in self.poses:
            a = np.zeros((N, 4, 4)); a[:, 3, 3] = 1
            for i, p in enumerate(self.S[ps]): a[i, :3, :3] = p['R']; a[i, :3, 3] = p['pos_mm']
            self.Tw[ps] = a
        # motion clusters
        par = list(range(N))
        def f(x):
            while par[x] != x: par[x] = par[par[x]]; x = par[x]
            return x
        inv = {ps: np.linalg.inv(self.Tw[ps]) for ps in self.poses}
        for a in range(N):
            for b in range(a + 1, N):
                if f(a) == f(b): continue
                r0 = inv['Open'][a] @ self.Tw['Open'][b]
                if all(np.abs(inv[ps][a] @ self.Tw[ps][b] - r0).max() < 1e-6 for ps in self.poses[1:]): par[f(a)] = f(b)
        roots = sorted({f(i) for i in range(N)}); self.cl = np.array([roots.index(f(i)) for i in range(N)])
        # meshes
        z = np.load(D + '/collision_meshes.npz'); self.models = {}; self.corners = {}; self.verts = {}
        for a in sorted({p['asset'] for p in self.meta}):
            v = z[a + '_vertices']; fc = z[a + '_faces']
            b = fcl.BVHModel(); b.beginModel(len(v), len(fc)); b.addSubModel(v, fc); b.endModel(); self.models[a] = b
            lo, hi = v.min(0), v.max(0); self.verts[a] = v
            self.corners[a] = np.array([[x, y, w] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for w in (lo[2], hi[2])])
        self.objs = [fcl.CollisionObject(self.models[p['asset']]) for p in self.meta]
        self.req = fcl.DistanceRequest(enable_nearest_points=True)
        self._map_clusters()

    # ---------------- my FK (design.json) ----------------
    def fk(self, pose, arm=(0, 0, 0, 0)):
        H = self.C['hand']; A_ = self.C['arm']; h = H['root_axis_offset_z']; fr = {'base': np.eye(4)}
        for f in H['fingers']:
            n = f['name']; rx, ry, rz = f['base_rpy']; q = pose[n]
            A = T(f['root'], rot(2, rz) @ rot(1, ry) @ rot(0, rx) @ rot(1, -90))
            O = A @ T(r=rot(0, -q[0])); fr[n + '_0'] = O
            A = O @ T(r=rot(1, 90)) @ T((0, 0, h))
            for j in range(3):
                O = A @ T(r=rot(0, q[j + 1])); fr[n + '_' + str(j + 1)] = O
                if j < 2:
                    A = O @ T((0, f['links'][j], 0))
                    if n == 'thumb' and j == 0: A = A @ T(r=rot(2, f['mcp_mount_yaw']))
        wy = -A_['wrist_to_palm'] - A_['wrist_axis_offset']; ey = wy - A_['elbow_to_wrist_flex']
        E = about((0, ey, 0), 0, arm[0]); F = about((0, ey + A_['elbow_to_roll'], 0), 1, arm[1])
        P = about((0, wy, 0), 0, arm[2]); Y = about((A_['wrist_lateral_offset'], -A_['wrist_to_palm'], 0), 2, arm[3])
        arm_chain = {'fixed': np.eye(4), 'elbow_link': E, 'forearm_link': E @ F, 'wrist_link': E @ F @ P, 'hand': E @ F @ P @ Y}
        return fr, arm_chain

    def _map_clusters(self):
        nc = self.cl.max() + 1; self.cframe = [None] * nc; err = 0.
        frs = {ps: self.fk(self.C['poses'][ps])[0] for ps in self.poses}
        for c in range(nc):
            i = int(np.where(self.cl == c)[0][0]); best = None
            for k in frs['Open']:
                rel = np.linalg.inv(frs['Open'][k]) @ self.Tw['Open'][i]
                e = max(np.abs(frs[ps][k] @ rel - self.Tw[ps][i]).max() for ps in self.poses)
                if best is None or e < best[0]: best = (e, k)
            self.cframe[c] = best[1]; err = max(err, best[0])
        assert err < 1e-6, err
        self.fk_err = err
        # arm links: arm geometry never moves in the 6 poses, so attach by the documented chain (names, 10 parts)
        AL = {'AK_elbow_flex': 'fixed', 'Elbow_fixed_cradle': 'fixed', 'Elbow_roll_carrier': 'elbow_link', 'AK_forearm_roll': 'elbow_link',
              'Roll_output_flange': 'forearm_link', 'Forearm_box_beam': 'forearm_link', 'AK_wrist_flex': 'forearm_link',
              'Wrist_box_carrier': 'wrist_link', 'AK_wrist_deviation': 'wrist_link'}
        self.armlink = [AL.get(n, 'hand') for n in self.names]
        self.body = [self.cframe[self.cl[i]] if self.armlink[i] == 'hand' else 'arm:' + self.armlink[i] for i in range(self.N)]
        frO, _ = self.fk(self.C['poses']['Open']); self.local = np.array(
            [np.linalg.inv(frO[self.cframe[self.cl[i]]]) @ self.Tw['Open'][i] for i in range(self.N)])

    def place(self, pose, arm=(0, 0, 0, 0)):
        fr, ac = self.fk(pose, arm); M = np.zeros((self.N, 4, 4))
        for i in range(self.N): M[i] = ac[self.armlink[i]] @ fr[self.cframe[self.cl[i]]] @ self.local[i]
        return M

    def set(self, M):
        lo = np.zeros((self.N, 3)); hi = np.zeros((self.N, 3))
        for i, p in enumerate(self.meta):
            self.objs[i].setTransform(fcl.Transform(M[i, :3, :3], M[i, :3, 3]))
            c = self.corners[p['asset']] @ M[i, :3, :3].T + M[i, :3, 3]; lo[i] = c.min(0); hi[i] = c.max(0)
        self.lo, self.hi = lo, hi

    def dist(self, i, j):
        r = fcl.DistanceResult(); d = float(fcl.distance(self.objs[i], self.objs[j], self.req, r))
        if d <= 1e-7:
            d = 0.0 if fcl.collide(self.objs[i], self.objs[j], fcl.CollisionRequest(), fcl.CollisionResult()) else max(d, 0)
        return d, (None if r.nearest_points is None else [np.asarray(x) for x in r.nearest_points])

    def near(self, pairs, cap):
        P = np.asarray(pairs); ii, jj = P[:, 0], P[:, 1]
        g = np.maximum(0, np.maximum(self.lo[ii] - self.hi[jj], self.lo[jj] - self.hi[ii]))
        return P[np.einsum('ij,ij->i', g, g) < cap * cap]

    def moving_pairs(self, filt=None):
        out = []
        for i in range(self.N):
            for j in range(i + 1, self.N):
                if self.body[i] == self.body[j]: continue
                if filt and not filt(i, j): continue
                out.append((i, j))
        return out

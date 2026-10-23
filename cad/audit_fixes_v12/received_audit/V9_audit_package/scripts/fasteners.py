"""Independent screw/nut engagement from the tessellated CAD (collision_meshes.npz, mm).
For each screw component: axis by PCA, head end = end with larger radius; tip = other end.
Matching nut = component of paired nut asset whose centre lies on the screw axis (<0.3 mm)."""
import sys, json, numpy as np, trimesh
D = sys.argv[1]; z = np.load(D + '/collision_meshes.npz')
S = json.load(open(D + '/scene.json'))['Open']
def comps(a):
    m = trimesh.Trimesh(z[a + '_vertices'], z[a + '_faces'], process=True); return m.split(only_watertight=False)
def screw_geom(c):
    v = c.vertices; ctr = v.mean(0); _, _, vt = np.linalg.svd(v - ctr, full_matrices=False); ax = vt[0]
    t = (v - ctr) @ ax; rad = np.linalg.norm((v - ctr) - np.outer(t, ax), axis=1)
    lo, hi = t.min(), t.max(); band = 0.3
    r_lo = rad[t < lo + band].max(); r_hi = rad[t > hi - band].max()
    if r_lo > r_hi: ax = -ax; t = -t; lo, hi = -hi, -lo; r_lo, r_hi = r_hi, r_lo
    # now head at hi end (larger radius); tip at lo end
    shank_r = np.median(rad[(t > lo + 1) & (t < lo + 3)]) if np.any((t > lo + 1) & (t < lo + 3)) else None
    head_len = (hi - t[rad > (shank_r or 0) * 1.3].min()) if shank_r else None
    return dict(ctr=ctr, ax=ax, tip=ctr + ax * lo, head_top=ctr + ax * hi, L=hi - lo, head_d=2 * r_hi, shank_d=2 * shank_r if shank_r else None,
                head_len=head_len)
pairs = [('M3_root_mount_screws', 'M3_root_mount_nuts'), ('M3_thumb_mount_screws', 'M3_thumb_mount_nuts'), ('M3_cover_screws', 'M3_cover_nuts'),
         ('M3_saddle_mount_screws', 'M3_saddle_mount_nuts'), ('M2_base_screws', 'M2_base_nuts'), ('M2_output_screws', 'M2_output_nuts'),
         ('M2_output_screws', 'M2_root_output_nuts')]
out = []
for sa, na in pairs:
    nuts = comps(na)
    for k, c in enumerate(comps(sa)):
        g = screw_geom(c); best = None
        for n in nuts:
            nc = n.vertices.mean(0); off = nc - g['tip']; along = off @ g['ax']; perp = np.linalg.norm(off - along * g['ax'])
            if perp < 0.3 and (best is None or abs(along) < abs(best[0])): best = (along, n)
        row = {'screw_asset': sa, 'nut_asset': na, 'k': k, 'length_total_mm': round(g['L'], 2), 'head_d': round(g['head_d'], 2),
               'shank_d': None if g['shank_d'] is None else round(g['shank_d'], 2), 'head_len': None if g['head_len'] is None else round(g['head_len'], 2)}
        if best is None: row['nut'] = None
        else:
            n = best[1]; tn = (n.vertices - g['tip']) @ g['ax']  # along axis, 0 = tip, + toward head
            nut_lo, nut_hi = tn.min(), tn.max()  # nut span relative to tip
            row.update(nut_h=round(nut_hi - nut_lo, 2), tip_beyond_nut_far_face_mm=round(-nut_lo, 2),
                       engaged_len_mm=round(max(0, nut_hi - max(nut_lo, 0)), 2))
        out.append(row)
seen = set()
for r in out:
    key = (r['screw_asset'], r['nut_asset'], r.get('tip_beyond_nut_far_face_mm'), r.get('engaged_len_mm'))
    if key in seen or r.get('nut_h') is None and r['nut_asset'] == 'M2_root_output_nuts': continue
    seen.add(key); print(r)
json.dump(out, open(sys.argv[2], 'w'), indent=1)

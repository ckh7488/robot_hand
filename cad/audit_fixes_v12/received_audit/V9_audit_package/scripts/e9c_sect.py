"""E9b: nut pocket wall by planar section at nut mid-height: wall = min distance from the nut hexagon (projected)
to any section boundary other than the pocket it sits in (i.e. remaining printed material between pocket and outside)."""
import sys, json, numpy as np, trimesh
from shapely.geometry import Polygon, MultiPolygon, Point
from shapely.ops import unary_union
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); z = np.load(sys.argv[1] + '/collision_meshes.npz'); M = g.Tw['Open']
def mesh(i):
    a = g.meta[i]['asset']; m = trimesh.Trimesh(z[a + '_vertices'], z[a + '_faces'], process=True); m.apply_transform(M[i]); return m
cases = [('index_mcp_base_nuts', 'index_spread_carrier'), ('index_pip_base_nuts', 'index_mcp_link'), ('index_root_base_nuts', 'Index_root_cartridge'),
         ('index_mcp_output_nuts', 'index_mcp_link'), ('index_root_output_nuts', 'index_spread_carrier'), ('Index_root_mount_nuts', 'Index_root_cartridge'),
         ('Palm_thumb_mount_nuts', 'Thumb_internal_cradle'), ('Palm_saddle_mount_nuts', 'Palm_back_chassis'), ('Palm_cover_nuts', 'Palm_back_chassis'),
         ('thumb_root_base_nuts', 'Thumb_internal_cradle')]
out = []
for nut, host in cases:
    hm = mesh(g.idx[host])
    for k, c in enumerate(mesh(g.idx[nut]).split(only_watertight=False)):
        ctr = c.vertices.mean(0); _, _, vt = np.linalg.svd(c.vertices - ctr); ax = vt[2]
        sec = hm.section(plane_origin=ctr, plane_normal=ax)
        if sec is None: out.append({'nut': nut, 'k': k, 'host': host, 'wall_mm': None, 'note': 'no section'}); continue
        p2, T = sec.to_planar()
        nut2 = trimesh.transform_points(c.vertices, np.linalg.inv(T))[:, :2]
        hexp = Polygon(nut2).convex_hull
        mat = unary_union([Polygon(p) for p in p2.polygons_full]) if p2.polygons_full else None
        if mat is None or mat.is_empty: out.append({'nut': nut, 'k': k, 'host': host, 'wall_mm': None, 'note': 'empty'}); continue
        from shapely.geometry import LinearRing
        cpt = Point(hexp.centroid); res = None
        geoms = list(mat.geoms) if hasattr(mat, 'geoms') else [mat]
        for poly in geoms:
            for hole in poly.interiors:
                hp = Polygon(hole)
                if hp.contains(cpt):
                    others = [LinearRing(poly.exterior.coords)] + [LinearRing(h.coords) for h in poly.interiors if h is not hole]
                    res = ('closed', round(min(LinearRing(hole.coords).distance(o) for o in others), 2), round(hp.area - hexp.area, 2))
        if res is None:
            # side-open: wall = min distance from hex to the material boundary beyond the touching face, approximated by
            # distance from hex to exterior of the nearest material polygon measured on the far side
            near = min(geoms, key=lambda p: p.distance(hexp))
            res = ('side-open', None, None)
        wall = res
        b = hexp.buffer(0.4).difference(hexp.buffer(0.05)); openfrac = 1 - b.intersection(mat).area / b.area
        out.append({'nut': nut, 'k': k, 'host': host, 'pocket': wall[0], 'min_wall_mm': wall[1], 'pocket_minus_nut_area_mm2': wall[2], 'pocket_open_fraction_at_0.4mm': round(openfrac, 2)})
        print(out[-1])
json.dump(out, open(sys.argv[3], 'w'), indent=1)

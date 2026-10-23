"""Read-only checkpoint integrity, FK and geometry screen; never a hardware release."""
import argparse, hashlib, importlib, json, sys, math
from pathlib import Path
import numpy as np
import trimesh
from scipy.integrate import quad
from scipy.special import comb
P = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument('--output', type=Path)
ap.add_argument('--manifest', type=Path)
a = ap.parse_args()
current = json.loads((P/'CHECKPOINT.json').read_text(encoding='utf-8'))
manifest_path = a.manifest or P/current['manifest']
manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
for relative, expected in manifest['files'].items():
    f = (P/relative).resolve()
    assert f.is_relative_to(P), relative
    assert f.is_file(), 'Missing '+relative
    assert hashlib.sha256(f.read_bytes()).hexdigest() == expected['sha256'], 'Changed '+relative
frozen = json.loads((P/'AUDIT_EVIDENCE_INDEX.json').read_text(encoding='utf-8'))
n = 0
for group in ['geometry_sha256','code_sha256','report_sha256']:
    for relative, expected in frozen[group].items():
        assert hashlib.sha256((P/relative).read_bytes()).hexdigest() == expected, relative
        n += 1
B = P/'cad'/'modular_roots_v9'
sys.path.insert(0, str(B))
k = importlib.import_module('kinematics_numpy')
assert Path(k.__file__).resolve().is_relative_to(P), 'Import escaped clean checkout'
fk = k.verify()
meshes = []
for relative in [
    'cad/audit_fixes_v12/parts/Thumb_metacarpal_tool_access.stl',
    'cad/service_loops_v11/parts/Little_root_cable_saddle.stl',
    'cad/service_loops_v11/parts/Little_MCP_link_cable_saddle.stl',
    *(['cad/assembly_fixes_v13/parts/Palm_front_cover_recessed.stl'] if current['candidate']=='cad/assembly_fixes_v13' else []),
]:
    m = trimesh.load_mesh(P/relative)
    components = len(m.split(only_watertight=False))
    assert m.is_watertight and m.is_winding_consistent and components == 1, relative
    meshes.append({'file': relative, 'watertight': True, 'components': components, 'volume_mm3': float(m.volume)})
curves = json.loads((P/'cad/service_loops_v11/reviews/service_loop_curves.json').read_text(encoding='utf-8'))['chosen']
def bernstein(t,n):
    return np.array([comb(n,i)*t**i*(1-t)**(n-i) for i in range(n+1)]).T
err = 0.
radius = math.inf
t = np.linspace(0.,1.,10001)
for row in curves['curves']:
    control = np.asarray(row['controls_mm'])
    dp = 5*np.diff(control,axis=0)
    length = quad(lambda u: np.linalg.norm(bernstein(np.array([u]),4)[0]@dp),0,1,epsabs=1e-9,limit=100)[0]
    err = max(err,abs(length-curves['length_mm']))
    v = bernstein(t,4)@dp
    acc = bernstein(t,3)@(20*np.diff(control,n=2,axis=0))
    curvature = np.linalg.norm(np.cross(v,acc),axis=1)/np.linalg.norm(v,axis=1)**3
    radius = min(radius,float(1/curvature.max()))
assert len(curves['curves']) == 123
assert err < 1e-3 and radius >= 10.
assert not current['manufacturing_release']
result = {'checkpoint': current['id'], 'manifest_files_checked': len(manifest['files']), 'V9_frozen_hashes_checked': n,
          'FK': fk, 'local_import': str(Path(k.__file__).relative_to(P)), 'meshes': meshes,
          'loop_curves_recomputed': len(curves['curves']), 'loop_max_length_error_mm': err,
          'loop_min_dense_sampled_radius_mm': radius, 'manufacturing_release': False,
          'scope': 'Read-only hashes, fixed-pose FK, watertight candidate meshes and geometric curve quadrature. No global collision, cable dynamics, FDM strength or motor-performance approval.',
          'environment': {'python': sys.version.split()[0], 'numpy': np.__version__, 'trimesh': trimesh.__version__}}
text = json.dumps(result,indent=2)
if a.output:
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(text,encoding='utf-8')
print(text)

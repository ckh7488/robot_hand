import sys, json, hashlib, os, glob
R = sys.argv[1]; D = R + '/cad/modular_roots_v9'; B = R + '/cad/modular_roots_v8'
h = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
I = json.load(open(R + '/AUDIT_EVIDENCE_INDEX.json', encoding='utf-8'))
for grp in ['geometry_sha256', 'code_sha256', 'report_sha256']:
    bad = [n for n, w in I[grp].items() if not os.path.exists(os.path.join(R, n)) or h(os.path.join(R, n)) != w]
    print('%-16s listed %3d  mismatched/missing %d %s' % (grp, len(I[grp]), len(bad), bad[:5]))
G = json.load(open(D + '/geometry_review.json', encoding='utf-8'))
bad = [n for n, w in G['cad_inputs_sha256'].items() if h(os.path.join(D, n)) != w]
print('geometry_review cad_inputs:', len(G['cad_inputs_sha256']), 'mismatch', bad)
base = json.load(open(B + '/reviews/mesh_transitions.json', encoding='utf-8'))
bad = [n for n, w in base['input_sha256'].items() if h(os.path.join(B, n)) != w]
print('V8 transitions inputs:', len(base['input_sha256']), 'mismatch', bad, '| items', len(base['items']), '| passed', base.get('passed_at_threshold'))
# which files in the zip are NOT covered by any hash list
listed = set(I['geometry_sha256']) | set(I['code_sha256']) | set(I['report_sha256'])
allf = [os.path.relpath(p, R).replace('\\', '/') for p in glob.glob(R + '/**/*', recursive=True) if os.path.isfile(p)]
un = [f for f in allf if f not in listed]
print('files in zip:', len(allf), '| not hash-listed:', len(un)); print('  e.g.', [f for f in un if f.endswith(('.py', '.step', '.FCStd', '.json'))][:40])
# V8 vs V9 parts STEP byte identity
v8 = {os.path.basename(p): h(p) for p in glob.glob(B + '/parts/*.step')}; v9 = {os.path.basename(p): h(p) for p in glob.glob(D + '/parts/*.step')}
same = [k for k in v9 if k in v8 and v8[k] == v9[k]]; diff = [k for k in v9 if k in v8 and v8[k] != v9[k]]; new = [k for k in v9 if k not in v8]; gone = [k for k in v8 if k not in v9]
print('parts STEP: V9 %d, identical %d, changed %s, new %s, removed %s' % (len(v9), len(same), diff, new, gone))
d8 = json.load(open(B + '/design.json', encoding='utf-8')); d9 = json.load(open(D + '/design.json', encoding='utf-8'))
print('design.json keys equal?', {k: d8.get(k) == d9.get(k) for k in sorted(set(d8) | set(d9))})
print('scene.json identical bytes?', h(B + '/scene.json') == h(D + '/scene.json'))
print('collision_meshes.npz identical?', h(B + '/collision_meshes.npz') == h(D + '/collision_meshes.npz'))

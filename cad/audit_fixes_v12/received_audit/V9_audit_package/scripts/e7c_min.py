import sys, json, numpy as np
exec(open(sys.argv[2] + '/e7_tools.py').read().split("names = g.names")[0])
segs[:] = [('head_pass', 1.6, 0.05, 8), ('shaft', 1.4, 8, 60), ('handle', 8, 60, 140)]
names = g.names; out = []
def mating_of(sc):
    return [m for m in g.meta[g.idx[sc]].get('mating_with', []) if m in g.idx]
for f in ['index', 'middle', 'ring', 'little', 'thumb']:
    for sc in [n for n in names if n.startswith(f + '_') and (n.endswith('_base_screws') or n.endswith('_output_screws'))]:
        st = [g.idx[m] for m in mating_of(sc)]
        for r in check(sc, st):
            r['stage'] = 'minimal: ' + ','.join(mating_of(sc)); out.append(r)
json.dump(out, open(sys.argv[3], 'w'), indent=1, default=str)
import collections
s = collections.defaultdict(list)
for r in out: s[(r['screw'], r['stage'])].append(r)
for (sc, st), rr in s.items():
    blk = [(r['k'], r['head_pass'] if r['head_pass'][0] <= 0 else r['shaft']) for r in rr if min(r['head_pass'][0], r['shaft'][0]) <= 0]
    print(sc.ljust(26), st[:85].ljust(85), 'blocked', len(blk), blk[:2], 'min', min(min(r['head_pass'][0], r['shaft'][0]) for r in rr))

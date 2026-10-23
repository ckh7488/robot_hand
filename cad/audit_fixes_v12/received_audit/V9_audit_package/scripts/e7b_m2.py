import sys, json, numpy as np
sys.argv = [sys.argv[0], sys.argv[1], sys.argv[2], sys.argv[3]]
exec(open(sys.argv[2] + '/e7_tools.py').read().split("names = g.names")[0])
segs[:] = [('head_pass', 1.6, 0.05, 8), ('shaft', 1.4, 8, 60), ('handle', 8, 60, 140)]
names = g.names; out = []
for f in ['index', 'thumb']:
    F = f.capitalize()
    mod = [i for i, n in enumerate(names) if n.startswith(f + '_') or n.startswith(F + '_root_')]
    if f == 'thumb': mod += [g.idx['Thumb_internal_cradle']]
    printed_frames = [i for i in mod if g.meta[i]['role'] in ('printed', 'vendor_frame_modified') or names[i].endswith('_nuts')]
    for sc in [n for n in names if n.startswith(f + '_') and (n.endswith('_base_screws') or n.endswith('_output_screws'))]:
        for st, parts in [('frames+printed only', printed_frames), ('full module', mod), ('full hand', list(range(g.N)))]:
            for r in check(sc, parts): out.append(dict(r, stage=st))
json.dump(out, open(sys.argv[3], 'w'), indent=1, default=str)
import collections
summ = collections.defaultdict(list)
for r in out: summ[(r['screw'], r['stage'])].append(min(r['head_pass'][0], r['shaft'][0]))
for k, v in summ.items():
    blk = [r for r in out if (r['screw'], r['stage']) == k and min(r['head_pass'][0], r['shaft'][0]) <= 0]
    print(k[0].ljust(28), k[1].ljust(20), 'blocked', len(blk), '/', len(v), 'min', min(v), [ (b['k'], b['head_pass'][1] if b['head_pass'][0]<=0 else b['shaft'][1]) for b in blk][:4])

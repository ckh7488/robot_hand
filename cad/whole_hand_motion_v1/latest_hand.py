"""Extend all discovery poses to the later 364-part whole-hand drive candidate.

Reuse only pair outcomes whose two saved STEP hashes, bodies and FK are unchanged.
Every pair involving either a modified or added part is newly examined.
"""
import argparse,copy,hashlib,importlib.util,json,math,time
from pathlib import Path
import numpy as np
import cadquery as cq

HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def mod(name,p):
    s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--select',default='');a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=HERE.parents[1];candidate=root/'cad/thumb_ip_drive_v1';hashes={}
    def bind(p,expected=None):
        p=Path(p).resolve();h=sha(p)
        if expected and h!=expected:raise ValueError('Changed source '+str(p))
        hashes[p.relative_to(root).as_posix()]=h;return p
    bind(__file__);base=read(bind(HERE/'reviews/discovery.json'));trans=read(bind(HERE/'reviews/transitions.json'))
    if not base['completed'] or not trans['completed'] or not base['input_hash_guard']['passed'] or not trans['input_hash_guard']['passed']:raise ValueError('Incomplete baseline')
    for k,h in base['source_sha256'].items():bind(root/k,h)
    oldscene=read(root/'cad/manufacturing_v2/generated/scene.json');old={r['name']:r for r in oldscene['hand']}
    scene=read(bind(candidate/'generated/scene.json'));rows={r['name']:r for r in scene['hand']}
    build=read(bind(candidate/'generated/build_report.json'))
    if not build['completed'] or build['errors'] or not build['input_hash_guard']['passed']:raise ValueError('Invalid drive build')
    for k,h in build['source_sha256'].items():
        if k.endswith('generated/design.json') or k.endswith('generated/scene.json'):bind(root/k,h)
    modified=set(build['modified_names']);added=set(build['added_names']);changed=modified|added;unchanged=set(old)-modified
    if len(old)!=359 or len(rows)!=364 or len(modified)!=2 or len(added)!=5 or set(rows)!=set(old)|added or len(unchanged)!=357:raise ValueError('Wrong part partition')
    for name in unchanged:
        for k in ['sha256_step','sha256_stl','body','volume_mm3','bbox_mm']:
            if rows[name][k]!=old[name][k]:raise ValueError('Part not invariant '+name+'/'+k)
    for name in modified:
        if rows[name]['body']!=old[name]['body']:raise ValueError('Changed body')
    native=mod('_latest_native',bind(root/'cad/gesture_hand_v3/audit_gestures.py'));v=native.load_v2(root);design=base['requests']['design'];neutral=v.frames(design,np.zeros(20));parts={}
    for row in scene['hand']:
        for k,hkey in [('step','sha256_step'),('mesh','sha256_stl')]:
            p=(root/row[k]) if row['location']=='repo' else candidate/'generated'/row[k];bind(p,row[hkey])
            if k=='step':
                s=cq.importers.importStep(str(p)).val()
                if not s.isValid() or len(s.Solids())!=1 or abs(s.Volume()-row['volume_mm3'])>1e-3:raise ValueError('Invalid STEP '+row['name'])
                parts[row['name']]=s
    chains=native.body_chains(v.joint_names());names=list(parts);bodies={n:rows[n]['body'] for n in names}
    pairs=[(x,y) for i,x in enumerate(names) for y in names[i+1:] if x in changed or y in changed]
    selected_indices={p:native.relative_indices(bodies[p[0]],bodies[p[1]],chains) for p in pairs};cache={}
    observations=[dict(id=c['id'],kind='static',angles_deg=c['angles_deg'],baseline=c['result']) for c in base['cases']]
    observations += [dict(id=p['source']+'__'+p['target']+'@'+str(o['fraction']),kind='transition',
                    angles_deg=o['result']['angles_deg'],baseline=o['result']) for p in trans['paths'] for o in p['observations']]
    if a.select:observations=[c for c in observations if c['id'] in a.select.split(',')]
    report=dict(completed=False,source_sha256=hashes,requests=base['requests'],cases=[],
        partition=dict(parts=364,unchanged=357,modified=sorted(modified),added=sorted(added),newly_examined_pair_count=len(pairs),
        reused_only_when_both_parts_unchanged=True,kinematics='Drive build references the identical manufacturing_v2 design hash; same neutral-world STEP frames and explicit body labels. No joint-frame edits.'),
        scope='364 hand parts only. Separate external motor bench, tendon routing and later isolated testbench are not installed whole-hand hardware. Finite rigid nominal poses; no physical or continuous-motion release.')
    def save():
        a.output.parent.mkdir(parents=True,exist_ok=True);tmp=a.output.with_suffix('.partial');tmp.write_text(json.dumps(report,separators=(',',':'),allow_nan=False),encoding='utf-8');tmp.replace(a.output)
    for obs in observations:
        q=obs['angles_deg'];current=v.frames(design,q);moved={n:v.relocate(s,current[bodies[n]],neutral[bodies[n]]) for n,s in parts.items()};boxes={n:v.bounds(s) for n,s in moved.items()};collisions=[];errors=[];calls=0
        for x,y in pairs:
            if np.any(np.minimum(boxes[x][1],boxes[y][1])-np.maximum(boxes[x][0],boxes[y][0])<=0):continue
            key=(x,y,tuple(float(q[i]).hex() for i in selected_indices[(x,y)]))
            if key not in cache:
                calls+=1
                try:
                    common=moved[x].intersect(moved[y],tol=1e-6);vol=float(common.Volume())
                    if not math.isfinite(vol) or vol< -1e-9:raise ValueError('Invalid intersection volume')
                    cache[key]=dict(a=x,b=y,material_intersection_mm3=vol,collision=vol>.02)
                except Exception as e:cache[key]=dict(a=x,b=y,error=repr(e))
            row=cache[key]
            if 'error' in row:errors.append(row)
            elif row['collision']:collisions.append(row)
        inherited=[c for c in obs['baseline']['collision_details'] if c['a'] in unchanged and c['b'] in unchanged]
        inherited_errors=[c for c in obs['baseline']['operation_failures'] if c['a'] in unchanged and c['b'] in unchanged]
        combined=inherited+collisions
        report['cases'].append(dict(id=obs['id'],kind=obs['kind'],angles_deg=q,collision_details=combined,collision_count=len(combined),
            new_or_modified_pair_collisions=collisions,inherited_invariant_pair_collisions=len(inherited),operation_failures=inherited_errors+errors,exact_boolean_calls=calls))
        save();print(json.dumps(dict(pose=len(report['cases']),total=len(observations),id=obs['id'],collisions=len(combined),delta_collisions=len(collisions))),flush=True)
    report['completed']=True;report['input_hash_guard']=dict(passed=all(sha(root/k)==h for k,h in hashes.items()))
    report['summary']=dict(observations=len(report['cases']),static_cases=sum(c['kind']=='static' for c in report['cases']),
        colliding_static_cases=sum(c['kind']=='static' and c['collision_count']>0 for c in report['cases']),
        transition_observations=sum(c['kind']=='transition' for c in report['cases']),
        colliding_transition_paths=len({c['id'].split('@')[0] for c in report['cases'] if c['kind']=='transition' and c['collision_count']}),
        operation_failure_observations=sum(len(c['operation_failures']) for c in report['cases']),
        whole_hand_motion_approved=False)
    save()
if __name__=='__main__':main()

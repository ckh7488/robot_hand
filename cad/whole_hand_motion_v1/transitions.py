"""Whole-hand transition discovery at quarter points of every gesture pair.

Unplanned linear interpolation is explicitly a probe, not a feasible trajectory.
Time reversal has the same rigid pose set; it is not double counted.
"""
import argparse,hashlib,importlib.util,itertools,json,sys,time
from pathlib import Path
import numpy as np
import cadquery as cq

HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def mod(name,p):
    s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,default=HERE.parents[1]);ap.add_argument('--audit',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=a.repo;source=read(a.audit)
    if not source['completed'] or source['errors']:raise ValueError('Incomplete source')
    for k,h in source['source_sha256'].items():
        if sha(root/k)!=h:raise ValueError('Changed input '+k)
    n=mod('_transition_native',root/'cad/gesture_hand_v3/audit_gestures.py');v=n.load_v2(root)
    scene=read(root/'cad/manufacturing_v2/generated/scene.json');parts={}
    for r in scene['hand']:parts[r['name']]=cq.importers.importStep(str(root/'cad/manufacturing_v2'/r['step'])).val()
    engine=n.PairScreen(v,parts,scene['hand'],source['requests']['design'],0.,True)
    gestures=[c for c in source['cases'] if c['kind']=='accepted_gesture']
    if len(gestures)!=9:raise ValueError('Expected9 accepted gestures')
    report=dict(completed=False,audit_sha256=sha(a.audit),script_sha256=sha(__file__),paths=[],errors=[],
        scope='All36 unordered pairs among9 accepted gestures. Quarter/mid/three-quarter poses only, plus already audited endpoints. Reversing identical interpolation follows the same geometry. No continuous/free-path certificate, dynamics or physical reverse-motion approval.')
    def save():
        a.output.parent.mkdir(parents=True,exist_ok=True);t=a.output.with_suffix('.partial');t.write_text(json.dumps(report,separators=(',',':'),allow_nan=False),encoding='utf-8');t.replace(a.output)
    for first,second in itertools.combinations(gestures,2):
        path=dict(source=first['id'],target=second['id'],observations=[])
        for t in [.25,.5,.75]:
            q=(1-t)*np.array(first['angles_deg'])+t*np.array(second['angles_deg']);r=engine.evaluate(q)
            r.pop('exact_candidate_results');r.pop('near_pairs');path['observations'].append(dict(fraction=t,result=r))
        path['sampled_collision_count']=sum(x['result']['collision_count'] for x in path['observations'])
        report['paths'].append(path);save();print(json.dumps(dict(path=len(report['paths']),source=path['source'],target=path['target'],collisions=path['sampled_collision_count'])),flush=True)
    report['completed']=True;report['summary']=dict(paths=36,intermediate_observations=108,
        colliding_paths=sum(p['sampled_collision_count']>0 for p in report['paths']),
        collision_observations=sum(p['sampled_collision_count'] for p in report['paths']),
        operation_failures=sum(len(o['result']['operation_failures']) for p in report['paths'] for o in p['observations']),whole_hand_motion_approved=False)
    report['input_hash_guard']=dict(passed=all(sha(root/k)==h for k,h in source['source_sha256'].items()))
    save()
if __name__=='__main__':main()

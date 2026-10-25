"""Verify a finite 0.5 mm cross-digit gap goal using conservative box pruning.

This computes a certified nominal lower bound at samples, not a global minimum
distance or a continuous-path certificate. All index-to-middle pairs are covered.
"""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np
import cadquery as cq
HERE=Path(__file__).resolve().parent
GOAL=.5

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--audit',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=HERE.parents[1];report=read(a.audit)
    if not report['completed'] or not report['selected']:raise ValueError('No selected sampled path')
    def guard():return all(sha(root/k)==h for k,h in report['source_sha256'].items())
    if not guard():raise ValueError('Input changed')
    p=root/'cad/articulation_drive_v2/verify.py';s=importlib.util.spec_from_file_location('_gap_goal_fk',p);v=importlib.util.module_from_spec(s);s.loader.exec_module(v)
    rows=[r for r in read(root/'cad/thumb_ip_drive_v1/generated/scene.json')['hand'] if r['name'].startswith(('index_','middle_'))];parts={}
    for r in rows:
        p=root/r['step'] if r['location']=='repo' else root/'cad/thumb_ip_drive_v1/generated'/r['step'];parts[r['name']]=cq.importers.importStep(str(p)).val()
    bodies={r['name']:r['body'] for r in rows};neutral=v.frames(report['design'],np.zeros(20));path=next(p for p in report['paths'] if p['target']==report['selected']);answers=[]
    for o in path['observations']:
        q=o['result']['angles_deg'];frames=v.frames(report['design'],q);shapes={n:v.relocate(s,frames[bodies[n]],neutral[bodies[n]]) for n,s in parts.items()};boxes={n:v.bounds(s) for n,s in shapes.items()}
        count=0;near=[]
        for x in shapes:
            if not x.startswith('index_'):continue
            for y in shapes:
                if not y.startswith('middle_'):continue
                count+=1
                lower=float(np.linalg.norm(np.maximum(np.maximum(boxes[x][0]-boxes[y][1],boxes[y][0]-boxes[x][1]),0.)))
                if lower>GOAL+1e-6:continue
                distance=float(shapes[x].distance(shapes[y]))
                if not math.isfinite(distance) or distance<0:raise ValueError('Invalid exact gap')
                near.append(dict(pair=[x,y],gap_mm=distance,box_lower_bound_mm=lower))
        least=min((r['gap_mm'] for r in near),default=math.inf)
        row=dict(segment=o['segment'],fraction=o['fraction'],angles_deg=q,all_cross_digit_pairs=count,
                 exact_distance_calls=len(near),exact_near_results=near,
                 nominal_gap_lower_bound_mm=min(GOAL,least),meets_half_mm_goal=least>=GOAL)
        answers.append(row);print(json.dumps(dict(sample=len(answers),lower_bound_mm=row['nominal_gap_lower_bound_mm'],exact_calls=len(near))),flush=True)
    if not guard():raise ValueError('Input changed during check')
    result=dict(completed=True,input_hash_guard=True,source_audit_sha256=sha(a.audit),script_sha256=sha(__file__),selected=report['selected'],samples=answers,
        gap_goal_mm=GOAL,minimum_certified_sampled_gap_mm=min(r['nominal_gap_lower_bound_mm'] for r in answers),
        meets_sampled_half_mm_goal=all(r['meets_half_mm_goal'] for r in answers),global_minimum_computed=False,
        scope='Only index-vs-middle pairs at listed samples. Pairs beyond the goal excluded by conservative solid AABB lower bounds; all others use exact nominal BRep distance. No other digits, tolerances, continuous movement or actuation approval.')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,separators=(',',':'),allow_nan=False),encoding='utf-8')
if __name__=='__main__':main()

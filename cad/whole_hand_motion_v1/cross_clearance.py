"""Exact nominal index-to-middle clearance at the selected sampled crossing path."""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np
import cadquery as cq
HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--audit',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=HERE.parents[1];report=read(a.audit)
    if not report['completed'] or not report['selected']:raise ValueError('No selected collision-free sampled path')
    for k,h in report['source_sha256'].items():
        if sha(root/k)!=h:raise ValueError('Input changed '+k)
    p=root/'cad/articulation_drive_v2/verify.py';s=importlib.util.spec_from_file_location('_cross_gap_fk',p);v=importlib.util.module_from_spec(s);s.loader.exec_module(v)
    rows=[r for r in read(root/'cad/thumb_ip_drive_v1/generated/scene.json')['hand'] if r['name'].startswith(('index_','middle_'))];parts={}
    for r in rows:
        p=root/r['step'] if r['location']=='repo' else root/'cad/thumb_ip_drive_v1/generated'/r['step'];parts[r['name']]=cq.importers.importStep(str(p)).val()
    bodies={r['name']:r['body'] for r in rows};neutral=v.frames(report['design'],np.zeros(20));path=next(p for p in report['paths'] if p['target']==report['selected']);answers=[]
    for o in path['observations']:
        q=o['result']['angles_deg'];frames=v.frames(report['design'],q);shapes={n:v.relocate(s,frames[bodies[n]],neutral[bodies[n]]) for n,s in parts.items()};boxes={n:v.bounds(s) for n,s in shapes.items()}
        pairs=[]
        for x in shapes:
            if not x.startswith('index_'):continue
            for y in shapes:
                if not y.startswith('middle_'):continue
                lower=float(np.linalg.norm(np.maximum(np.maximum(boxes[x][0]-boxes[y][1],boxes[y][0]-boxes[x][1]),0.)))
                pairs.append((lower,x,y))
        best=math.inf;who=None;calls=0
        for lower,x,y in sorted(pairs):
            if lower>best:break
            gap=float(shapes[x].distance(shapes[y]));calls+=1
            if not math.isfinite(gap) or gap<0:raise ValueError('Invalid gap')
            if gap<best:best=gap;who=[x,y]
        answers.append(dict(segment=o['segment'],fraction=o['fraction'],angles_deg=q,minimum_gap_mm=best,pair=who,exact_distance_calls=calls,all_cross_digit_pairs=len(pairs)))
    guarded=all(sha(root/k)==h for k,h in report['source_sha256'].items())
    if not guarded:raise ValueError('Input changed during clearance calculation')
    result=dict(completed=True,input_hash_guard=guarded,source_audit_sha256=sha(a.audit),script_sha256=sha(__file__),selected=report['selected'],samples=answers,
        minimum_sampled_gap_mm=min(x['minimum_gap_mm'] for x in answers),meets_sampled_half_mm_goal=all(x['minimum_gap_mm']>=.5 for x in answers),
        scope='Only index-vs-middle part pairs at listed path samples; exact nominal BRep distances with AABB lower-bound pruning. No tolerances, other digits, continuous path or actuation approval.')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,separators=(',',':'),allow_nan=False),encoding='utf-8')
if __name__=='__main__':main()

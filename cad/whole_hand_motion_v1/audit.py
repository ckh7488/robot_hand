"""Saved whole-hand STEP motion discovery; finite poses, no release certificate."""
import argparse,copy,hashlib,importlib.util,json,sys,time,traceback
from pathlib import Path
import numpy as np
import cadquery as cq

HERE=Path(__file__).resolve().parent
def mod(name,path):
    s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def write(p,d):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);t=p.with_suffix('.partial');t.write_text(json.dumps(d,separators=(',',':'),allow_nan=False),encoding='utf-8');t.replace(p)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,default=HERE.parents[1]);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--select',default='');a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=a.repo.resolve();base=root/'cad/manufacturing_v2';hashes={}
    def bind(p,expected=None):
        p=Path(p).resolve();k=p.relative_to(root).as_posix();h=sha(p)
        if expected and expected!=h:raise ValueError('Changed input '+k)
        hashes[k]=h;return p
    start=time.monotonic();report=dict(completed=False,errors=[],cases=[],source_sha256=hashes)
    def save():
        report['elapsed_s']=time.monotonic()-start
        report['input_hash_guard']=dict(passed=all(sha(root/k)==h for k,h in hashes.items()))
        write(a.output,report)
    try:
        bind(__file__);cat=mod('_motion_catalog',bind(HERE/'catalog.py')).make(root)
        bind(root/'cad/gesture_hand_v3/poses.json');bind(base/'generated/design.json')
        report['requests']=cat; design=cat['design']
        scene=read(bind(base/'generated/scene.json'));normalized=copy.deepcopy(scene)
        for row in normalized['hand']:
            for key,hkey in [('step','sha256_step'),('mesh','sha256_stl')]:row[key]=bind(base/row[key],row[hkey]).relative_to(root).as_posix()
        native=mod('_motion_native',bind(root/'cad/gesture_hand_v3/audit_gestures.py'));bind(root/'cad/articulation_drive_v2/verify.py');v=native.load_v2(root)
        parts={}
        for i,row in enumerate(normalized['hand']):
            s=cq.importers.importStep(str(root/row['step'])).val()
            if not s.isValid() or len(s.Solids())!=1 or abs(s.Volume()-row['volume_mm3'])>1e-3:raise ValueError('Invalid saved part '+row['name'])
            parts[row['name']]=s
            if i%40==0:print(json.dumps(dict(loaded=i+1)),flush=True)
        report['part_count']=len(parts)
        # No solids omitted; test nominal parts before any conservative hardware envelope.
        engine=native.PairScreen(v,parts,normalized['hand'],design,0.,True)
        observations=[c for c in cat['cases'] if not a.select or c['id'] in a.select.split(',')]
        for obs in observations:
            result=engine.evaluate(obs['angles_deg'])
            # Keep collision evidence and counts, not repeated zero-volume interface tables.
            result.pop('exact_candidate_results');result.pop('near_pairs')
            report['cases'].append(dict(**obs,result=result))
            save();print(json.dumps(dict(case=obs['id'],collisions=result['collision_count'],seconds=report['elapsed_s'])),flush=True)
        report['completed']=True
        report['summary']=dict(cases=len(report['cases']),colliding_cases=sum(c['result']['collision_count']>0 for c in report['cases']),
            collision_observations=sum(c['result']['collision_count'] for c in report['cases']),operation_failures=sum(len(c['result']['operation_failures']) for c in report['cases']),
            whole_hand_motion_approved=False,scope='359 nominal saved parts, finite exploratory poses; no continuous movement, tolerances, cords, wiring, motor actuation or physical approval')
    except Exception:report['errors'].append(traceback.format_exc())
    save();return 0 if report['completed'] and report['input_hash_guard']['passed'] else 1
if __name__=='__main__':raise SystemExit(main())

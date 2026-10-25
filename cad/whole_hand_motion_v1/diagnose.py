"""Locate observed blocking angles on selected individual-joint part pairs.

Only a discovery diagnostic: 1-degree samples and refined hit brackets do not
certify the continuous path or other part pairs.
"""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np
import cadquery as cq

HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):
    s=importlib.util.spec_from_file_location('_motion_diagnostic_fk',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,default=HERE.parents[1]);ap.add_argument('--audit',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    report=read(a.audit);root=a.repo
    if not report['completed'] or report['errors']:raise ValueError('Incomplete discovery')
    for k,h in report['source_sha256'].items():
        if sha(root/k)!=h:raise ValueError('Changed source '+k)
    v=load(root/'cad/articulation_drive_v2/verify.py');design=report['requests']['design'];neutral=v.frames(design,np.zeros(20))
    scene=read(root/'cad/manufacturing_v2/generated/scene.json');rows={r['name']:r for r in scene['hand']};solids={};answers=[]
    selected=[]
    for c in report['cases']:
        if c['kind']!='isolated_axis_extreme' or not c['required_function'] or not c['result']['collision_details']:continue
        active=np.flatnonzero(c['angles_deg'])
        if len(active)!=1:continue
        # Every colliding pair in the axis probe; no maximum-only selection.
        for pair in c['result']['collision_details']:selected.append((c,pair,int(active[0])))
    for c,pair,j in selected:
        for name in [pair['a'],pair['b']]:
            if name not in solids:solids[name]=cq.importers.importStep(str(root/'cad/manufacturing_v2'/rows[name]['step'])).val()
        def measure(angle):
            q=np.zeros(20);q[j]=angle;current=v.frames(design,q)
            moved=[v.relocate(solids[n],current[rows[n]['body']],neutral[rows[n]['body']]) for n in [pair['a'],pair['b']]]
            boxes=[v.bounds(s) for s in moved]
            volume=0. if np.any(np.minimum(boxes[0][1],boxes[1][1])-np.maximum(boxes[0][0],boxes[1][0])<=0) else float(moved[0].intersect(moved[1],tol=1e-6).Volume())
            if not math.isfinite(volume) or volume< -1e-9:raise ValueError('Invalid intersection')
            return dict(angle_deg=float(angle),overlap_mm3=volume,hit=volume>.02)
        limit=c['angles_deg'][j];samples=[];prior=None;bracket=None
        for angle in np.linspace(0,limit,int(math.ceil(abs(limit)))+1):
            row=measure(angle);samples.append(row)
            if row['hit']:
                if prior is None:raise ValueError('Pair already overlaps at neutral')
                lo,hi=prior['angle_deg'],row['angle_deg']
                while abs(hi-lo)>.05:
                    mid=(lo+hi)/2;test=measure(mid);samples.append(test)
                    if test['hit']:hi=mid
                    else:lo=mid
                bracket=dict(nonreporting_angle_deg=lo,reporting_angle_deg=hi,width_deg=abs(hi-lo));break
            prior=row
        answers.append(dict(case=c['id'],joint=report['requests']['names'][j],a=pair['a'],b=pair['b'],
            samples=samples,first_observed_hit_bracket=bracket,scope='This pair only; earlier 1-degree grid can miss isolated narrow collisions. Bracket is around the first sampled hit, not a continuous free-travel certificate.'))
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(dict(completed=False,audit_sha256=sha(a.audit),pairs=answers),separators=(',',':')),encoding='utf-8')
        print(json.dumps(dict(case=c['id'],a=pair['a'],b=pair['b'],bracket=bracket)),flush=True)
    a.output.write_text(json.dumps(dict(completed=True,audit_sha256=sha(a.audit),script_sha256=sha(__file__),pairs=answers),separators=(',',':')),encoding='utf-8')
if __name__=='__main__':main()

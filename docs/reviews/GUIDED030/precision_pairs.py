"""Two adjacent sleeve/curve intersections only, with unchanged tolerances."""
import sys,argparse,subprocess,json
from pathlib import Path
sys.dont_write_bytecode=True
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
ROOT=args.root.resolve();sys.path.insert(0,str(ROOT/'cad/guided_housing_link_v1'))
import guided_link_model as a
m,c,p,np,cq=a.m,a.c,a.p,a.np,a.cq
measure=m.drive.module('_guided030_pair_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
man=p.read(ROOT/'checkpoints/GUIDED030.manifest.json')
guard=lambda:all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items())
assert guard();raw=p.read(ROOT/'cad/guided_housing_link_v1/reviews/check.json');case=raw['cases'][0];meta=p.read(ROOT/case['scene'])['parts'];rows=[]
def values(sh):
    z=measure.metrics(sh);return z['global_values']
for i in [0,1]:
    s=case['solutions'][i];edges=a.edges(s,'housing');fresh=[];names=['guided_housing_'+str(i)+'_'+str(j) for j in [4,5]]
    for j in [4,5]:fresh.append(m.swept(edges[j],1.5).cut(m.swept(edges[j],.65)).clean())
    saved=c.load([next(z for z in meta if z['name']==name) for name in names]);A=fresh[0].intersect(fresh[1],tol=1e-6);B=saved[0]['shape'].intersect(saved[1]['shape'],tol=1e-6)
    aa,bb=values(A),values(B);delta=np.abs(np.asarray(aa)-bb);ok=bool(np.max(delta)<1e-5)
    rows.append(dict(parts=names,source_common_default_volume_mm3=A.Volume(),STEP_common_default_volume_mm3=B.Volume(),source_adaptive_values=aa,STEP_adaptive_values=bb,absolute_differences=delta.tolist(),matched=ok));print(names,delta.tolist(),ok,flush=True)
clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=all(z['matched'] for z in rows) and clean and guard(),pairs=rows,epsilon=1e-10,absolute_comparison_limit=1e-5,boolean_tolerance_mm=1e-6,reporting_floor_mm3=.001,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),manifest_files=len(man['files']),checkout_clean=clean,input_hash_guard=guard(),initial_core_passed=False,geometry_or_reporting_thresholds_changed=False,other_native_groups_repeated=False,whole_hand_drive_approved=False,independent_audit=False,scope='Only source/source versus savedSTEP/savedSTEP native common of guide straight section4 and first distalcurve section5,one pair for each housing. Adaptive volume/area/center epsilon1e-10 with original abs1e-5. No new curve/part/angle/trim modification,no all-group recheck or physical routing approval.')
p.write(args.output,result)

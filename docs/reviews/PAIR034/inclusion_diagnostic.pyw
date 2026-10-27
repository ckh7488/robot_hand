"""Bounded diagnosis of only the two failed final-reference/STEP-envelope cuts."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,required=True);ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
sys.path.insert(0,str(args.repo/'cad/paired_rear_guide_v1'))
import pair_model as a
m,c,p,np,cq,ROOT,HERE=a.m,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.OSD import OSD_ThreadPool
def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
def volume(sh):
    if not sh.Solids():return 0.
    q=GProp_GProps();BRepGProp.VolumeProperties_s(sh.wrapped,q,1e-10,False,False);return q.Mass()
assert ROOT.resolve()==args.repo.resolve() and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
static=p.read(HERE/'reviews/static.json');service=p.read(HERE/'reviews/service.json');L=static['design']['centerline_length_mm'];saved=c.load(p.read(ROOT/service['feed_union_envelope_scene'])['parts']);results=[]
for i in [0,1]:
    ref=a.window(i,L+10)['shape'];source_env=a.window(i,L+10,L+230)['shape'];step_env=saved[i]['shape'];rv=volume(ref);ss=volume(ref.cut(source_env,tol=1e-6));common=ref.intersect(step_env,tol=1e-6);cv=volume(common);points=[]
    for s in np.linspace(L+10-220+.1,L+10-.1,25):
        ds=a.definitions(i);A=np.array(ds[0]['start']);D=np.array(ds[-1]['end']);start=0.
        if s<0:pt=A+[0,s,0];t=np.array([0.,1.,0.])
        elif s>L:pt=D+[0,s-L,0];t=np.array([0.,1.,0.])
        else:
            for d in ds:
                if s<=start+d['length']:
                    ed=a.edge(d);u=(s-start)/d['length'];pt=np.array(ed.positionAt(u).toTuple());t=np.array(ed.tangentAt(u).toTuple());break
                start+=d['length']
        n=np.array([-t[1],t[0],0.]);z=np.array([0.,0.,1.])
        for radius in [.3,.8,1.075,1.4,1.7]:
            for theta in np.linspace(0,2*np.pi,12,endpoint=False):
                q=pt+radius*(np.cos(theta)*n+np.sin(theta)*z);classification=[sh.isInside(cq.Vector(*q),1e-6) for sh in [ref,source_env,step_env]];points.append(dict(centerline_s_mm=float(s),radius_mm=radius,angle_rad=float(theta),point_mm=q.tolist(),inside_reference_source_envelope_STEP_envelope=classification))
    mismatches=[z for z in points if len(set(z['inside_reference_source_envelope_STEP_envelope']))!=1];results.append(dict(line=i,reference_volume_adaptive_mm3=rv,reference_source_envelope_cut_adaptive_mm3=ss,reference_STEP_envelope_common_adaptive_mm3=cv,common_minus_reference_mm3=cv-rv,source_and_STEP_envelope_valid=[source_env.isValid(),step_env.isValid()],original_exact_SHA_cut_outside_mm3=1263.0773263757774 if i==0 else 1263.0773263757783,point_count=len(points),point_classification_mismatches=mismatches,points=points,analytic_reference_interval_mm=[L+10-220,L+10],analytic_union_interval_mm=[-220,L+10],analytic_interval_subset=True));print(i,rv,ss,cv,len(points),len(mismatches),flush=True)
result=dict(completed=True,target_commit=args.commit,lines=results,initial_core_passed=False,original_two_failed_cut_results_preserved=True,scope='Only final tip186.428840823mm two references:source/source cut,source/STEP common with same Boolean tol1e-6,adaptive volume epsilon1e-10,750interior/bore/outside material classifications perline and analytic materialinterval inclusion.No 41native/7metrics/static/service/render rerun.No CAD/clone/reporting/comparison threshold changes.Not all surface equations/trim/Boolean equivalence or physical feeding proof.',full_STEP_inclusion_comparison_resolved=False,checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False)
p.write(args.output,result)

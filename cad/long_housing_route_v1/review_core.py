"""Scoped exact-SHA geometry/length/native replay, not physical validation."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import route_solver as m
base=m.drive.module('_housing027_review_inputs',m.HERE/'build_check.py')
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from scipy.optimize import linear_sum_assignment
def metrics(sh):
    vol=GProp_GProps();area=GProp_GProps();BRepGProp.VolumeProperties_s(sh.wrapped,vol,1e-10,False,False);BRepGProp.SurfaceProperties_s(sh.wrapped,area,1e-10,False)
    faces=[]
    for z in sh.Faces():
        a=GProp_GProps();BRepGProp.SurfaceProperties_s(z.wrapped,a,1e-10,False);faces.append(dict(kind=z.geomType(),values=[a.Mass(),*a.CentreOfMass().Coord()]))
    return dict(valid=sh.isValid(),solids=len(sh.Solids()),global_values=[vol.Mass(),area.Mass(),*vol.CentreOfMass().Coord()],faces=faces,edges=[dict(kind=z.geomType(),values=[z.Length(),*z.Center().toTuple()]) for z in sh.Edges()],vertices=[dict(kind='point',values=list(z.toTuple())) for z in sh.Vertices()])
def match(a,b):
    maximum=float(max(abs(np.asarray(a['global_values'])-b['global_values'])));bad=[]
    if not a['valid'] or not b['valid'] or a['solids']!=1 or b['solids']!=1:bad.append('valid_or_solids')
    for key in ['faces','edges','vertices']:
        for kind in sorted({z['kind'] for z in a[key]+b[key]}):
            aa=[z for z in a[key] if z['kind']==kind];bb=[z for z in b[key] if z['kind']==kind]
            if len(aa)!=len(bb):bad.append(key+'_count');continue
            cost=np.asarray([[max(abs(np.asarray(x['values'])-y['values'])) for y in bb] for x in aa]);ii,jj=linear_sum_assignment(cost);maximum=max(maximum,float(max(cost[ii,jj])))
    if maximum>1e-5:bad.append('absolute_metric_limit')
    return dict(matched=not bad,max_numeric_difference=maximum,differences=bad)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    man=p.read(ROOT/'checkpoints/HOUSING027.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');selected=p.read(HERE/'selected/reviews/check.json');cases=[z for z in selected['cases'] if 'scene' in z];geometrycase=cases[0] if cases else raw['cases'][0];fresh=[]
    for z in geometrycase['solutions']:fresh+=m.tubes(z)[0]
    saved=c.load(p.read(ROOT/geometrycase['scene'])['parts']);lookup={z['name']:z for z in fresh};geometry=[];differences=[];native=[];lengths=[]
    for z in saved:
        got=dict(match(metrics(lookup[z['name']]['shape']),metrics(z['shape'])),name=z['name']);geometry.append(got)
        if not got['matched']:differences.append('geometry_'+z['name'])
    for case in selected['cases']:
        for sol in case['solutions']:
            endpoints=m.endpoints(case['angles_deg'],sol['cord']);C=np.asarray(sol['controls']);err=max(np.max(abs(C[0]-endpoints['A'])),np.max(abs(C[-1]-endpoints['B'])));tangents=[float(np.dot((C[1]-C[0])/np.linalg.norm(C[1]-C[0]),endpoints['tA'])),float(np.dot((C[-1]-C[-2])/np.linalg.norm(C[-1]-C[-2]),endpoints['tB']))];length=m.properties(C)[0];ok=err<1e-5 and min(tangents)>1-1e-10 and abs(length-sol['quadrature_length_mm'])<1e-5;lengths.append(dict(id=case['id'],cord=sol['cord'],endpoint_max_error_mm=float(err),tangent_dot=tangents,quadrature_length_mm=length,solver_success_as_saved=sol['solver_success'],matched=bool(ok)))
            if not ok:differences.append('anchor_length_'+str(case['id'])+'_'+str(sol['cord']))
    data,pack,guide=base.inputs()
    def compare(label,got,want):
        def mapping(x):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']}
        a,b=mapping(got),mapping(want);ok=a.keys()==b.keys() and all(abs(a[k]-b[k])<1e-5 for k in a.keys()&b.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    def combined(xs):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in xs),overlaps=[v for z in xs for v in z['overlaps']],operation_failures=[v for z in xs for v in z['operation_failures']])
    selectedids=sorted({0,len(cases)//2,len(cases)-1}) if cases else []
    for label,case in [('original',raw['cases'][0])]+[('selected',cases[i]) for i in selectedids]:
        rows=c.load(p.read(ROOT/case['scene'])['parts']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
        for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]:compare(label+'_'+str(case['id'])+'_'+key,c.material(rows,ob),case['native'][key])
        compare(label+'_'+str(case['id'])+'_internal190',combined([c.material([z],rows[i+1:]) for i,z in enumerate(rows)]),combined(case['internal']))
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,anchor_and_length_comparisons=lengths,manifest_files=len(man['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),source_curve_controls_replayed=True,nonlinear_optimizer_rerun=False,inherited_OFFSET024_face_comparison_inconclusive=True,whole_hand_drive_approved=False,independent_audit=False,scope='20source/STEPadaptive global/facearea-centroid/edge-length-centroid/vertex metrics atabs1e-5;sourcefromsavedBeziercontrols.No per-face AddOptimal bbox or old TORUS review retry;no all surface equations proof.Current actualSTEP nativefirstfailure and selected first/mid/last available poses.All saved endpoint/length constraints re-evaluated.Noallposes/render,wrist/physicalelastic/friction/tension/pullout,wholehand or setupassembly approval.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()

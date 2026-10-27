"""Exact-SHA native replay and saved-control reconstruction, without re-solving."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import build_contact as b
s,a,m,g,c,p,np,cq,ROOT,HERE=b.s,b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
measure=m.drive.module('_contact036_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
ext=m.drive.module('_contact036_extrema',HERE/'curve_extrema.py')
independent=m.drive.module('_contact036_length',HERE/'length_supplement.py')
from OCP.OSD import OSD_ThreadPool
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    man=p.read(ROOT/'checkpoints/CONTACT036.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');fresh,lengths=b.tubes(raw['solver']);saved=c.load(p.read(ROOT/raw['scene'])['parts']);lookup={z['name']:z for z in fresh};geometry=[];native=[];differences=[]
    for i in [0,1]:
        hs=[z for z in saved if z['physical_line']==i and z['kind']=='housing'];indices=[0,1,3,5,7,9,10,12,len(hs)-1]+([15,16] if i==1 else [])
        for j in indices:
            z=hs[j];got=dict(name=z['name'],**measure.match(measure.metrics(lookup[z['name']]['shape']),measure.metrics(z['shape'])));geometry.append(got)
            if not got['matched']:differences.append('geometry_'+z['name'])
    data,pack,guide,wrist=s.prior.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));hand+=c.load([z for z in p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'] if z['installed_part_candidate']],data[0]);assert len(hand)==397
    def combined(xs):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in xs),overlaps=[v for z in xs for v in z['overlaps']],operation_failures=[v for z in xs for v in z['operation_failures']])
    def compare(label,got,want):
        mapping=lambda x:{(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']};aa,bb=mapping(got),mapping(want);err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];native.append(dict(id=label,native=got,max_numeric_difference=err,matched=bool(ok)));print(label,ok,flush=True)
        if not ok:differences.append(label)
    for k,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_guide',wrist)]:compare(k,c.material(saved,ob),raw['native'][k])
    compare('internal',combined([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),combined(raw['internal']))
    aa=s.anchors(np.zeros(20));C=np.asarray(raw['solver']['controls']);anchor_checks=[];radius_checks=[];length_checks=[];er=p.read(HERE/'reviews/extrema.json');lr=p.read(HERE/'reviews/length_supplement.json')
    for i in [0,1]:
        for j,side in enumerate(['up','down']):
            cc=C[i*2+j];v=s.GD@cc;length=float(s.GW@np.linalg.norm(v,axis=1));A,Z,tA,tB=[aa[i][side+'_'+k] for k in ['A','B','tA','tB']];dots=[float(np.dot((cc[1]-cc[0])/np.linalg.norm(cc[1]-cc[0]),tA)),float(np.dot((cc[-1]-cc[-2])/np.linalg.norm(cc[-1]-cc[-2]),tB))];error=float(max(np.linalg.norm(cc[0]-A),np.linalg.norm(cc[-1]-Z)));got=dict(line=i,span=side,length_mm=length,endpoint_error_mm=error,tangent_dot=dots,matched=error<1e-5 and min(dots)>1-1e-8 and abs(length-raw['solver']['free_span_lengths_mm'][i*2+j])<1e-5);anchor_checks.append(got)
            if not got['matched']:differences.append('anchor_'+str(i)+'_'+side)
            rr=ext.extrema(cc);rmatch=abs(rr['minimum_radius_mm']-er['curves'][2*i+j]['minimum_radius_mm'])<1e-5;radius_checks.append(dict(line=i,span=side,minimum_radius_mm=rr['minimum_radius_mm'],matched=rmatch))
            if not rmatch:differences.append('radius_'+str(i)+'_'+side)
            ll=independent.length(cc);lmatch=abs(ll-lr['selected_free_span_lengths_mm'][2*i+j])<1e-5;length_checks.append(dict(line=i,span=side,split_quad_length_mm=ll,matched=lmatch))
            if not lmatch:differences.append('independent_length_'+str(i)+'_'+side)
    residual_checks=[]
    for i in [0,1]:
        target=raw['solver']['new_free_length_targets_mm'][i];gauss=sum(z['length_mm'] for z in anchor_checks if z['line']==i)-target;quad=sum(z['split_quad_length_mm'] for z in length_checks if z['line']==i)-target;ok=abs(gauss-raw['solver']['length_residual_mm'][i])<1e-5 and abs(quad-lr['physical_lines'][i]['independent_residual_to_same_saved_target_mm'])<1e-5;residual_checks.append(dict(line=i,Gauss64_residual_mm=gauss,split_quad_residual_mm=quad,matched=ok))
        if not ok:differences.append('length_residual_'+str(i))
    failure_preserved=not raw['solver']['solver_success'] and raw['solver']['iterations']==80 and not raw['candidate_admissible']
    if not failure_preserved:differences.append('original_solver_failure_not_preserved')
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,analytic_anchor_reconstructions=anchor_checks,polynomial_radius_reconstructions=radius_checks,independent_lengths=length_checks,length_residual_reconstructions=residual_checks,original_solver_failure_preserved=failure_preserved,manifest_files=len(man['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,scope='Neutral-only6nativegroups(current397hand/385pack/64elect/10arm/PAIR6 plusallnewinternal),20representativehousing adaptive source/STEP signatures abs1e-5,4saved-control actualendpoint+tangent+Gausslength/polynomialcurvature/splitquadlength reconstructions,2originalresiduals.No contactoptimizer/wholeboundaryfield rerun/all68signatures/parent+wristmotions/render/assembly/actualcordthreading/force/friction/tension/materialorphysicalapproval.Original80iteration failure and length residuals preserved.Inheritedcomparisoninconclusive states untouched.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()

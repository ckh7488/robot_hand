"""Exact-SHA replay of the prescribed length family and its explicit failures."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_motion as k
a,m,g,c,p,np,cq,ROOT,HERE=k.a,k.m,k.g,k.c,k.p,k.np,k.cq,k.ROOT,k.HERE
measure=m.drive.module('_length037_metric_review',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/LENGTH037.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');analysis=p.read(HERE/'reviews/analysis.json');common=p.read(HERE/'reviews/actual_common.json');C0=np.asarray(a.read_seed()['solver']['controls']);differences=[]
    result=dict(completed=False,target_commit=args.commit,differences=differences,key_samples=[],geometry_comparisons=[],analytic_curves=[],material_markers=[],return_parameter_comparisons=[],adaptive_common_pairs=[])
    def save():a.save(args.output,result)
    save()
    for case in analysis['cases']:
        q=case['angles_deg'];P,H,F,TT=a.definitions(q,case['wrist_flex_deg'],case['wrist_deviation_deg']);C=np.asarray(case['solution']['controls']);anchors=a.s.anchors(q)
        for j,cc in enumerate(C):
            i,side=j//2,'up' if j%2==0 else 'down';z=case['solution']['lines'][i];expected=a.transported(C0[j],*TT[j])
            if side=='up':expected+=z['prescribed_loop_bulge_mm']*a.MODE[:,None]*np.asarray(z['prescribed_loop_direction'])
            e=anchors[i];A=e[side+'_A'] if side=='up' else (H@np.r_[e[side+'_A'],1])[:3];B=(H@np.r_[e[side+'_B'],1])[:3];ta=e[side+'_tA'] if side=='up' else H[:3,:3]@e[side+'_tA'];tb=H[:3,:3]@e[side+'_tB'];pos=max(np.linalg.norm(cc[0]-A),np.linalg.norm(cc[-1]-B));dots=[float(np.dot((cc[1]-cc[0])/np.linalg.norm(cc[1]-cc[0]),ta)),float(np.dot((cc[-1]-cc[-2])/np.linalg.norm(cc[-1]-cc[-2]),tb))];length=a.quad_length(cc);rad=a.ext.extrema(cc)['minimum_radius_mm'];ok=np.max(abs(cc-expected))<1e-8 and pos<1e-5 and min(dots)>1-1e-8 and abs(length-z['upstream_downstream_lengths_mm'][j%2])<1e-5 and abs(rad-case['curvature_checks'][j]['minimum_radius_mm'])<1e-5
            result['analytic_curves'].append(dict(case_id=case['id'],line=i,span=side,endpoint_error_mm=float(pos),tangent_dot=dots,length_mm=length,minimum_radius_mm=rad,local_radius_failure_preserved=bool(rad<=1.5)==(not case['curvature_checks'][j]['necessary_radius_greater_than_housing_outer_radius_1p5']),matched=bool(ok)))
            if not ok:differences.append('analytic_'+str(case['id'])+'_'+str(j))
        for i,z in enumerate(case['solution']['lines']):
            assert abs(a.quad_length(C[2*i])+a.quad_length(C[2*i+1])-z['free_length_basis_mm'])<1e-5
        if case['id'] in [0,1,10]:
            for i in [0,1]:
                points=a.material_points(case,i,np.linspace(0,1,9));error=max(np.max(abs(np.asarray(z['point_mm'])-want['point_mm'])) for z,want in zip(points,case['material_markers'][i]));ok=error<1e-5;result['material_markers'].append(dict(case_id=case['id'],line=i,point_count=9,maximum_position_error_mm=float(error),matched=bool(ok)))
                if not ok:differences.append('marker_'+str(case['id'])+'_'+str(i))
    for x,y in [(0,8),(1,7),(2,6),(3,5)]:
        error=float(np.max(abs(np.asarray(analysis['cases'][x]['solution']['controls'])-analysis['cases'][y]['solution']['controls'])));result['return_parameter_comparisons'].append(dict(case_ids=[x,y],maximum_control_difference_mm=error,matched=error<1e-8));assert error<1e-8
    save();data,pack,guide,wrist=a.s.prior.inputs()
    def combined(xs):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in xs),overlaps=[v for z in xs for v in z['overlaps']],operation_failures=[v for z in xs for v in z['operation_failures']])
    def compare(label,got,want):
        mapping=lambda x:{(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']};aa,bb=mapping(got),mapping(want);locations=aa.keys()==bb.keys();err=max([abs(aa[z]-bb[z]) for z in aa.keys()&bb.keys()]+[0.]);ok=locations and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];result['key_samples'].append(dict(id=label,native=got,overlap_locations_match=locations,max_basic_numeric_difference=err,matched=bool(ok)));print(label,ok,flush=True)
        if not ok:differences.append(label)
        save()
    for got in raw['cases']:
        case=analysis['cases'][got['id']];saved=c.load(p.read(ROOT/got['scene'])['parts']);lookup={z['name']:z for z in saved}
        for key,ob in k.obstacles(data,pack,guide,wrist,case):compare(str(case['id'])+'_'+key,c.material(saved,ob),got['native'][key])
        compare(str(case['id'])+'_internal',combined([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),combined(got['internal']))
        selected=[z for z in saved if z['kind']=='housing' and z['segment'] in [2,12]]
        if case['id']==10:selected+=[lookup['material_cord_0_2'],lookup['material_cord_1_2']]
        for z in selected:
            edge=a.full_edges(case,z['physical_line'],z['kind'])[z['segment']];fresh=m.swept(edge,1.5).cut(m.swept(edge,.65)).clean() if z['kind']=='housing' else m.swept(edge,.3);test=dict(case_id=case['id'],name=z['name'],**measure.match(measure.metrics(fresh),measure.metrics(z['shape'])));result['geometry_comparisons'].append(test)
            if not test['matched']:differences.append('geometry_'+str(case['id'])+'_'+z['name'])
        if case['id']==common['case_id']:
            for z in common['pairs']:
                pair=z['pair'];sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6);vals=measure.metrics(sh)['global_values'];err=float(max(abs(np.asarray(vals)-z['source_source_common_adaptive_values'])));ok=err<1e-5;result['adaptive_common_pairs'].append(dict(pair=pair,STEP_STEP_common_adaptive_values=vals,max_numeric_difference=err,matched=ok))
                if not ok:differences.append('adaptive_common_'+pair['lead']+'_'+pair['part'])
        save()
    clean=not git('status','--porcelain=v1');result.update(completed=True,passed=not differences and guard() and clean,all_overlap_locations_reproduced=all(z['overlap_locations_match'] for z in result['key_samples']),all_adaptive_common_pairs_reproduced=all(z['matched'] for z in result['adaptive_common_pairs']),manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,scope='Twoactualwristposes12nativegroups/10sourceSTEPmetrics including3overlap pairs basicvalues plus3preplannedadaptivecommon comparisons,44savedcurve endpoint/tangent/transport/splitquad/radius checks,54materialpoints/4return comparisons.No Newton/contactoptimizer/fullfield rerun/parenthousingSolids/originalCONTACT036native/render/wholeinstallation/physicalforces/friction/preload/tension orwholehandapproval.Originalprescribedfamily failures andoldcomparisonstates preserved.')
    save()
if __name__=='__main__':main()

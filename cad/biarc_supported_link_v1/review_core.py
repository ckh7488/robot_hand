"""Exact-SHA replay of new biarc boundaries and one neutral full-material scene."""
import sys,argparse,subprocess,ast
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import details as d
w,l,a,m,g,c,p,np,ROOT,HERE=d.w,d.l,d.a,d.m,d.g,d.c,d.p,d.np,d.ROOT,d.HERE
measure=d.measure
from OCP.OSD import OSD_ThreadPool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=p.read(ROOT/'checkpoints/BIARC039.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');analysis=p.read(HERE/'reviews/analysis.json');prec=p.read(HERE/'reviews/overlap_precision.json');differences=[];result=dict(completed=False,target_commit=args.commit,differences=differences,native_samples=[],analytic_cases=[],geometry_comparisons=[],chain_comparisons=[],adaptive_common_pairs=[])
    def save():a.save(args.output,result)
    initial=ast.parse((HERE/'history/analysis_link_model_initial.py.txt').read_text(encoding='utf-8-sig'));now=ast.parse((HERE/'link_model.py').read_text(encoding='utf-8-sig'));clean=lambda tree:ast.dump(ast.Module(body=[z for z in tree.body if not isinstance(z,ast.FunctionDef) or z.name!='edge'],type_ignores=[]));assert clean(initial)==clean(now);assert m.drive.sha(HERE/'history/analysis_link_model_initial.py.txt')==analysis['source_sha256']['cad/biarc_supported_link_v1/link_model.py'];result['analytic_source_AST_preserved_except_CAD_edge_array_conversion']=True;save()
    for want in analysis['cases']:
        got=l.geometry(want['q'],want['wrist_flex_deg'],want['wrist_yaw_deg']);errs=[]
        for i in [0,1]:
            for side in ['up','down']:
                left,right=got['lines'][i][side],want['lines'][i][side];errs.extend([abs(left['length_mm']-right['length_mm']),abs(left['minimum_radius_mm']-right['minimum_radius_mm']),np.max(abs(np.asarray(left['join_mm'])-right['join_mm']))])
                for z,t in zip(left['arcs'],right['arcs']):
                    errs.extend([abs(z['length_mm']-t['length_mm']),np.max(abs(np.asarray(z['start'])-t['start'])),np.max(abs(np.asarray(z['end'])-t['end'])),np.max(abs(np.asarray(z['t0'])-t['t0'])),np.max(abs(np.asarray(z['t1'])-t['t1']))]);assert abs(l.edge(t).Length()-t['length_mm'])<1e-5
            errs.append(abs(got['lines'][i]['housing_length_mm']-want['lines'][i]['housing_length_mm']))
        err=float(max(errs));ok=err<1e-5;result['analytic_cases'].append(dict(id=want['id'],max_numeric_difference=err,matched=ok))
        if not ok:differences.append('analytic_'+str(want['id']))
    case=analysis['cases'][0];data,groups=d.inputs(case);saved=c.load(p.read(ROOT/raw['scene'])['parts']);fresh,_,chains=w.build(case);fresh_by={z['name']:z for z in fresh};lookup={z['name']:z for z in saved+[v for _,zz in groups for v in zz]}
    def compare(label,got,want):
        aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']};locations=aa.keys()==bb.keys();err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=locations and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];result['native_samples'].append(dict(id=label,native=got,overlap_locations_match=locations,max_basic_numeric_difference=err,matched=bool(ok)));print(label,ok,flush=True)
        if not ok:differences.append(label)
        save()
    for key,zz in groups:compare(key,c.material(saved,zz),raw['native'][key])
    combine=lambda xs:dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in xs),overlaps=[v for z in xs for v in z['overlaps']],operation_failures=[v for z in xs for v in z['operation_failures']])
    compare('internal',combine([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),combine(raw['internal']))
    selected=[z for z in saved if z['kind']=='housing' and z['segment'] in [3,4,9,10]]
    for i in [0,1]:
        selected.append(next(z for z in saved if z['kind']=='cord' and z['physical_line']==i and z['role']=='middle_0'));selected.append(next(z for z in saved if z['kind']=='cord' and z['physical_line']==i and z['role'].startswith('distal_')))
    for z in selected:
        test=dict(name=z['name'],**measure.match(measure.metrics(fresh_by[z['name']]['shape']),measure.metrics(z['shape'])));result['geometry_comparisons'].append(test)
        if not test['matched']:differences.append('metric_'+z['name'])
        save()
    for got,want in zip(chains,raw['chains']):
        error=max([abs(got['length_mm']-want['length_mm'])]+[max(abs(v[k]-t[k]) for k in ['endpoint_gap_mm','tangent_vector_error']) for v,t in zip(got['joins'],want['joins'])]);ok=error<1e-5 and got['segments']==want['segments'];result['chain_comparisons'].append(dict(line=got['line'],joins=got['joins'],length_mm=got['length_mm'],max_numeric_difference=error,matched=bool(ok)))
        if not ok:differences.append('chain_'+str(got['line']))
    classification=p.read(HERE/'reviews/seam_classification.json');pair=classification['original_record'];left,right=lookup[pair['lead']],lookup[pair['part']];same=left['kind']==right['kind']=='cord' and left['physical_line']==right['physical_line'] and abs(left['segment']-right['segment'])==1;assert same;result['same_physical_cord_adjacent_record_identity_verified']=same
    initial=not differences and guard() and not git('status','--porcelain=v1')
    for ref in prec['source_common_pairs']:
        pair=ref['pair'];sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6);vals=measure.metrics(sh)['global_values'];error=float(np.max(abs(np.asarray(vals)-ref['source_common_adaptive_values'])));ok=error<1e-5;result['adaptive_common_pairs'].append(dict(pair=pair,STEP_STEP_adaptive_values=vals,max_numeric_difference=error,matched=bool(ok)))
    precise=all(z['matched'] for z in result['adaptive_common_pairs']);locations=all(z['overlap_locations_match'] for z in result['native_samples']);other=all(z['matched'] for key in ['analytic_cases','geometry_comparisons','chain_comparisons'] for z in result[key]);result.update(completed=True,passed=initial and precise,initial_core_passed=initial,explicit_results_reproduced=locations and other and precise and guard() and not git('status','--porcelain=v1'),all_overlap_locations_reproduced=locations,adaptive_pairs_passed=precise,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,full_motion_threading_equilibrium_or_hand_approved=False);save()
if __name__=='__main__':main()

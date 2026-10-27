"""Exact-SHA replay of a bounded whole-link study, including its failures."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_link as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
measure=m.drive.module('_guided030_measure',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=p.read(ROOT/'checkpoints/GUIDED030.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');differences=[];analysis=[];geometry=[];native=[]
    for case in raw['analytic_cases']:
        for s in case['solutions']:
            i=s['physical_line'];got=a.describe(case['angles_deg'],i,s['slack_parameter']);e=m.endpoints(case['angles_deg'],i)
            nums=['total_housing_length_mm','upstream_flexible_length_mm','downstream_flexible_length_mm','material_coordinate_at_guide_entry_mm']
            err=max(abs(got[k]-s[k]) for k in nums);anchor=max(float(np.max(abs(np.asarray(got['anchors'][k])-s['anchors'][k]))) for k in e)
            edges=a.edges(s,'housing');joins=[dict(distance_mm=(u.endPoint()-v.startPoint()).Length,tangent_dot=float(u.tangentAt(1).dot(v.tangentAt(0)))) for u,v in zip(edges[:-1],edges[1:])]
            U=np.asarray(s['upstream_controls']);D=np.asarray(s['downstream_controls']);axis=np.array([0,1,0.]);tangent=[float((U[-1]-U[-2])@axis/np.linalg.norm(U[-1]-U[-2])),float((D[1]-D[0])@axis/np.linalg.norm(D[1]-D[0]))]
            ok=err<1e-5 and anchor<1e-5 and abs(got['total_housing_length_mm']-s['target_total_length_mm'])<1e-5 and max(x['distance_mm'] for x in joins)<1e-5 and min(x['tangent_dot'] for x in joins)>1-1e-8
            analysis.append(dict(id=case['id'],line=i,max_numeric_difference=err,endpoint_error_mm=anchor,joins=joins,guide_tangent_dot=tangent,matched=bool(ok)))
            if not ok:differences.append('analysis_'+str(case['id'])+'_'+str(i))
    fresh=a.build(raw['cases'][0]['solutions']);saved=c.load(p.read(ROOT/raw['cases'][0]['scene'])['parts']);lookup={z['name']:z for z in fresh}
    for z in saved:
        got=dict(name=z['name'],**measure.match(measure.metrics(lookup[z['name']]['shape']),measure.metrics(z['shape'])));geometry.append(got)
        if not got['matched']:differences.append('geometry_'+z['name'])
    data,pack,guide,hardware=b.inputs()
    def compare(label,got,want):
        def mapping(x):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']}
        aa,bb=mapping(got),mapping(want);err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.])
        ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        native.append(dict(id=label,native=got,max_numeric_difference=err,matched=bool(ok)));print(label,ok,err,flush=True)
        if not ok:differences.append(label)
    def combined(xs):return dict(pairs_in_scope=sum(x['pairs_in_scope'] for x in xs),overlaps=[z for x in xs for z in x['overlaps']],operation_failures=[z for x in xs for z in x['operation_failures']])
    for case in [raw['cases'][0]]:
        rows=c.load(p.read(ROOT/case['scene'])['parts']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
        for k,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_hardware',hardware)]:compare('open_'+k,c.material(rows,ob),case['native'][k])
        compare('open_internal630',combined([c.material([z],rows[j+1:]) for j,z in enumerate(rows)]),combined(case['internal']))
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,analytic_comparisons=analysis,manifest_files=len(man['files']),frozen_files=len(frozen),checkout_clean=clean,input_hash_guard=guard(),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,
        scope='First whole-link opening pose 5 obstacle groups +630 internal pairs;36 regenerated/source/STEP adaptive measurement signatures atabs1e-5;9 approach/release analytical cases x2lines lengths,actualFKendpoints,joinposition/tangents. No elastic/friction/tension,threading/installation,wristmotion or wholehand approval;not all surface equations/trim/Boolean equivalence. Prior OFFSET024 and HOUSING027 comparison anomalies not retested.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()

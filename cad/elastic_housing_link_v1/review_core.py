"""Exact-SHA replay of saved arc-rod parameters and selected native groups."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_elastic as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
metric=m.drive.module('_elastic033_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool

def reconstruct(case,s):
    e=a.ports(case['angles_deg'],s['physical_line'],case['wrist_flex_deg'],case['wrist_deviation_deg']);x=np.asarray(s['parameters']);up=a.chain(e['A'],e['tA'],x[:a.NUP*2].reshape(-1,2),x[-1]);down=a.chain(e['V'],e['tG'],x[a.NUP*2:-1].reshape(-1,2),a.TOTAL-138.-x[-1]);dif=[]
    for key,value in e.items():dif.append(float(np.max(np.abs(np.asarray(value)-s['anchors'][key]))))
    for got,want in zip(up[2]+down[2],s['upstream_arcs']+s['downstream_arcs']):
        for key in ['start','mid','end','tangent_start','tangent_end','angle_rad','length_mm']:dif.append(float(np.max(np.abs(np.asarray(got[key])-want[key]))))
    residuals=[np.linalg.norm(up[0]-e['U']),np.linalg.norm(down[0]-e['B'])];dots=[np.dot(up[1],e['tG']),np.dot(down[1],e['tB'])];maxdiff=max(dif);reg=min(z['radius_mm'] for z in up[2]+down[2] if z['radius_mm'] is not None)
    return dict(case_id=case['id'],physical_line=s['physical_line'],max_geometry_numeric_difference=float(maxdiff),endpoint_residual_mm=[float(z) for z in residuals],tangent_dot=[float(z) for z in dots],minimum_arc_radius_mm=float(reg),length_mm=float(17+96+25+x[-1]+a.TOTAL-138-x[-1]),matched=bool(maxdiff<1e-5 and max(residuals)<1e-5 and min(dots)>1-1e-8 and reg>1.5))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    man=p.read(ROOT/'checkpoints/ELASTICA033.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();r=p.read(HERE/'reviews/check.json');analytic=[reconstruct(case,s) for case in r['analytic_cases'] if case['admissible'] for s in case['solutions']];differences=['analytic_'+str(z['case_id'])+'_'+str(z['physical_line']) for z in analytic if not z['matched']];native=[];geometry=[];data,pack,guide,hardware=b.inputs()
    def combined(xs):return dict(pairs_in_scope=sum(x['pairs_in_scope'] for x in xs),overlaps=[z for x in xs for z in x['overlaps']],operation_failures=[z for x in xs for z in x['operation_failures']])
    def compare(label,got,want):
        mapping=lambda x:{(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']};aa,bb=mapping(got),mapping(want);err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];native.append(dict(id=label,native=got,max_numeric_difference=err,matched=bool(ok)));print(label,ok,err,flush=True)
        if not ok:differences.append(label)
    for case in r['native_cases']:
        rows=c.load(p.read(ROOT/case['scene'])['parts']);P,H=a.a.wrist_frames(case['wrist_flex_deg'],case['wrist_deviation_deg']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);hand=[a.a.transform(z,H) for z in hand]
        for key,ob in [('hand',hand),('packaging',b.scope.posed_pack(pack,H)),('electrical',data[6]),('arm',a.a.posed_arm(data[7],P,H)),('wrist_hardware',[a.a.transform(z,H) for z in hardware])]:compare(str(case['id'])+'_'+key,c.material(rows,ob),case['native'][key])
        compare(str(case['id'])+'_internal',combined([c.material([z],rows[j+1:]) for j,z in enumerate(rows)]),combined(case['internal']))
        if case['id']==0:
            source={z['name']:z for z in a.build(case['solutions'])}
            selected=[z for z in rows if z['kind']=='housing' and z['segment'] in [0,1,6,12,13,14,21,22]]
            assert len(selected)==16
            for z in selected:
                got=dict(name=z['name'],**metric.match(metric.metrics(source[z['name']]['shape']),metric.metrics(z['shape'])));geometry.append(got)
                if not got['matched']:differences.append('geometry_'+z['name'])
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,analytic_reconstructions=analytic,manifest_files=len(man['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,optimizer_reexecuted=False,calibrated_physics_verified=False,scope='All actually completed native poses x5obstacle groups+internal,16representative open housing source/STEP adaptive metric signatures abs1e-5,all admissible saved rod-parameter arcs/endposes/lengths reconstructed.No SLSQP rerun,all92source/STEP signatures,render/threading/fullphysics/contact/friction/pretension/physical or wholehand approval.Inherited comparison limits untouched.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()

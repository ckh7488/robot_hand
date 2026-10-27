"""Only the five common-material pairs whose basic integrals differed."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--commit',required=True);ap.add_argument('--initial',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();ROOT=args.root.resolve();sys.path.insert(0,str(ROOT/'cad/elastic_housing_link_v1'))
import arc_rod as a
m,g,c,p,np,cq=a.m,a.g,a.c,a.p,a.np,a.cq
metric=m.drive.module('_elastic033_precision_metric',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
man=p.read(ROOT/'checkpoints/ELASTICA033.manifest.json');frozen={}
for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
guard=lambda:all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
assert guard();r=p.read(ROOT/'cad/elastic_housing_link_v1/reviews/check.json');core=p.read(args.initial);assert not core['passed'];case=r['native_cases'][0];meta={z['name']:z for z in p.read(ROOT/case['scene'])['parts']};fresh={};saved={};pairs=[]
def source(name):
    if name in fresh:return fresh[name]
    z=meta[name];s=case['solutions'][z['physical_line']];e=s['anchors'];j=z['segment'];kind=z['kind'];line=lambda x,y:cq.Edge.makeLine(cq.Vector(*x),cq.Vector(*y))
    if j==0:ed=line(e['motor_seat' if kind=='housing' else 'motor_cord'],e['A'])
    elif j<=12:ed=a.edge(s['upstream_arcs'][j-1])
    elif j==13:ed=line(e['U'],e['V'])
    elif j<=21:ed=a.edge(s['downstream_arcs'][j-14])
    else:ed=line(e['B'],e['distal_seat' if kind=='housing' else 'distal_cord'])
    fresh[name]=m.swept(ed,1.5).cut(m.swept(ed,.65)).clean() if kind=='housing' else m.swept(ed,.3);return fresh[name]
def step(name):
    if name not in saved:saved[name]=c.load([meta[name]])[0]['shape']
    return saved[name]
for key in ['hand','internal']:
    wanted=case['native']['hand']['overlaps'] if key=='hand' else [z for t in case['internal'] for z in t['overlaps']];got=next(z for z in core['key_samples'] if z['id']=='0_'+key)['native']['overlaps'];lookup={(z['lead'],z['part']):z for z in got}
    for hit in wanted:
        replay=lookup[(hit['lead'],hit['part'])];basic_delta=abs(hit['material_intersection_mm3']-replay['material_intersection_mm3'])
        if basic_delta<1e-5:continue
        A0,A1=source(hit['lead']),step(hit['lead'])
        if key=='internal':B0,B1=source(hit['part']),step(hit['part'])
        else:
            ob=next(z for z in p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'] if z['name']==hit['part']);B0=c.load([ob])[0]['shape'];N=g.g.v.frames(g.DESIGN,np.zeros(20));B0=g.g.v.relocate(B0,N[ob['body']],N[ob['body']]);B1=B0
        S=A0.intersect(B0,tol=1e-6);T=A1.intersect(B1,tol=1e-6);sv,tv=metric.metrics(S)['global_values'],metric.metrics(T)['global_values'];delta=np.abs(np.asarray(sv)-tv);ok=bool(max(delta)<1e-5)
        pairs.append(dict(group=key,lead=hit['lead'],part=hit['part'],original_report_default_volume_mm3=hit['material_intersection_mm3'],initial_replay_default_volume_mm3=replay['material_intersection_mm3'],initial_default_difference_mm3=basic_delta,source_common_default_volume_mm3=S.Volume(),STEP_common_default_volume_mm3=T.Volume(),source_common_adaptive_values=sv,STEP_common_adaptive_values=tv,absolute_differences=delta.tolist(),matched=ok));print(key,hit['lead'],hit['part'],delta.tolist(),ok,flush=True)
assert len(pairs)==5
clean=not git('status','--porcelain=v1');p.write(args.output,dict(completed=True,target_commit=args.commit,passed=all(z['matched'] for z in pairs) and guard() and clean,pairs=pairs,epsilon=1e-10,absolute_comparison_limit=1e-5,boolean_tolerance_mm=1e-6,reporting_floor_mm3=.001,initial_core_passed=False,all_overlap_locations_and_pairs_reproduced=True,other_native_groups_repeated=False,source_shape_pieces_reconstructed=len(fresh),manifest_files=len(man['files']),frozen_files=len(frozen),checkout_clean=clean,input_hash_guard=guard(),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),geometry_or_thresholds_changed=False,independent_audit=False,whole_hand_drive_approved=False,scope='Only five mismatched source/source versus STEP/STEP native common pairs;one same saved thenar obstacle for handpair.Adaptive common volume/area/centroid epsilon1e-10 at unchanged abs1e-5.No whole6groups/16metrics/22analyses/SLSQP/render repeated.No allbasic-integral equivalence or allsurface/trim/Booleanproof;preserve initialfalse.'))

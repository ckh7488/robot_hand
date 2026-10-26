"""Bounded adaptive-integration comparison at an unchanged exact SHA clone."""
import sys,json,hashlib,subprocess,argparse
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--target',required=True);ap.add_argument('--initial',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
ROOT=args.root.resolve();sys.dont_write_bytecode=True;sys.path.insert(0,str(ROOT/'cad/long_housing_route_v1'));import route_solver as m
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.OSD import OSD_ThreadPool
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.target and not git('status','--porcelain=v1')
initial=read(args.initial);assert initial['differences']==['original_0_arm'] and all(z['matched'] for z in initial['geometry_comparisons']) and sum(z['matched'] for z in initial['key_samples'])==9
raw=read(ROOT/'cad/long_housing_route_v1/reviews/check.json');case=raw['cases'][0];C=m.np.asarray(case['solutions'][0]['controls']);edge=m.cq.Edge.makeBezier([m.cq.Vector(*z) for z in m.segment(C,2/3,1.)]);source=m.swept(edge,1.5).cut(m.swept(edge,.65)).clean();row=next(z for z in read(ROOT/case['scene'])['parts'] if z['name']=='long_housing_0_3');saved=m.c.load([row])[0]['shape'];arm=next(z for z in read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference'] if z['name']=='Palm_wrist_saddle');obstacle=m.cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(arm['asset']+'.step'))).val()
def measure(sh):
 v=GProp_GProps();a=GProp_GProps();ve=BRepGProp.VolumeProperties_s(sh.wrapped,v,1e-10,False,False);ae=BRepGProp.SurfaceProperties_s(sh.wrapped,a,1e-10,False)
 return dict(values=[v.Mass(),a.Mass(),*v.CentreOfMass().Coord()],default_volume_mm3=sh.Volume(),adaptive_volume_error=ve,adaptive_surface_error=ae,valid=sh.isValid(),solids=len(sh.Solids()))
whole=[measure(source),measure(saved)];commons=[measure(source.intersect(obstacle,tol=1e-6)),measure(saved.intersect(obstacle,tol=1e-6))];dwhole=(m.np.asarray(whole[0]['values'])-whole[1]['values']).tolist();dcommon=(m.np.asarray(commons[0]['values'])-commons[1]['values']).tolist();matched=max(abs(x) for x in dwhole+dcommon)<=1e-5;clean=not git('status','--porcelain=v1');man=read(ROOT/'checkpoints/HOUSING027.manifest.json');frozen={}
for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
guard=all(sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(sha(ROOT/z)==h for z,h in frozen.items())
result=dict(completed=True,passed=matched and clean and guard,target_commit=args.target,initial_core_passed=False,initial_sha256=sha(args.initial),initial_preserved=True,comparison_code_sha256=sha(Path(__file__)),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),scope='One failed original long_housing_0_3/Palm_wrist_saddle pair only;fresh source tube fromsavedBeziercontrols andsavedSTEP each nativeintersect samearm.No threshold/CAD/clone changes.No rerun9matchingnativegroups or20matchingselectedshapecomparisons.Not fullsourceBoolean equivalence or force/motion approval.',absolute_metric_tolerance=1e-5,adaptive_relative_epsilon=1e-10,whole_source_STEP_metrics=whole,common_source_STEP_metrics=commons,whole_metric_delta=dwhole,common_metric_delta=dcommon,original_recorded_default_volume_mm3=next(z['material_intersection_mm3'] for z in case['native']['arm']['overlaps'] if z['lead']=='long_housing_0_3'),first_replay_default_volume_mm3=next(z['material_intersection_mm3'] for z in next(z for z in initial['key_samples'] if z['id']=='original_0_arm')['native']['overlaps'] if z['lead']=='long_housing_0_3'),checkout_clean=clean,input_hash_guard=guard,manifest_files=len(man['files']),frozen_files=len(frozen))
args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(passed=result['passed'],common_delta=dcommon,whole_delta=dwhole),ensure_ascii=False),flush=True)

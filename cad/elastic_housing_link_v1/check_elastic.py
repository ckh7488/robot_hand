"""Read every current obstacle and test a supported-port rod candidate."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import arc_rod as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
base=m.drive.module('_elastic033_inputs',ROOT/'cad/long_housing_route_v1/build_check.py')
scope=m.drive.module('_elastic033_pack_frames',ROOT/'cad/wrist_external_feed_v1/correct_wrist_scope.py')
export=m.drive.module('_elastic033_export',ROOT/'cad/wrist_housing_guide_v1/wrist_survey.py').export
from OCP.OSD import OSD_ThreadPool

def inputs():
    data,pack,guide=base.inputs();scene=p.read(ROOT/'cad/wrist_external_feed_v1/generated/assembly_scene.json');hardware=c.load([z for z in scene['new_parts']+scene['retained_parts'] if z['installed_part_candidate']],data[0]);data[0]['cad/wrist_external_feed_v1/generated/assembly_scene.json']=m.drive.sha(ROOT/'cad/wrist_external_feed_v1/generated/assembly_scene.json')
    return data,pack,guide,hardware

def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();target=np.asarray(next(z for z in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'] if z['id']=='parent_full')['angles_deg']);r=dict(completed=False,analytic_cases=[],native_cases=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),total_housing_candidate_length_mm=650.,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,long_housing_installed=False,whole_hand_drive_approved=False,calibrated_physics_or_force_verified=False,new_hardware_rows=0,actual_supports='Motor/distal captured clamps and FEED031 sleeves only;arc joints are integration DOFs,not installed guides.',references=[dict(title='Bergou et al. 2008 Discrete Elastic Rods',url='https://www.cs.columbia.edu/cg/pdfs/143-rods.pdf',used='Naturally straight isotropic curvature-squared bending special case;this implementation uses constant-curvature circular-arc centerline elements,not their full material-frame/twist/dynamics method.')])
    previous=[None,None]
    for j,(factor,flex,yaw) in enumerate([(0.,0,0),(.25,0,0),(.5,0,0),(.75,0,0),(1.,0,0),(.75,0,0),(.5,0,0),(.25,0,0),(0.,0,0),(0.,-20,-15),(0.,20,15)]):
        q=target*factor;s=[a.solve(q,i,flex,yaw,previous[i]) for i in [0,1]];case=dict(id=j,factor=factor,wrist_flex_deg=flex,wrist_deviation_deg=yaw,angles_deg=q.tolist(),solutions=s,admissible=all(v['admissible_endpoint_solution'] for v in s));r['analytic_cases'].append(case);p.write(out,r);print('analytic',j,case['admissible'],[(v['solver_message'],v['endpoint_position_residual_mm'],v['minimum_arc_radius_mm']) for v in s],flush=True)
        if not case['admissible']:break
        previous=s
    if not r['analytic_cases'][0]['admissible']:r.update(completed=True,endpoint_problem_solved=False,native_poses_completed=0,elapsed_s=time.monotonic()-start);p.write(out,r);return
    data,pack,guide,hardware=inputs();r['source_sha256']=data[0]
    planned=[r['analytic_cases'][j] for j in [0,2,4,9,10] if j<len(r['analytic_cases']) and r['analytic_cases'][j]['admissible']]
    for case in planned:
        rows=a.build(case['solutions']);P,H=a.a.wrist_frames(case['wrist_flex_deg'],case['wrist_deviation_deg']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);hand=[a.a.transform(z,H) for z in hand]
        tests={k:c.material(rows,ob) for k,ob in [('hand',hand),('packaging',scope.posed_pack(pack,H)),('electrical',data[6]),('arm',a.a.posed_arm(data[7],P,H)),('wrist_hardware',[a.a.transform(z,H) for z in hardware])]};internal=[c.material([z],rows[j+1:]) for j,z in enumerate(rows)];scene=export(rows,HERE/'generated'/('pose_'+str(case['id'])))
        got=dict(**case,scene=scene,native=tests,internal=internal,clear=all(m.drive.clear(v) for v in [*tests.values(),*internal]));r['native_cases'].append(got);p.write(out,r);print('native',case['id'],got['clear'],{k:len(v['overlaps']) for k,v in tests.items()},'internal',sum(len(v['overlaps']) for v in internal),flush=True)
        if not got['clear']:break
    tests=[v for case in r['native_cases'] for v in [*case['native'].values(),*case['internal']]]
    for n in ['arc_rod.py','check_elastic.py']:r['source_sha256']['cad/elastic_housing_link_v1/'+n]=m.drive.sha(HERE/n)
    r.update(completed=True,endpoint_problem_solved=all(z['admissible'] for z in r['analytic_cases']),native_poses_completed=len(r['native_cases']),planned_native_poses=len(planned),native_candidate_clear=len(r['native_cases'])==len(planned) and all(z['clear'] for z in r['native_cases']),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start);p.write(out,r)
if __name__=='__main__':main()

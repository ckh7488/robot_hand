"""Read current real solids, evaluate one guided whole-link candidate."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import guided_link_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
base=m.drive.module('_guided030_inputs',ROOT/'cad/long_housing_route_v1/build_check.py')
export=m.drive.module('_guided030_export',ROOT/'cad/wrist_housing_guide_v1/wrist_survey.py').export
from OCP.OSD import OSD_ThreadPool

def inputs():
    data,pack,guide=base.inputs();scene=p.read(ROOT/'cad/wrist_outlet_route_v1/inboard/generated/assembly_scene.json')
    hardware=c.load([z for z in scene['new_parts']+scene['retained_parts'] if z['installed_part_candidate']],data[0])
    data[0]['cad/wrist_outlet_route_v1/inboard/generated/assembly_scene.json']=m.drive.sha(ROOT/'cad/wrist_outlet_route_v1/inboard/generated/assembly_scene.json')
    return data,pack,guide,hardware

def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic()
    target=np.asarray(next(z for z in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'] if z['id']=='parent_full')['angles_deg'])
    # Use the longest neutral-bow member in this finite parent family, rather
    # than assuming the opening pose is already long enough for every pose.
    lengths=[max(a.describe(target*f,i,0.)['total_housing_length_mm'] for f in [0.,.25,.5,.75,1.]) for i in [0,1]]
    analytic=[]
    for j,factor in enumerate([0.,.25,.5,.75,1.,.75,.5,.25,0.]):
        q=target*factor;solutions=[a.solve(q,i,lengths[i]) for i in [0,1]]
        analytic.append(dict(id=j,factor=factor,phase='approach' if j<=4 else 'release',angles_deg=q.tolist(),solutions=solutions))
    for i in [0,1]:
        initial=analytic[0]['solutions'][i]['material_coordinate_at_guide_entry_mm']
        for z in analytic:z['solutions'][i]['guide_axial_sliding_from_open_mm']=z['solutions'][i]['material_coordinate_at_guide_entry_mm']-initial
    r=dict(completed=False,analytic_cases=analytic,cases=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
           whole_length_kinematic_family_only=True,elastic_equilibrium_tension_friction_verified=False,guide_axial_material_anchor=False,wrist_motion_tested=False,
           long_housing_installed=False,whole_hand_drive_approved=False,shape_parameters_are_not_installed_guides=True)
    data,pack,guide,hardware=inputs();r['source_sha256']=data[0]
    for z in analytic[:5]:
        rows=a.build(z['solutions']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,z['angles_deg'])
        tests={k:c.material(rows,ob) for k,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_hardware',hardware)]}
        internal=[c.material([v],rows[j+1:]) for j,v in enumerate(rows)]
        scene=export(rows,HERE/'generated'/('pose_'+str(z['id'])))
        got=dict(**z,scene=scene,native=tests,internal=internal,clear=all(m.drive.clear(v) for v in [*tests.values(),*internal]))
        r['cases'].append(got);p.write(out,r);print(z['id'],{k:v['overlaps'] for k,v in tests.items()},'internal',sum(len(v['overlaps']) for v in internal),flush=True)
        if not got['clear']:break
    tests=[t for z in r['cases'] for t in [*z['native'].values(),*z['internal']]]
    for n in ['guided_link_model.py','check_link.py']:r['source_sha256']['cad/guided_housing_link_v1/'+n]=m.drive.sha(HERE/n)
    r.update(completed=True,static_candidate_clear=len(r['cases'])==5 and all(z['clear'] for z in r['cases']),
             native_poses_completed=len(r['cases']),planned_native_poses=5,release_native_executed=False,
             pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),
             overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),
             input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    p.write(out,r);print('finished',r['native_poses_completed'],r['static_candidate_clear'],flush=True)
if __name__=='__main__':main()

import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import inboard_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
base=m.drive.module('_outlet029_inboard_base',ROOT/'cad/long_housing_route_v1/build_check.py')
export=m.drive.module('_outlet029_inboard_export',ROOT/'cad/wrist_housing_guide_v1/wrist_survey.py').export
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'inboard/reviews/check.json';assert not out.exists();new,retained,design=a.build();rows=new+retained;data,pack,guide=base.inputs();scene=export(new,HERE/'inboard/generated/parts');p.write(HERE/'inboard/generated/assembly_scene.json',dict(new_parts=p.read(ROOT/scene)['parts'],retained_parts=[{k:v for k,v in z.items() if k!='shape'} for z in retained],replaced_part=design['replaces'],new_hardware=1,retained_hardware=5,local_references=4))
    r=dict(completed=False,source_sha256=data[0],design=design,scene=scene,internal=[c.material([z],rows[i+1:]) for i,z in enumerate(rows)],cases=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    for case in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases']:
        hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);native={key:c.material(rows,ob) for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]};got=dict(id=case['id'],angles_deg=case['angles_deg'],native=native,clear=all(m.drive.clear(z) for z in native.values()));r['cases'].append(got);p.write(out,r);print(case['id'],{k:z['overlaps'] for k,z in native.items()},flush=True)
        if not got['clear']:break
    tests=r['internal']+[z for case in r['cases'] for z in case['native'].values()];r.update(completed=True,local_eight_static_clear=len(r['cases'])==8 and all(m.drive.clear(z) for z in tests),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests))
    # A conservative carrier cylinder uses the actual existing rotation axis.
    cat=data[2];carrier=next(z for z in cat['hand'] if z['name']=='thumb_root_spread_carrier');F=g.g.v.frames(g.DESIGN,np.zeros(20));origin,R=F['thumb_base_sweep'];axis=R[:,2];assert np.max(abs(axis-[0,0,1]))<1e-8
    envelope=cq.Solid.makeCylinder(18,60,cq.Vector(origin[0],origin[1],-20),cq.Vector(0,0,1));outside=data[4][carrier['name']].cut(envelope,tol=1e-6);outside_volume=outside.Volume() if outside.Solids() else 0.;envelope_test=c.material(new,[dict(name='carrier_R18_conservative_axis_space',shape=envelope)]);r['carrier_rotation_bound']=dict(axis_origin_mm=origin.tolist(),axis_world=axis.tolist(),cylinder_radius_mm=18.,cylinder_z_mm=[-20,40],carrier_material_outside_mm3=outside_volume,source_carrier_contained=outside_volume<1e-8,new_parts_vs_envelope=envelope_test,root_yaw_only_not_whole_hand_motion=True)
    for name in ['outlet_survey.py','inboard_model.py','inboard_check.py']:r['source_sha256']['cad/wrist_outlet_route_v1/'+name]=m.drive.sha(HERE/name)
    p.write(out,r)
if __name__=='__main__':main()

"""Complete aggregation and the new root bound; reuse eight completed poses."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
from outlet_survey import m,g,c,p,np,cq,ROOT,HERE
from OCP.OSD import OSD_ThreadPool

def main():
    raw=HERE/'inboard/reviews/check.json';out=raw.with_name('final.json');assert not out.exists();r=p.read(raw);assert not r['completed'] and len(r['cases'])==8
    expected=p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'];assert [z['id'] for z in r['cases']]==[z['id'] for z in expected] and all(z['angles_deg']==v['angles_deg'] for z,v in zip(r['cases'],expected))
    assert all(m.drive.sha(ROOT/z)==h for z,h in r['source_sha256'].items());tests=r['internal']+[z for case in r['cases'] for z in case['native'].values()];assert all(m.drive.clear(z) for z in tests)
    cat=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');meta=next(z for z in cat['hand'] if z['name']=='thumb_root_spread_carrier');source={};carrier=c.load([meta],source)[0];origin,R=g.g.v.frames(g.DESIGN,np.zeros(20))['thumb_base_sweep'];axis=R[:,2];assert np.max(abs(axis-[0,0,1]))<1e-8
    envelope=cq.Solid.makeCylinder(18,60,cq.Vector(origin[0],origin[1],-20),cq.Vector(0,0,1));outside=carrier['shape'].cut(envelope,tol=1e-6);volume=outside.Volume() if outside.Solids() else 0.;new=c.load(p.read(ROOT/r['scene'])['parts'],source);bound=c.material(new,[dict(name='carrier_R18_conservative_axis_space',shape=envelope)])
    r.update(completed=True,local_eight_static_clear=True,pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=0,operation_failures=0,source_partial_report=str(raw.relative_to(ROOT)),source_partial_sha256=m.drive.sha(raw),static_native_groups_recomputed=0,original_eight_native_cases_unchanged=True,carrier_rotation_bound=dict(axis_origin_mm=origin.tolist(),axis_world=axis.tolist(),cylinder_radius_mm=18.,cylinder_z_mm=[-20,40],carrier_material_outside_mm3=volume,source_carrier_contained=volume<1e-8,new_parts_vs_envelope=bound,root_yaw_only_not_whole_hand_motion=True),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    r['source_sha256'].update(source)
    for name in ['outlet_survey.py','inboard_model.py','history/inboard_check_attempt.py','finalize_inboard.py']:r['source_sha256']['cad/wrist_outlet_route_v1/'+name]=m.drive.sha(HERE/name)
    p.write(out,r);print('final',volume,bound['overlaps'],flush=True)
if __name__=='__main__':main()

"""Read the saved actual blockage relative to the existing guide exit."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/wrist_housing_guide_v1'))
from wrist_survey import m,g,c,p,np,cq
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'reviews/survey.json';assert not out.exists();meta=p.read(ROOT/'cad/wrist_housing_guide_v1/generated/parent_block/scene.json')['parts'];rows=c.load(meta);common=next(z for z in rows if z['name']=='actual_parent_quarter_housing_common');sh=common['shape'];center=np.asarray(sh.Center().toTuple());axis=np.array([1.,1.,0.])/np.sqrt(2.);exit=np.array([33.,-4.,-3.]);bounds=c.m.pack._bbox(sh)
    r=dict(completed=True,source_sha256={z['step']:z['sha256_step'] for z in meta},actual_saved_common=dict(volume_mm3=sh.Volume(),center_mm=center.tolist(),bbox_mm=bounds,axial_center_after_guide_centerline_end_mm=float((center-exit)@axis),guide_centerline_exit_mm=exit.tolist(),projection_note='Axial signed distance from perpendicular sleeve end;not arclength of a new route.'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),new_route_native_tests=0,old_failed_intersection_recalculated=False)
    r['source_sha256']['cad/wrist_outlet_route_v1/outlet_survey.py']=m.drive.sha(Path(__file__));p.write(out,r);print(r['actual_saved_common'],flush=True)
if __name__=='__main__':main()

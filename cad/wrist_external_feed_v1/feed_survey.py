"""Read actual rear wrist solids before choosing a supported feed axis."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/wrist_outlet_route_v1'))
from outlet_survey import m,g,c,p,np,cq
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'reviews/survey.json';assert not out.exists();source={}
    catalog=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand']
    hand=c.load([z for z in catalog if z['name'] in ['Palm_tapered_chassis','thumb_root_fixed_housing','thumb_root_spread_carrier','thumb_segment1_thenar_shell']],source)
    arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        if z['name'] not in ['AK_wrist_flex','AK_wrist_deviation','Wrist_box_carrier','Palm_wrist_saddle']:continue
        path=ROOT/'cad/modular_roots_v9/parts'/(z['asset']+'.step');source[path.relative_to(ROOT).as_posix()]=m.drive.sha(path)
        arm.append(dict(name=z['name'],shape=cq.importers.importStep(str(path)).val()))
    probes=[dict(name='rear_axis_'+str(x)+'_'+str(i),shape=cq.Solid.makeCylinder(2.5,106,cq.Vector(x,-110,z),cq.Vector(0,1,0))) for x in [26.,100.] for i,z in enumerate([-3.,2.])]
    target=next(z for z in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'] if z['id']=='parent_full');N=g.g.v.frames(g.DESIGN,np.zeros(20));cases=[]
    for tag,q in [('open',np.zeros(20)),('parent_full',np.array(target['angles_deg']))]:
        F=g.g.v.frames(g.DESIGN,q);posed=[dict(z,shape=g.g.v.relocate(z['shape'],F[z['body']],N[z['body']])) for z in hand]
        tests={k:c.material(probes,ob) for k,ob in [('selected_hand',posed),('four_wrist_references',arm)]}
        cases.append(dict(id=tag,angles_deg=q.tolist(),native=tests));print(tag,{k:v['overlaps'] for k,v in tests.items()},flush=True)
    origin=np.array([32.,-65.,0.]);envelope=cq.Solid.makeCylinder(60.,110.,cq.Vector(32,-65,-60),cq.Vector(0,0,1));bounds=[]
    for z in arm:
        if z['name'] not in ['Wrist_box_carrier','AK_wrist_deviation']:continue
        sh=z['shape'].cut(envelope,tol=1e-6);bounds.append(dict(name=z['name'],material_outside_cylinder_mm3=sh.Volume() if sh.Solids() else 0.))
    source['cad/wrist_external_feed_v1/feed_survey.py']=m.drive.sha(Path(__file__))
    p.write(out,dict(completed=True,source_sha256=source,material=[dict(name=z['name'],bbox_mm=c.m.pack._bbox(z['shape']),volume_mm3=z['shape'].Volume()) for z in arm+hand],cases=cases,
         proxy_axis_x_mm=[26,100],proxy_y_mm=[-110,-4],proxy_outer_diameter_mm=5,proxies_are_not_installed_parts=True,all_hand_or_packaging_included=False,
         wrist_carrier_rotation_bound=dict(origin_mm=origin.tolist(),axis=[0,0,1],radius_mm=60,z_mm=[-60,50],containment=bounds,external_axis_minimum_radius_mm=68,external_sleeve_outer_radius_mm=2.5,not_entire_bracket_or_wrist_motion_approval=True),
         reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads()))
if __name__=='__main__':main()

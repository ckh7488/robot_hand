"""Inspect the existing wrist opening before proposing any new support."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/long_housing_route_v1'))
import route_solver as m
base=m.drive.module('_wrist028_inputs',ROOT/'cad/long_housing_route_v1/build_check.py')
g,c,p,np,cq=m.g,m.c,m.p,m.np,m.cq
from OCP.OSD import OSD_ThreadPool

def export(rows,folder):
    folder.mkdir(parents=True,exist_ok=False);meta=[]
    for z in rows:
        st=folder/(z['name']+'.step');me=st.with_suffix('.stl')
        cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.15,angularTolerance=.25)
        meta.append(dict(**{k:v for k,v in z.items() if k!='shape'},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=m.drive.sha(st),sha256_stl=m.drive.sha(me)))
    p.write(folder/'scene.json',dict(parts=meta,inspection_only=True));return (folder/'scene.json').relative_to(ROOT).as_posix()

def main():
    start=time.monotonic();out=HERE/'reviews/survey.json';assert not out.exists()
    data,pack,guide=base.inputs();hand,v,F,N=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20))
    wrist=[z for z in data[7] if z['name'] in ['Wrist_box_carrier','Palm_wrist_saddle']]
    probes=[]
    for i,x in enumerate([26.,34.]):
        sh=cq.Solid.makeCylinder(1.5,36,cq.Vector(x,-18,-11),cq.Vector(0,1,0))
        probes.append(dict(name='window_space_probe_'+str(i),shape=sh,body='palm',color=[.7,.25,.85],installed_part=False))
    tests={k:c.material(probes,z) for k,z in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]}
    samples=[]
    for z in wrist:
        planes=[]
        for y in [-9.,-3.,3.]:
            inside=[]
            for x in range(16,45,2):
                for zz in range(-32,11,2):
                    if z['shape'].isInside(cq.Vector(x,y,zz),1e-6):inside.append([x,zz])
            planes.append(dict(y_mm=y,xz_material_samples_mm=inside))
        samples.append(dict(name=z['name'],bbox_mm=c.m.pack._bbox(z['shape']),volume_mm3=z['shape'].Volume(),planes=planes))
    report=dict(completed=True,source_sha256=data[0],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,source_window=dict(part='Palm_wrist_saddle',nominal_x_mm=[20,40],nominal_y_mm=[-6,0],nominal_z_mm=[-28,6],corner_radius_mm=3,opening_from_original_source_not_new_cut=True),wrist_material=samples,probes=dict(centers_xz_mm=[[26,-11],[34,-11]],y_mm=[-18,18],outside_diameter_assumed_mm=3,solid_space_proxies_not_installed_housings=True),native=tests,probe_scene=export(probes,HERE/'generated/survey_probes'),wrist_scene=export([dict(z,color=[.5,.6,.7],installed_part=False) for z in wrist],HERE/'generated/wrist_reference'),whole_hand_drive_complete=False,guide_installed=False,elapsed_s=time.monotonic()-start)
    report['source_sha256']['cad/wrist_housing_guide_v1/wrist_survey.py']=m.drive.sha(Path(__file__))
    p.write(out,report);print({k:z['overlaps'] for k,z in tests.items()},flush=True)
if __name__=='__main__':main()

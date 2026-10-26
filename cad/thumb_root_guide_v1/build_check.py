import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import guide_model as g
from OCP.OSD import OSD_ThreadPool
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT

def export(folder,parts,extra):
    folder.mkdir(parents=True,exist_ok=False);saved=[]
    for z in parts:
        st=folder/(z['name']+'.step');me=st.with_suffix('.stl');cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.035,angularTolerance=.1)
        saved.append(dict(**{k:v for k,v in z.items() if k not in ['shape','local_shape']},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=g.g.drive.sha(st),sha256_stl=g.g.drive.sha(me),bbox_mm=m.pack._bbox(z['shape'])))
    p.write(folder/'scene.json',dict(parts=saved,**extra));return (folder/'scene.json').relative_to(ROOT).as_posix()

def inputs(design):
    source=design['source_sha256'];_,v,cat,scene,hand=p.load()
    for z in scene['hand']:source[z['step']]=z['sha256_step'];source[z['mesh']]=z['sha256_stl']
    mount=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json')
    packaging=c.load(mount['retained_parts']+mount['parts'],source)
    wires=c.load(p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'],source)
    arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+z['asset']+'.step';source[rel]=g.g.drive.sha(ROOT/rel)
        arm.append(dict(name=z['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    return source,v,cat,scene,hand,packaging,wires,arm

def evaluate(q,parts,design,data):
    source,v,cat,scene,hand,pack,wires,arm=data;neutral=v.frames(cat['design'],np.zeros(20));frames=v.frames(cat['design'],q)
    fresh=[dict(z,shape=v.relocate(z['shape'],frames[z['body']],neutral[z['body']])) for z in parts]
    retained=[dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand'] if z['name'] not in design['replaced_hand_parts']]
    cords=g.cord_pieces(q);tests=dict(new_hand=c.material(fresh,retained),new_packaging=c.material(fresh,pack),new_wires=c.material(fresh,wires),new_arm=c.material(fresh,arm),
        cords_hand=c.material(cords,retained),cords_new=c.material(cords,fresh),cords_packaging=c.material(cords,pack),cords_wires=c.material(cords,wires),cords_arm=c.material(cords,arm),
        between_cords=c.material([z for z in cords if z['cord']==0],[z for z in cords if z['cord']==1]))
    return dict(angles_deg=list(map(float,q)),native=tests,nominal_geometry_clear=all(g.g.drive.clear(z) for z in tests.values()),local_cord_length_mm=[sum(z['length_mm'] for z in cords if z['cord']==i) for i in range(2)]),fresh,cords

def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();parts,design=g.build()
    report=dict(completed=False,design=design,source_sha256=design['source_sha256'],parts_count=len(parts),cases=[],reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),upstream_route_complete=False,whole_drive_complete=False)
    def save():p.write(out,report)
    report['scene']=export(HERE/'generated/parts',parts,dict(design=design,installed_candidate_only=True,upstream_root_route_complete=False,reference_hand='cad/motion_clearance_v2/generated/assembly_catalog.json',reference_packaging='cad/motor_spool_coupling_v1/selected/generated/scene.json'))
    data=inputs(design);report['source_sha256']=data[0]
    for rel in ['cad/thumb_root_guide_v1/guide_model.py','cad/thumb_root_guide_v1/build_check.py','cad/thumb_tendon_route_v1/route_model.py','cad/gesture_hand_v3/generated/build_report.json']:
        data[0][rel]=g.g.drive.sha(ROOT/rel)
    report['new_pairs']=[]
    for i,z in enumerate(parts):report['new_pairs'].append(c.material([z],parts[i+1:]))
    target=next(z for z in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases'] if z['id']=='pinch25')['angles_deg']
    cases=[('open',np.zeros(20)),('ip45',[0.]*19+[45.]),('ip85',[0.]*19+[85.]),('ip_minus15',[0.]*19+[-15.])]+[('parent_'+str(n),np.asarray(target)*f) for n,f in [('quarter',.25),('mid',.5),('three_quarter',.75),('full',1.)]]
    save()
    for label,q in cases:
        got,fresh,cords=evaluate(q,parts,design,data);got['id']=label;got['cord_scene']=export(HERE/'generated'/label,cords,dict(angles_deg=list(map(float,q)),distal_segment_only=True,motor_to_inlet_connected=False))
        report['cases'].append(got);save();print(label,{k:len(z['overlaps']) for k,z in got['native'].items()},flush=True)
    report.update(completed=True,input_hash_guard=all(g.g.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start,
        all_nominal_snapshots_clear=all(z['nominal_geometry_clear'] for z in report['cases']) and all(g.g.drive.clear(z) for z in report['new_pairs']))
    save();assert report['input_hash_guard'];print('guide20 done',report['all_nominal_snapshots_clear'],flush=True)
if __name__=='__main__':main()

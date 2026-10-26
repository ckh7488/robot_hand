"""Finite native checks for newly supported distal housing end stops."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import distal_model as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
def export(folder,parts):
    folder.mkdir(parents=True,exist_ok=False);meta=[]
    for z in parts:
        st=folder/(z['name']+'.step');me=st.with_suffix('.stl');cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.035,angularTolerance=.1)
        meta.append(dict(**{k:v for k,v in z.items() if k not in ['shape','local_shape']},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=m.drive.sha(st),sha256_stl=m.drive.sha(me),bbox_mm=c.m.pack._bbox(z['shape'])))
    p.write(folder/'scene.json',dict(parts=meta));return (folder/'scene.json').relative_to(ROOT).as_posix()
def inputs(design):
    data=m.base.check.inputs(design);source=data[0]
    scene=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json')
    pack=c.load(scene['new_parts']+scene['retained_parts'],source)
    for name in ['distal_model.py','check.py']:
        source['cad/thumb_sheath_terminal_v1/'+name]=m.drive.sha(HERE/name)
    source['cad/thumb_guide_assembly_v2/radial_model.py']=m.drive.sha(ROOT/'cad/thumb_guide_assembly_v2/radial_model.py')
    return data,pack
def hand_rows(data,guide,q):
    _,v,cat,scene,hand=data[:5];neutral=v.frames(cat['design'],np.zeros(20));f=v.frames(cat['design'],q)
    rows=[dict(name=z['name'],shape=v.relocate(hand[z['name']],f[z['body']],neutral[z['body']])) for z in scene['hand'] if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]
    rows += [dict(name=z['name'],shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) for z in guide]
    return rows,v,f,neutral
def main():
    out=HERE/'reviews/static.json';assert not out.exists();start=time.monotonic();new,guide,design=m.build();data,pack=inputs(design);housing=m.witnesses()
    report=dict(completed=False,design=design,source_sha256=data[0],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,internal=[],cases=[],long_route_complete=False)
    def save():p.write(out,report)
    report['scene']=export(HERE/'generated/parts',new);report['housing_scene']=export(HERE/'generated/housing_reference',housing)
    for i,z in enumerate(new):report['internal'].append(c.material([z],new[i+1:]))
    cases=p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases']
    for case in cases:
        q=case['angles_deg'];hand,v,f,neutral=hand_rows(data,guide,q)
        moving=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) for z in new+housing]
        tests={key:c.material(moving,obstacles) for key,obstacles in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]}
        got=dict(id=case['id'],angles_deg=q,native=tests,clear=all(m.drive.clear(x) for x in tests.values()));report['cases'].append(got);save();print(case['id'],got['clear'],{k:len(v['overlaps']) for k,v in tests.items()},flush=True)
    report.update(completed=True,static_candidate_clear=all(x['clear'] for x in report['cases']) and all(m.drive.clear(z) for z in report['internal']),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start);save();print('done',report['static_candidate_clear'],flush=True)
if __name__=='__main__':main()

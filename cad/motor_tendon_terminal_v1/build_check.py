"""Check current complete obstacles; failed candidates remain separate outputs."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import terminal_model as model
g,c,p,np,cq,HERE,ROOT=model.g,model.c,model.p,model.np,model.cq,model.HERE,model.ROOT
from OCP.OSD import OSD_ThreadPool
def export(folder,parts,extra=None):
    folder.mkdir(parents=True,exist_ok=False);meta=[]
    for row in parts:
        st=folder/(row['name']+'.step');me=st.with_suffix('.stl');cq.exporters.export(row['shape'],str(st));cq.exporters.export(row['shape'],str(me),tolerance=.035,angularTolerance=.1)
        meta.append(dict(**{k:v for k,v in row.items() if k!='shape'},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=model.drive.sha(st),sha256_stl=model.drive.sha(me),bbox_mm=c.m.pack._bbox(row['shape'])))
    p.write(folder/'scene.json',dict(parts=meta,**(extra or {})));return (folder/'scene.json').relative_to(ROOT).as_posix()
def inputs(design):
    data=model.prior.check.inputs(design);source=data[0]
    source['cad/motor_tendon_terminal_v1/build_check.py']=model.drive.sha(Path(__file__))
    source['cad/thumb_guide_assembly_v2/material_cache.py']=model.drive.sha(ROOT/'cad/thumb_guide_assembly_v2/material_cache.py')
    retained=[z for z in data[5] if z['name'] not in design['replaced_packaging']];assert len(retained)==366
    guide=c.load(p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'],source)
    return data,retained,guide
def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();parts,design=model.build();data,retained,guide=inputs(design)
    report=dict(completed=False,design=design,source_sha256=data[0],reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),tests={},internal=[],hand_cases=[],hand_complete=False,upstream_connection_complete=False)
    def save():p.write(out,report)
    report['scene']=export(HERE/'generated/parts',parts,dict(design=design,retained_packaging_count=366,total_packaging_candidate_count=373,candidate_only=True))
    cords=model.cords();witness=model.witnesses();report['cord_scene']=export(HERE/'generated/cords',cords);report['housing_scene']=export(HERE/'generated/housing_reference',witness,dict(complete_housing_route=False))
    for i,z in enumerate(parts):report['internal'].append(c.material([z],parts[i+1:]))
    coupling,_=model.drive.build();envelopes,_=model.drive.custom_envelopes(coupling)
    for label,a,b in [('new_packaging',parts,retained),('new_wires',parts,data[6]),('new_arm',parts,data[7]),('full_coupling_rotation',envelopes,parts),('cords_new',cords,parts),('cords_packaging',cords,retained),('cords_wires',cords,data[6]),('housing_new',witness,parts),('housing_packaging',witness,retained),('housing_wires',witness,data[6]),('housing_arm',witness,data[7])]:
        report['tests'][label]=c.material(a,b);save();print(label,len(report['tests'][label]['overlaps']),flush=True)
    v,cat,scene,hand=data[1:5];neutral=v.frames(cat['design'],np.zeros(20));old=p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')
    for label in ['open','parent_full']:
        q=next(z['angles_deg'] for z in old['cases'] if z['id']==label);frames=v.frames(cat['design'],q)
        shapes=[dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand'] if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]
        shapes += [dict(z,shape=v.relocate(z['shape'],frames[z['body']],neutral[z['body']])) for z in guide]
        report['hand_cases'].append(dict(id=label,angles_deg=q,native=c.material(parts+cords+witness,shapes)));save()
    report.update(completed=True,static_candidate_clear=all(model.drive.clear(z) for z in list(report['tests'].values())+report['internal']+[z['native'] for z in report['hand_cases']]),input_hash_guard=all(model.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    save();assert report['input_hash_guard'];print('terminal complete',report['static_candidate_clear'],flush=True)
if __name__=='__main__':main()

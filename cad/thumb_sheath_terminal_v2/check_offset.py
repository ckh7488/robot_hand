"""New actual supports and upstream channel, bounded native pose tests."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import offset_model as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
prior=m.drive.module('_offset024_prior_check',ROOT/'cad/thumb_sheath_terminal_v1/check.py')
from OCP.OSD import OSD_ThreadPool
def inputs(design):
    data,pack=prior.inputs(design)
    for n in ['offset_model.py','check_offset.py']:data[0]['cad/thumb_sheath_terminal_v2/'+n]=m.drive.sha(HERE/n)
    return data,pack
def export(folder,parts):return prior.export(folder,parts)
def main():
    out=HERE/'reviews/static.json';assert not out.exists();start=time.monotonic();parts,guide,design=m.build();data,pack=inputs(design);housing=m.witnesses();cords=m.extension_cords()
    r=dict(completed=False,design=design,source_sha256=data[0],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,cases=[],internal=[],whole_hand_drive_complete=False)
    def save():p.write(out,r)
    r['scene']=export(HERE/'generated/parts',parts);r['housing_scene']=export(HERE/'generated/housing_reference',housing);r['cord_scene']=export(HERE/'generated/cord_extension',cords)
    for j,z in enumerate(parts):r['internal'].append(c.material([z],parts[j+1:]))
    r['housing_internal']=c.material(housing,parts+guide);r['extension_internal']=c.material(cords,parts+guide+housing)
    for case in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases']:
        q=case['angles_deg'];hand,v,f,neutral=prior.hand_rows(data,guide,q)
        moving=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) for z in parts+housing+cords]
        tests={key:c.material(moving,obstacles) for key,obstacles in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]}
        got=dict(id=case['id'],angles_deg=q,native=tests,clear=all(m.drive.clear(z) for z in tests.values()));r['cases'].append(got);save();print(case['id'],got['clear'],{k:len(z['overlaps']) for k,z in tests.items()},flush=True)
    r.update(completed=True,static_candidate_clear=all(z['clear'] for z in r['cases']) and all(m.drive.clear(z) for z in r['internal']+[r['housing_internal'],r['extension_internal']]),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start);save();print('offsetdone',r['static_candidate_clear'],flush=True)
if __name__=='__main__':main()

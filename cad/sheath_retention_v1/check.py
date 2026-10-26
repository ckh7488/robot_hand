"""Bounded four-terminal retention candidate, preserving every failed pair."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import retention_model as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
def export(rows):
    folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=False);meta=[]
    for z in rows:
        st=folder/(z['name']+'.step');me=st.with_suffix('.stl');cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.03,angularTolerance=.1)
        meta.append(dict(**{k:v for k,v in z.items() if k!='shape'},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=m.drive.sha(st),sha256_stl=m.drive.sha(me)))
    p.write(folder/'scene.json',dict(parts=meta,candidate_only=True));return (folder/'scene.json').relative_to(ROOT).as_posix()
def inputs(design):
    data,pack,hand,original,new,housing=m.s.setup()
    guide=[z for z in original if z['name'] not in ['liner_flange_keeper_0','liner_flange_keeper_1']]+[z for z in new if z['name'] not in design['replaced_distal_parts']]
    motorhousing=c.load(p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/housing_reference/scene.json')['parts'],data[0])
    for n in ['retention_model.py','check.py']:data[0]['cad/sheath_retention_v1/'+n]=m.drive.sha(HERE/n)
    return data,pack,guide,housing+motorhousing
def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();fresh,design=m.build();data,pack,guide,housing=inputs(design)
    r=dict(completed=False,source_sha256=data[0],design=design,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,internal=[],cases=[],long_housing_connected=False,cord_threading_tested=False)
    r['scene']=export(fresh)
    def save():p.write(out,r)
    for j,z in enumerate(fresh):r['internal'].append(c.material([z],fresh[j+1:]));save()
    for case in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases']:
        hand,v,f,neutral=m.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
        moving=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) if z['body']!='forearm' else z for z in fresh]
        hous=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) if z['body']!='forearm' else z for z in housing]
        tests={key:c.material(moving,ob) for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('housing_reference',hous)]}
        r['cases'].append(dict(id=case['id'],angles_deg=case['angles_deg'],native=tests,clear=all(m.drive.clear(z) for z in tests.values())));save();print(case['id'],{k:len(z['overlaps']) for k,z in tests.items()},flush=True)
    tests=r['internal']+[t for a in r['cases'] for t in a['native'].values()]
    r.update(completed=True,static_candidate_clear=all(m.drive.clear(z) for z in tests),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),native_AABB_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start);save();print('retention',r['static_candidate_clear'],flush=True)
if __name__=='__main__':main()

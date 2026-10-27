"""New distal bend candidate against all current parts, at eight hand states."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import entry_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool

def moved(z,q):
    v=g.g.v;O0,R0=v.frames(g.DESIGN,np.zeros(20))['thumb_mcp'];O,R=v.frames(g.DESIGN,q)['thumb_mcp'];F=np.eye(4);F[:3,:3]=R@R0.T;F[:3,3]=O-F[:3,:3]@O0
    return dict(z,shape=g.original.transform(z['shape'],F))

def inputs():
    data,pack,guide=a.prior.base.inputs();guide=[z for z in guide if z['name']!=a.REPLACED];s=p.read(ROOT/'cad/paired_rear_guide_v1/generated/assembly_scene.json');hardware=c.load([z for z in s['new_parts']+s['retained_parts'] if z['installed_part_candidate']],data[0]);return data,pack,guide,hardware

def main():
    out=HERE/'reviews/static.json';assert not out.exists();start=time.monotonic();new,design=a.build();data,pack,guide,hardware=inputs();data[0].update(design['source_sha256']);scene=a.prior.export(new,HERE/'generated/parts');p.write(HERE/'generated/assembly_scene.json',dict(new_parts=p.read(ROOT/scene)['parts'],replaced_hand_parts=[a.REPLACED],current_wrist_hardware_scene='cad/paired_rear_guide_v1/generated/assembly_scene.json',whole_link_installed=False))
    r=dict(completed=False,source_sha256=data[0],design=design,scene=scene,cases=[],internal=[c.material([z],new[j+1:]) for j,z in enumerate(new)],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,whole_hand_drive_approved=False,whole_flexible_link_connected=False,prior_straight_housing_references_not_duplicated=True)
    for case in a.prior.scope.b.cases()[:8]:
        hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);moving=[moved(z,case['angles_deg']) for z in new];tests={k:c.material(moving,ob) for k,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_guide',hardware)]};got=dict(**case,native=tests,clear=all(m.drive.clear(v) for v in tests.values()));r['cases'].append(got);p.write(out,r);print(case['id'],got['clear'],{k:v['overlaps'] for k,v in tests.items()},flush=True)
        if not got['clear']:break
    tests=[*r['internal'],*[v for z in r['cases'] for v in z['native'].values()]]
    for n in ['entry_model.py','check_entry.py']:r['source_sha256']['cad/thumb_entry_bend_v1/'+n]=m.drive.sha(HERE/n)
    r.update(completed=True,sampled_candidate_clear=len(r['cases'])==8 and all(m.drive.clear(z) for z in tests),static_cases_completed=len(r['cases']),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in r['source_sha256'].items()),elapsed_s=time.monotonic()-start);p.write(out,r)
if __name__=='__main__':main()

"""Move the 39 actual palm-mounted packaging rows with the hand base.

The original run is retained. Its neutral-wrist hand cases are reusable, but
its wrist cases kept those 39 rows fixed and are not approval evidence.
"""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_feed as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
from OCP.OSD import OSD_ThreadPool

def posed_pack(pack,H):
    palm=[z for z in pack if z.get('body')=='palm'];fixed=[z for z in pack if z.get('body')!='palm']
    assert len(palm)==39 and len(fixed)==346
    assert {z['module'] for z in palm}=={'palm_carrier','thumb_base_sweep_actuator','thumb_opposition_actuator','middle_mcp_actuator','ring_mcp_actuator'}
    return [a.transform(z,H) for z in palm]+fixed

def main():
    out=HERE/'reviews/final.json';assert not out.exists();raw=p.read(HERE/'reviews/check.json');assert raw['completed']
    assert len(raw['cases'])>=8 and all(z['clear'] for z in raw['cases'][:8])
    assert all(m.drive.sha(ROOT/z)==v for z,v in raw['source_sha256'].items())
    sourcecase=b.cases()[:8]
    for got,want in zip(raw['cases'][:8],sourcecase):
        assert all(got[k]==v for k,v in want.items()) and got['wrist_flex_deg']==got['wrist_deviation_deg']==0
    data,pack,guide=b.base.inputs();scene=p.read(HERE/'generated/assembly_scene.json');rows=c.load(scene['new_parts']+scene['retained_parts'],data[0]);assert len(rows)==10
    r=dict(raw,completed=False,cases=raw['cases'][:8].copy(),neutral_hand_native_cases_reused=8,new_corrected_wrist_cases=0,original_wrist_cases_not_approval=True,original_wrist_error='39 palm-motor/support rows were kept in world instead of hand-base frame.',source_sha256=dict(raw['source_sha256']))
    for case in b.cases()[8:]:
        P,H=a.wrist_frames(case['wrist_flex_deg'],case['wrist_deviation_deg']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);hand=[a.transform(z,H) for z in hand];moving=[a.transform(z,H) for z in rows]
        tests={k:c.material(moving,ob) for k,ob in [('hand',hand),('packaging',posed_pack(pack,H)),('electrical',data[6]),('arm',a.posed_arm(data[7],P,H))]}
        got=dict(**case,native=tests,clear=all(m.drive.clear(v) for v in tests.values()));r['cases'].append(got);r['new_corrected_wrist_cases']+=1;p.write(out,r);print(case['id'],{k:v['overlaps'] for k,v in tests.items()},flush=True)
        if not got['clear']:break
    tests=[*r['internal'],*[v for z in r['cases'] for v in z['native'].values()]]
    r['source_sha256']['cad/wrist_external_feed_v1/correct_wrist_scope.py']=m.drive.sha(Path(__file__))
    r.update(completed=True,sampled_candidate_clear=len(r['cases'])==16 and all(m.drive.clear(z) for z in tests),static_cases_completed=len(r['cases']),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in r['source_sha256'].items()))
    p.write(out,r)
if __name__=='__main__':main()

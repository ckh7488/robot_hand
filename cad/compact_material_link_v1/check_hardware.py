"""New guide-vs-hardware poses only; preserve closed PACK049 exact states."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import compact_model as a
n,k,np,ROOT,HERE=a.n,a.k,a.np,a.ROOT,a.HERE


def case_equal(left,right):
    return all(np.max(abs(np.array(left[key])-right[key]))<1e-12 for key in ['q','P','H','F'])


def main():
    out=HERE/'reviews/hardware.json';assert not out.exists();start=time.monotonic()
    raw=n.p.read(HERE/'reviews/continuous.json');assert raw['completed'];prior=n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    ctx,hw=a.context();source=dict(ctx[0][0]);names={z['name'] for z in hw};tested=[];inherited=[];states=[]
    result=dict(completed=False,source_sha256=source,states=states,inherited_exact_static_states=inherited,
        new_rigid_guide_rows=3,other_hardware_rows=859,materials_excluded_because_already_tested_against_all_hardware=True,
        closed_PACK049_native_reexecuted=0,other_859_internal_pairs_reexecuted=0)
    def save():n.a.save(out,result)
    for state in raw['states']:
        match=next((p for p in prior['states'] if case_equal(state['case'],p['case'])),None)
        duplicate=next((p for p in tested if case_equal(state['case'],p['case'])),None)
        if match or duplicate:
            inherited.append(dict(id=state['id'],from_report='PACK049' if match else 'LINK050',from_id=(match or duplicate)['id'],
                same_case_q_P_H_F=True,new_hardware_input_files_unchanged=True));save();continue
        groups=a.pose(ctx,hw,state['case']);moving=[z for _,rows in groups for z in rows if z['name'] in names]
        got=dict(id=state['id'],case=state['case'],native={});states.append(got);save()
        for key,rows in groups:got['native'][key]=n.c.material(moving,[z for z in rows if z['name'] not in names]);save()
        got['native']['internal']=k.flat([n.c.material([z],moving[j+1:]) for j,z in enumerate(moving)])
        got['clear']=all(n.m.drive.clear(v) for v in got['native'].values());tested.append(got);save();print(state['id'],got['clear'],flush=True)
    for f in ['check_hardware.py','reviews/continuous.json']:source[(HERE/f).relative_to(ROOT).as_posix()]=n.m.drive.sha(HERE/f)
    source['cad/compact_turnback_pack_v1/reviews/check.json']=n.m.drive.sha(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    tests=[v for z in states for v in z['native'].values()];agg=k.flat(tests)
    result.update(completed=True,pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
        newly_checked_guide_hardware_poses=len(states),inherited_or_duplicate_poses=len(inherited),sampled_guide_hardware_clear=all(z['clear'] for z in states),
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save()


if __name__=='__main__':main()

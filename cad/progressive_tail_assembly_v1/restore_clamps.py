"""Restore the actual clamp parts only after both whole tails reached target."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_progressive as k
n,s,t,a,m,g,c,p,np,ROOT,HERE=k.n,k.s,k.t,k.a,k.m,k.g,k.c,k.p,k.np,k.ROOT,k.HERE


def main():
    out=HERE/'reviews/restoration.json';assert not out.exists();raw=p.read(HERE/'reviews/check.json')
    assert raw['completed'] and raw['forming_sequence_clear']
    case,data,full,groups,hw=k.old.inputs(p.read(ROOT/'cad/motor_guide_prethread_v1/reviews/check.json'))
    last=raw['cases'][-1];built=[n.build(case,i,last['formed_mm'][i],hw[i]) for i in [0,1]]
    housing=sum([z[0] for z in built],[]);mobile=[z[1] for z in built];pack=dict(full)['packaging']
    parts=[[next(z for z in pack if z['name']=='motor_housing_clamp_'+str(i)+'_'+name) for name in ['jaw_1','nut_0','nut_1','bolt_0','bolt_1']] for i in [0,1]]
    installed=[];report=dict(completed=False,source_sha256=dict(raw['source_sha256']),paths=[],final=None,formed_scene=last['scene'],
                             manual_loose_parts_holding=True,thread_or_tool_or_torque_or_clamp_force_validated=False)
    for rel in ['cad/progressive_tail_assembly_v1/restore_clamps.py','cad/progressive_tail_assembly_v1/reviews/check.json']:
        report['source_sha256'][rel]=m.drive.sha(ROOT/rel)
    def save():a.save(out,report)
    ok=True
    for i in [0,1]:
        for item in parts[i]:
            axis=s.b.frame(i)[:3,2]*(-1 if '_nut_' in item['name'] else 1)
            distances=[14.,7.,2.,0.] if '_jaw_' in item['name'] else [10.,5.,0.]
            path=dict(moving=item['name'],line=i,installed_before=[z['name'] for z in mobile+installed],
                      missing_before=[z['name'] for z in sum(parts,[]) if z['name'] not in [q['name'] for q in installed]],samples=[])
            report['paths'].append(path)
            for j,d in enumerate(distances):
                row=dict(item,shape=item['shape'].translate(tuple(axis*d)))
                tests={key:c.material([row],zz) for key,zz in groups}
                tests['housings_and_installed_clamps']=c.material([row],housing+mobile+installed)
                ok=all(m.drive.clear(z) for z in tests.values());sample=dict(distance_mm=d,native=tests,clear=ok);path['samples'].append(sample);save()
                if j in [0,len(distances)-1] or not ok:sample['scene']=k.export([row],'restore_'+item['name']+'_'+str(j));save()
                print(item['name'],d,ok,flush=True)
                if not ok:break
            if not ok:break
            installed.append(item)
        if not ok:break
    if ok:
        tests={key:c.material(housing,zz) for key,zz in full}
        tests['internal']=k.old.k.combine([c.material([z],housing[j+1:]) for j,z in enumerate(housing)])
        report['final']=dict(native=tests,clear=all(m.drive.clear(z) for z in tests.values()),scene=k.export(housing,'final_housings'),
                             restored_packaging_names=sorted(z['name'] for z in pack));ok=report['final']['clear']
    agg=k.old.k.combine([z for path in report['paths'] for sample in path['samples'] for z in sample['native'].values()]+list(report['final']['native'].values() if report['final'] else []))
    report.update(completed=True,conditional_CAD_sequence_clear=ok,pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],
                  overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
                  input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in report['source_sha256'].items()),physical_installation_approved=False)
    save()


if __name__=='__main__':main()

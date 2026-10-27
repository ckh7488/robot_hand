"""Independent changed-geometry relations while preserving the failed pad inclusion."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm074_native',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/continuation_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    start=time.monotonic();rows,membership,source=m.context();initial=n.p.read(HERE/'reviews/check.json')
    new=n.c.load(n.p.read(ROOT/initial['scene'])['parts'],source)[0];fixed=[z for z in rows if z['name']!=m.OLD];assert len(fixed)==885
    for f in [Path(__file__),HERE/'reviews/check.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw=dict(initial,source_sha256=source,completed=False,native={},native_contracts=[],new_native_groups=0,
        preserved_initial_M3_pad_inclusion_false=True,initial_attachment_queries_repeated=0,
        continuation_is_independent_geometry_relations_not_attachment_approval=True)
    def save():n.a.save(out,raw)
    def test(label,left,right,leftM=None,rightFrames=None):
        q=n.c.material(left,right);raw['native'][label]=q;raw['native_contracts'].append(dict(id=label,left_names=[z['name'] for z in left],right_names=[z['name'] for z in right],left_transform=np.eye(4) if leftM is None else leftM,right_frame_transforms=rightFrames))
        save();print(label,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
        if q['overlaps'] or q['operation_failures']:raw['first_new_blocked_group']=label;save();return False
        return True
    passed=True;save()

    if passed:passed=test('new_whole_support__all_retained885_neutral',[new],fixed)
    old59=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    if passed:
        names=m.a.a.CLOCK_NAMES|{'thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer'};output=[z for z in fixed if z['name'] in names];assert len(output)==9
        for state in old59['output_command_checks']:
            C=np.array(state['transform']);moving=m.p.moved(output,C)
            if not test('output_'+str(int(state['command_deg']))+'__new_support',moving,[new],C):passed=False;break
    if passed:
        arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
        for state in old59['arm_motion']:
            F=d.transforms(state['angles_deg'],arm);moving=m.p.moved([new],F['forearm'])
            obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')])) for z in fixed if membership.get(z['name'],'forearm') in state['compared_frames']]
            if not test(state['id']+'__new_support_relative_bodies',moving,obstacles,F['forearm'],{z['name']:F[membership.get(z['name'],'forearm')] for z in obstacles}):passed=False;break
    total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_native_clear=not total['overlaps'] and not total['operation_failures'],all_planned_geometry_relations_completed=passed,
        whole_candidate_approved=False,elapsed_s=time.monotonic()-start,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()))
    save();print('completed native',raw['new_native_groups'],'records',raw['overlap_records'],'pad false preserved',flush=True)


if __name__=='__main__':main()
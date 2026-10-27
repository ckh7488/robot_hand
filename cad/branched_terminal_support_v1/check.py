"""Check the whole replacement against all current proposal parts and relative motions."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm074_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    start=time.monotonic();rows,membership,source=m.context();new,design=m.build(source);fixed=[z for z in rows if z['name']!=m.OLD];assert len(fixed)==885
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    scene=n.a.a.prior.export([new],HERE/'generated/hardware');old73=n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json')
    raw=dict(completed=False,source_sha256=source,scene=scene,design=design,native={},native_contracts=[],attachment_queries=m.queries(new,source),
        original16packets=m.p.packets(rows),original_final886_names=sorted(z['name'] for z in rows),new_final886_names=sorted([z['name'] for z in fixed]+[m.NAME]),
        initial_world_present_names=old73['packet_final_present_names'],pending_original_names=old73['packet_final_pending_names'],restored_modules=old73['restored_modules'],new_restored_modules=[],
        current862_replaced=False,proposal886_adopted=False,new_support_geometry=1,other885_unchanged=True,old_native_groups_repeated=0,old_common_Boolean_repeated=0,
        previous_49_and92_assembly_to_changed_support_inherited=False,service_paths_executed=0,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1)
    def save():n.a.save(out,raw)
    def test(label,left,right,leftM=None,rightFrames=None):
        q=n.c.material(left,right);raw['native'][label]=q;raw['native_contracts'].append(dict(id=label,left_names=[z['name'] for z in left],right_names=[z['name'] for z in right],left_transform=np.eye(4) if leftM is None else leftM,right_frame_transforms=rightFrames))
        save();print(label,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
        if q['overlaps'] or q['operation_failures']:raw['first_new_blocked_group']=label;save();return False
        return True
    passed=all(q['volume_mm3']<1e-8 and q['solids']==0 for q in raw['attachment_queries']);save()
    if not passed:raw['first_failed_attachment']=next(q['id'] for q in raw['attachment_queries'] if q['solids'] or q['volume_mm3']>=1e-8)
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
        old_support_bbox=n.c.m.pack._bbox(next(z['shape'] for z in rows if z['name']==m.OLD)),new_support_bbox=n.c.m.pack._bbox(new['shape']),
        candidate_assembly_world48_front7_not_executed=True,conditional_world670_unchanged=True,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('completed',raw['new_native_groups'],'records',raw['overlap_records'],'attachments',passed,flush=True)


if __name__=='__main__':main()

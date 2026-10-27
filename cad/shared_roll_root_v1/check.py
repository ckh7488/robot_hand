"""Changed shared rear unit, preserved pads, and conditional cap/fastener assembly."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm065_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    start=time.monotonic();before,membership,source=a.context();new,entry,inherit,design,region,plate=a.build(before)
    fixed=[z for z in before if z['name'] not in a.OLD];assert len(fixed)==880
    prior=n.p.read(ROOT/'cad/roll_bolted_interface_v1/reviews/check.json')
    allentry=inherit+entry;outer=n.c.load(n.p.read(ROOT/prior['outer_scene'])['parts'],source)
    oldentry=n.c.load(n.p.read(ROOT/prior['entry_scene'])['parts'],source)
    q060=n.p.read(ROOT/'cad/rotation_reserved_support_v1/reviews/check.json');bounds=n.c.load(n.p.read(ROOT/q060['protection_scene'])['parts'],source)[:9]
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/roll_bolted_interface_v1/reviews/check.json',ROOT/'cad/roll_bolted_interface_v1/reviews/evidence.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw=dict(completed=False,source_sha256=source,design=design,scene=n.a.a.prior.export(new,HERE/'generated/hardware'),
        entry_scene=n.a.a.prior.export(entry,HERE/'generated/entry_reference'),unchanged_entry4_names=[z['name'] for z in inherit],
        reused_outer_scene=prior['outer_scene'],reused_output_entry6=[z['name'] for z in oldentry if z['name'].startswith('arm064_output_')],
        reused_protection_scene=q060['protection_scene'],native={},rear_pad_queries=[],arm_motion=[],assembly=[],
        OCC_pool_threads=1,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,current862_replaced=False,proposal886_adopted=False,
        previous_stage_native_groups_repeated=0,stator7_reference_and_110E_records_inherited_not_retested=True)
    def save():n.a.save(out,raw)
    def run(label,left,right):
        q=n.c.material(left,right);raw['native'][label]=q;save();print(label,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True);return q
    save();run('new6__retained880',new,fixed)
    raw['native']['new6_internal15']=d.k.flat([n.c.material([z],new[i+1:]) for i,z in enumerate(new[:-1])]);save()
    run('entry8__retained880',allentry,fixed);run('entry8__new6',allentry,new)
    run('new_structure2__source_outer2',new[:2],outer)
    run('existing_rotation_bounds9__new_structure2',bounds,new[:2])
    run('retained_output_entry6__new_structure2',[z for z in oldentry if z['name'].startswith('arm064_output_')],new[:2])
    for z in a.rear_pads(before):
        q=z['shape'].cut(new[0]['shape'],tol=1e-6);assert q.isValid()
        raw['rear_pad_queries'].append(dict(id=z['id'],input_volume_mm3=z['shape'].Volume(),outside_mm3=q.Volume() if q.Solids() else 0.,outside_solids=len(q.Solids())))
    old=next(z['shape'] for z in before if z['name']==a.p.a.CARRIER)
    missing=old.cut(new[0]['shape'],tol=1e-6).cut(region,tol=1e-6)
    pd=plate.cut(new[0]['shape'],tol=1e-6)
    raw['preservation']=dict(old_material_missing_outside_whole_replacement_region_mm3=missing.Volume() if missing.Solids() else 0.,
        output_plate_material_missing_mm3=pd.Volume() if pd.Solids() else 0.,queries_valid=missing.isValid() and pd.isValid())
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in prior['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in new]
        obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')])) for z in fixed if membership.get(z['name'],'forearm')!='forearm']
        run(state['id']+'__new_shared_unit_relative',moving,obstacles)
        raw['arm_motion'].append(dict(id=state['id'],angles_deg=state['angles_deg'],transforms=F))
    # Conditional service: beam, motors, front clamp, electrical hardware and
    # output bolts already present. Only shared cap and its eight fasteners absent.
    assembly_names=[a.CAP]+[z['name'] for z in new[2:]]+[z['name'] for z in fixed if z['name'].startswith(('arm064_clamp_M3x12_','arm064_clamp_nut_'))]
    assert len(assembly_names)==9
    final=fixed+new;lookup={z['name']:z for z in final};present=[z for z in final if z['name'] not in assembly_names]
    raw['assembly_initial_absent_names']=assembly_names;raw['assembly_initial_present_names']=[z['name'] for z in present]
    hardware=d.k.flat([raw['native']['new6__retained880'],raw['native']['new6_internal15']])
    blocked=bool(hardware['overlaps'] or hardware['operation_failures'])
    order=[a.CAP]+[k for k in assembly_names if 'nut' in k]+[k for k in assembly_names if 'M3x12' in k]
    for path_index,name in enumerate(order if not blocked else []):
        sign=-1. if 'M3x12' in name else 1.
        for step,distance in enumerate([14.,7.,0.] if name==a.CAP else [20.,10.,0.]):
            M=np.eye(4);M[2,3]=sign*distance;row=dict(lookup[name],shape=n.g.original.transform(lookup[name]['shape'],M))
            label='assembly_'+str(path_index)+'_'+str(step);q=run(label,[row],present)
            raw['assembly'].append(dict(id=label,path_index=path_index,name=name,translation_mm=M[:3,3],
                present_names=[z['name'] for z in present],absent_names=[k for k in assembly_names if k not in {z['name'] for z in present}],
                manual_support_before_fastening=True))
            if q['overlaps'] or q['operation_failures']:blocked=True;break
        if blocked:break
        present.append(lookup[name])
    total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],
        AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),
        raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],whole_proposal_raw_clear=False,
        assembly_completed=not blocked and len(present)==886,assembly_final_present_names=[z['name'] for z in present],
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('complete',raw['native_groups'],raw['overlap_records'],raw['operation_failures'],'assembly',raw['assembly_completed'],flush=True)


if __name__=='__main__':main()

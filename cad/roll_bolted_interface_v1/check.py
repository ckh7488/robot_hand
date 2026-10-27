"""New mounting hardware and declared access only; retain every raw contact."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm064_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    start=time.monotonic();before,membership,source=a.context();new,entry,reserved,design=a.build(before)
    fixed=[z for z in before if z['name'] not in a.OLD];assert len(fixed)==869
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/mounting_unit_interface_basis_v1/reviews/check.json',ROOT/'cad/mounting_unit_interface_basis_v1/reviews/interface_followup.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw=dict(completed=False,source_sha256=source,design=design,scene=n.a.a.prior.export(new,HERE/'generated/hardware'),
        entry_scene=n.a.a.prior.export(entry,HERE/'generated/entry_reference'),outer_scene=n.a.a.prior.export(reserved,HERE/'generated/outer_reference'),
        native={},arm_motion=[],OCC_pool_threads=1,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        current862_replaced=False,proposal888_adopted=False,old_completed_groups_repeated=0,existing_nominal_motor_not_removed_from_raw_native=True)
    def save():n.a.save(out,raw)
    def run(label,left,right):
        q=n.c.material(left,right);raw['native'][label]=q;save();print(label,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
    save()
    run('new19__retained869',new,fixed)
    raw['native']['new19_internal171']=d.k.flat([n.c.material([z],new[i+1:]) for i,z in enumerate(new[:-1])]);save()
    run('entry16__retained869',entry,fixed)
    run('entry16__new19',entry,new)
    run('structural3__source_outer_references2',new[:3],reserved)
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for sid,angles in [('roll40',[0,40,0,0]),('roll_minus40',[0,-40,0,0]),('combined55',[55,40,-20,15]),('combined110',[110,-40,20,-15])]:
        F=d.transforms(angles,arm)
        for frame in ['elbow','forearm']:
            moving=[dict(z,shape=n.g.original.transform(z['shape'],F[frame])) for z in new if z['body']==frame]
            obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')])) for z in fixed if membership.get(z['name'],'forearm')!=frame]
            obstacles += [dict(z,shape=n.g.original.transform(z['shape'],F[z['body']])) for z in new if z['body']!=frame]
            run(sid+'__new_'+frame+'_relative',moving,obstacles)
        raw['arm_motion'].append(dict(id=sid,angles_deg=angles,transforms=F))
    raw['seat_distances_mm']=[dict(left=new[0]['name'],right='case_rear_plane',value_mm=abs(max(a.n.c.m.pack._bbox(new[0]['shape'])[1][1],-261)+261)),
        dict(left=new[1]['name'],right='Forearm_box_beam',value_mm=new[1]['shape'].distance(next(z['shape'] for z in fixed if z['name']=='Forearm_box_beam')))]
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],
        overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_clear=not total['overlaps'] and not total['operation_failures'],
        assembly_native_executed=False,assembly_gate='Neutral hardware/access records retained. No assembly path pass claimed for statically blocked integrated proposal.',
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('complete',raw['native_groups'],raw['overlap_records'],raw['operation_failures'],flush=True)


if __name__=='__main__':main()

"""Current CAD route candidates, motion snapshots and unsolved length residuals."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import route_model as g
from OCP.OSD import OSD_ThreadPool
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT
PLANS=[dict(id='rear_turn',entry_y=-12.,turn_y=-60.,rise_y=-30.),
       dict(id='front_turn',entry_y=-12.,turn_y=5.,rise_y=25.),
       dict(id='front_joint_entry',entry_y=-8.,turn_y=5.,rise_y=25.)]


def inputs():
    source={z:g.drive.sha(ROOT/z) for z in ['cad/thumb_tendon_route_v1/route_model.py','cad/thumb_tendon_route_v1/audit.py',
        'cad/motor_spool_coupling_v1/candidate2.py','cad/motor_spool_coupling_v1/model.py',
        'cad/motor_spool_coupling_v1/selected/generated/scene.json','cad/palm_motor_layout_v1/selected/generated/layout.json',
        'cad/functional_motion_v3/generated/catalog.json','cad/object_pinch_v2/reviews/audit.json',
        'cad/thumb_ip_drive_v1/routing.py','cad/thumb_ip_drive_v1/source_docs/tendon/facts.json']}
    _,v,cat,scene,hand=p.load()
    for row in scene['hand']:source[row['step']]=row['sha256_step'];source[row['mesh']]=row['sha256_stl']
    fixed,wires,arm,retained,replaced=g.drive.fixed_inputs(source)
    source['cad/remote_motor_lead_v2/generated/scene.json']=g.drive.sha(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')
    local,design=g.drive.build()
    return source,v,cat,scene,hand,fixed,wires,arm,local


def evaluate(q,plan,data):
    source,v,cat,scene,hand,fixed,wires,arm,local=data
    kwargs={k:z for k,z in plan.items() if k!='id'};rows,kin=g.coordinated(q,**kwargs);cords=g.tubes(rows)
    neutral=v.frames(cat['design'],np.zeros(20));frames=v.frames(cat['design'],q)
    current=[dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand']]
    motors=g.drive.moved(local,kin['motor_angle_deg'])
    tests=dict(packaging=c.material(cords,fixed+motors),wires=c.material(cords,wires),neutral_arm=c.material(cords,arm),
        hand=c.material(cords,current),between_cords=c.material([z for z in cords if z['cord']==0],[z for z in cords if z['cord']==1]))
    report=dict(angles_deg=list(map(float,q)),plan=plan,kinematics=kin,native=tests,
        rope_geometric_clear=all(g.drive.clear(z) for z in tests.values()),cord_pieces=len(cords),
        guide_loci=[dict(cord=z['cord'],loci=z['guide_loci']) for z in rows],
        intermediate_guides_installed=False,forces_solved=False,
        caveat='Pose-specific free circular guide loci,not installed pulley CAD or fixed guide-axis constraints.Original motor anchor tail has26.565degree tangent entry change,no fatigue qualification.')
    return report,cords,rows


def export_case(label,cords,rows):
    folder=HERE/'generated'/label;folder.mkdir(parents=True,exist_ok=False);saved=[]
    for z in cords:
        st=folder/(z['name']+'.step');me=folder/(z['name']+'.stl');cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.04,angularTolerance=.12)
        saved.append(dict(**{k:v for k,v in z.items() if k!='shape'},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),
            sha256_step=g.drive.sha(st),sha256_stl=g.drive.sha(me),bbox_mm=m.pack._bbox(z['shape'])))
    p.write(folder/'scene.json',dict(parts=saved,cord_count=2,cord_nominal_diameter_mm=.6,
        guide_loci=[dict(cord=z['cord'],loci=z['guide_loci']) for z in rows],guide_parts_installed=False,tensioner_installed=False,
        reference_drive_scene='cad/motor_spool_coupling_v1/selected/generated/scene.json',hand_reference='cad/motion_clearance_v2/generated/assembly_catalog.json',
        hand_geometry_changed=False,motor_positions_changed=False,whole_force_path_complete=False))
    return (folder/'scene.json').relative_to(ROOT).as_posix()


def main():
    out=HERE/'reviews/audit.json';assert not out.exists();started=time.monotonic();data=inputs()
    report=dict(completed=False,source_sha256=data[0],plans=[],selected=None,cases=[],parent_length_study=[],
        hand_geometry_changed=False,motor_positions_changed=False,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),guide_hardware_complete=False,tensioner_complete=False,
        whole_hand_drive_complete=False,physical_approved=False)
    def save():p.write(out,report)
    selected=None;fallback=None
    for plan in PLANS:
        got,cords,rows=evaluate(np.zeros(20),plan,data);got['id']=plan['id'];report['plans'].append(got)
        print(plan['id'],{k:len(z['overlaps']) for k,z in got['native'].items()},flush=True)
        got['scene']=export_case(plan['id'],cords,rows);fallback=(plan,got)
        save()
        if got['rope_geometric_clear']:selected=plan;report['selected']=plan['id'];break
    # Even if no neutral route clears, evaluate the last candidate as an
    # unadopted study. Preserve obstructions rather than hiding the parent task.
    plan=selected or fallback[0]
    for label,q in [('ip45',[0.]*19+[45.]),('ip_minus15',[0.]*19+[-15.])]:
        got,cords,rows=evaluate(q,plan,data);got['id']=label;got['scene']=export_case(label,cords,rows);report['cases'].append(got);save()
        print(label,got['rope_geometric_clear'],flush=True)
    target=next(z for z in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases'] if z['id']=='pinch25')['angles_deg']
    for fraction in [i/16 for i in range(17)]:
        q=np.asarray(target)*fraction;rows,kin=g.coordinated(q,**{k:z for k,z in plan.items() if k!='id'})
        report['parent_length_study'].append(dict(fraction=fraction,angles_deg=q.tolist(),kinematics=kin,guide_loci=[z['guide_loci'] for z in rows]))
    for label,fraction in [('parent_mid',.5),('pinch25',1.)]:
        got,cords,rows=evaluate(np.asarray(target)*fraction,plan,data);got['id']=label;got['scene']=export_case(label,cords,rows);report['cases'].append(got);save()
        print(label,got['rope_geometric_clear'],flush=True)
    length=[z['kinematics']['common_length_residual_mm'] for z in report['parent_length_study']]
    report['common_length_range_mm']=[min(length),max(length)];report['ideal_two_to_one_compensator_stroke_mm']=(max(length)-min(length))/2.
    report['length_caveat']='Only this pose-dependent free-guide candidate,not a manufactured fixed-pulley or Bowden assembly.The ideal2:1 stroke is a geometric planning quantity,not a selected spring or proof every route needs this travel.Motor rotation solves differential length only;common residual and guide constraints remain.'
    report.update(completed=True,input_hash_guard=all(g.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-started)
    save();print('complete',report['selected'],report['common_length_range_mm'],flush=True);assert report['input_hash_guard']


if __name__=='__main__':main()

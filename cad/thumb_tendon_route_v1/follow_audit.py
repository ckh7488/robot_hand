"""One bounded route correction, preserving original failures and free-guide limits."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import follow_parent as g
import audit as prior
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT
from OCP.OSD import OSD_ThreadPool


def evaluate(q,data):
    source,v,cat,scene,hand,fixed,wires,arm,local=data;rows,kin=g.coordinated(q);cords=g.tubes(rows)
    neutral=v.frames(cat['design'],np.zeros(20));frames=v.frames(cat['design'],q)
    current=[dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand']]
    motors=g.drive.moved(local,kin['motor_angle_deg'])
    tests=dict(packaging=c.material(cords,fixed+motors),wires=c.material(cords,wires),neutral_arm=c.material(cords,arm),
        hand=c.material(cords,current),between_cords=c.material([z for z in cords if z['cord']==0],[z for z in cords if z['cord']==1]))
    result=dict(angles_deg=list(map(float,q)),kinematics=kin,native=tests,rope_geometric_clear=all(g.drive.clear(z) for z in tests.values()),
        cord_pieces=len(cords),guide_loci=[dict(cord=z['cord'],loci=z['guide_loci']) for z in rows],
        guide_hardware_installed=False,tensioner_installed=False,pose_dependent_guide_centers=True,
        meaning='Free-guide envelope study only.These pose-dependent fillet centers are not a solved fixed-pulley/guide mount mechanism.Clear geometry does not establish taut fixed-length cables or force transmission.')
    return result,cords,rows


def main():
    out=HERE/'reviews/follow.json';assert not out.exists();started=time.monotonic();data=prior.inputs()
    for name in ['follow_parent.py','follow_audit.py']:data[0]['cad/thumb_tendon_route_v1/'+name]=g.drive.sha(HERE/name)
    data[0]['cad/thumb_tendon_route_v1/reviews/audit.json']=g.drive.sha(HERE/'reviews/audit.json')
    report=dict(completed=False,source_sha256=data[0],cases=[],parent_length_study=[],
        hand_geometry_changed=False,motor_positions_changed=False,guide_hardware_complete=False,tensioner_complete=False,whole_hand_drive_complete=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():p.write(out,report)
    target=next(z for z in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases'] if z['id']=='pinch25')['angles_deg']
    for label,q in [('open',np.zeros(20)),('ip45',[0.]*19+[45.]),('parent_quarter',np.asarray(target)*.25),
                    ('parent_mid',np.asarray(target)*.5),('parent_three_quarter',np.asarray(target)*.75),('pinch25',target)]:
        try:
            got,cords,rows=evaluate(q,data);got['id']=label;got['scene']=prior.export_case('follow_'+label,cords,rows);report['cases'].append(got)
            print(label,got['rope_geometric_clear'],{k:len(z['overlaps']) for k,z in got['native'].items()},flush=True)
        except AssertionError as exc:
            report['cases'].append(dict(id=label,angles_deg=list(map(float,q)),geometry_error=repr(exc),rope_geometric_clear=False));print(label,repr(exc),flush=True)
        save()
    for fraction in [i/16 for i in range(17)]:
        q=np.asarray(target)*fraction
        try:
            rows,kin=g.coordinated(q);report['parent_length_study'].append(dict(fraction=fraction,angles_deg=q.tolist(),kinematics=kin,guide_loci=[z['guide_loci'] for z in rows]))
        except AssertionError as exc:report['parent_length_study'].append(dict(fraction=fraction,geometry_error=repr(exc)))
    valid=[z for z in report['parent_length_study'] if 'kinematics' in z];values=[z['kinematics']['common_length_residual_mm'] for z in valid]
    report['common_length_range_mm']=[min(values),max(values)];report['ideal_two_to_one_compensator_stroke_mm']=(max(values)-min(values))/2.
    report['sampled_free_guide_envelopes_clear']=all(z['rope_geometric_clear'] for z in report['cases']) and len(valid)==17
    report['length_study_meaning']='Residual and hypothetical2:1travel apply only to these pose-dependent free-guide curves.Guide centers move with the constructed bends;they are not an installed fixed/gimballed guide design.No spring/preload/taut-force equilibrium approval.Do not infer structural impossibility of other routes.'
    report.update(completed=True,input_hash_guard=all(g.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-started)
    save();print('follow complete',report['sampled_free_guide_envelopes_clear'],report['common_length_range_mm'],flush=True);assert report['input_hash_guard']


if __name__=='__main__':main()

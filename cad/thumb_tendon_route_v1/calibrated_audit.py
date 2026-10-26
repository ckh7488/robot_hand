"""Preflight then native snapshots for one calibrated outside-parent route."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import calibrated as g
import audit as prior
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT
from OCP.OSD import OSD_ThreadPool


def evaluate(q,data):
    source,v,cat,scene,hand,fixed,wires,arm,local=data;rows,kin=g.coordinated(q);cords=g.tubes(rows)
    neutral=v.frames(cat['design'],np.zeros(20));frames=v.frames(cat['design'],q)
    current=[dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand']]
    motors=g.drive.moved(local,kin['motor_angle_deg'])
    tests=dict(packaging=c.material(cords,fixed+motors),wires=c.material(cords,wires),neutral_arm=c.material(cords,arm),hand=c.material(cords,current),
        between_cords=c.material([z for z in cords if z['cord']==0],[z for z in cords if z['cord']==1]))
    return dict(angles_deg=list(map(float,q)),kinematics=kin,native=tests,rope_geometric_clear=all(g.drive.clear(z) for z in tests.values()),cord_pieces=len(cords),
        guide_loci=[dict(cord=z['cord'],loci=z['guide_loci']) for z in rows],guide_hardware_installed=False,tensioner_installed=False,
        meaning='Free-guide locus envelopes only;no fixed guide-axis/mount or taut force-equilibrium claim.'),cords,rows


def main():
    out=HERE/'reviews/calibrated.json';assert not out.exists();started=time.monotonic()
    report=dict(completed=False,source_sha256={},geometry_preflight=[],cases=[],parent_length_study=[],
        motor_neutral_phase_deg=g.CALIBRATION_DEG,guide_hardware_complete=False,tensioner_complete=False,whole_hand_drive_complete=False,
        hand_geometry_changed=False,motor_positions_changed=False,hand_angles_changed=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():p.write(out,report)
    target=next(z for z in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases'] if z['id']=='pinch25')['angles_deg']
    for f in [i/16 for i in range(17)]:
        q=np.asarray(target)*f
        try:
            rows,kin=g.coordinated(q);report['geometry_preflight'].append(dict(fraction=f,valid=True));report['parent_length_study'].append(dict(fraction=f,angles_deg=q.tolist(),kinematics=kin,guide_loci=[z['guide_loci'] for z in rows]))
        except AssertionError as exc:report['geometry_preflight'].append(dict(fraction=f,valid=False,error=repr(exc)))
    save()
    if not all(z['valid'] for z in report['geometry_preflight']):
        report.update(completed=True,native_cases_not_run=True);save();print('calibrated preflight failed',flush=True);return
    data=prior.inputs();report['source_sha256']=data[0]
    for name in ['follow_parent2.py','calibrated.py','calibrated_audit.py']:data[0]['cad/thumb_tendon_route_v1/'+name]=g.drive.sha(HERE/name)
    for rel in ['cad/thumb_tendon_route_v1/reviews/audit.json','cad/palm_motor_layout_v1/reviews/check2.json',
        'cad/motor_spool_coupling_v1/selected/reviews/check.json','cad/form_candidate_v1/generated/scene.json']:
        data[0][rel]=g.drive.sha(ROOT/rel)
    for label,q in [('open',np.zeros(20)),('parent_mid',np.asarray(target)*.5),('ip45',[0.]*19+[45.]),
        ('parent_quarter',np.asarray(target)*.25),('parent_three_quarter',np.asarray(target)*.75),('pinch25',target)]:
        got,cords,rows=evaluate(q,data);got['id']=label;got['scene']=prior.export_case('calibrated_'+label,cords,rows);report['cases'].append(got);save()
        print(label,got['rope_geometric_clear'],{k:len(z['overlaps']) for k,z in got['native'].items()},flush=True)
    values=[z['kinematics']['common_length_residual_mm'] for z in report['parent_length_study']]
    report['common_length_range_mm']=[min(values),max(values)];report['ideal_two_to_one_compensator_stroke_mm']=(max(values)-min(values))/2.
    report['sampled_free_guide_envelopes_clear']=all(z['rope_geometric_clear'] for z in report['cases'])
    report['length_study_meaning']='Only these pose-dependent free-guide curves and engineering rest phase.Each cable is a connected centreline split into CAD pieces.Guide supports/fixed axes,sheaths,tension adjustment and actual cable-force constraints are absent;geometry0 is not installed drive success.'
    report.update(completed=True,input_hash_guard=all(g.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-started)
    save();print('calibrated complete',report['sampled_free_guide_envelopes_clear'],report['common_length_range_mm'],flush=True);assert report['input_hash_guard']


if __name__=='__main__':main()

"""Actual common solids and endpoint contact/guide-constraint diagnostics."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import calibrated as g
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def main():
    report=p.read(HERE/'reviews/calibrated.json');assert report['completed'] and report['input_hash_guard']
    out=HERE/'generated/visual_evidence';out.mkdir(parents=True,exist_ok=False)
    handmeta={z['name']:z for z in p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand']}
    neutral=g.v.frames(g.DESIGN,np.zeros(20));source={};evidence=[]
    initial=p.read(HERE/'reviews/audit.json');bad=next(z for z in initial['cases'] if z['id']=='parent_mid');F=g.v.frames(g.DESIGN,bad['angles_deg'])
    cordmap={z['name']:z for z in c.load(p.read(ROOT/bad['scene'])['parts'],source)}
    for i,hit in enumerate(bad['native']['hand']['overlaps']):
        row=handmeta[hit['part']];shape=c.load([row],source)[0]['shape'];shape=g.v.relocate(shape,F[row['body']],neutral[row['body']])
        common=cordmap[hit['lead']]['shape'].intersect(shape,tol=1e-6);st=out/('original_mid_common_'+str(i)+'.step');me=st.with_suffix('.stl')
        cq.exporters.export(common,str(st));cq.exporters.export(common,str(me),tolerance=.02,angularTolerance=.1)
        evidence.append(dict(name='actual_mid_common_'+str(i),source_case='initial_parent_mid',step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=g.drive.sha(st),sha256_stl=g.drive.sha(me),
            actual_common_volume_mm3=common.Volume(),pair=[hit['lead'],hit['part']],color=[.9,.06,.1],body='palm',installed_part=False))
    latest=next(z for z in report['cases'] if z['id']=='parent_mid');F=g.v.frames(g.DESIGN,latest['angles_deg'])
    cordmap={z['name']:z for z in c.load(p.read(ROOT/latest['scene'])['parts'],source)}
    for i,hit in enumerate(latest['native']['hand']['overlaps']):
        row=handmeta[hit['part']];shape=c.load([row],source)[0]['shape'];shape=g.v.relocate(shape,F[row['body']],neutral[row['body']])
        common=cordmap[hit['lead']]['shape'].intersect(shape,tol=1e-6);st=out/('calibrated_mid_common_'+str(i)+'.step');me=st.with_suffix('.stl')
        cq.exporters.export(common,str(st));cq.exporters.export(common,str(me),tolerance=.02,angularTolerance=.1)
        evidence.append(dict(name='calibrated_mid_common_'+str(i),source_case='calibrated_parent_mid',step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=g.drive.sha(st),sha256_stl=g.drive.sha(me),
            actual_common_volume_mm3=common.Volume(),pair=[hit['lead'],hit['part']],color=[.9,.06,.1],body='palm',installed_part=False))
    p.write(out/'scene.json',dict(parts=evidence,angles_deg=bad['angles_deg'],evidence_only=True))
    contacts=[];local,_=g.drive.build();selftests=[];continuity=[]
    for case in report['cases']:
        saved=c.load(p.read(ROOT/case['scene'])['parts'],source);totals=[]
        rows,_=g.coordinated(case['angles_deg'])
        continuity.append(dict(case=case['id'],max_centerline_join_error_mm=max(float(np.linalg.norm(a['z']-z['a'])) for row in rows for a,z in zip(row['pieces'],row['pieces'][1:])),
            fixed_motor_anchor_tangent_change_deg=26.56505117707799,force_or_bend_life_approved=False))
        for index in range(2):
            group=[z for z in saved if z['cord']==index]
            for i,z in enumerate(group):
                totals.append(c.material([z],group[i+2:]))
        selftests.append(dict(case=case['id'],pairs_in_scope=sum(z['pairs_in_scope'] for z in totals),exact_pairs=sum(z['exact_pairs'] for z in totals),
            overlaps=[q for z in totals for q in z['overlaps']],operation_failures=[q for z in totals for q in z['operation_failures']],
            scope='All nonadjacent pieces of each same physical cord;adjacent shared-boundary portions are one cord,not separate colliding parts.'))
    for label in ['open','ip45']:
        case=next(z for z in report['cases'] if z['id']==label);q=case['angles_deg'];F=g.v.frames(g.DESIGN,q)
        cords=c.load(p.read(ROOT/case['scene'])['parts'],source);motors=g.drive.moved(local,case['kinematics']['motor_angle_deg']);motor={z['name']:z['shape'] for z in motors}
        target={}
        for name in ['thumb_segment3_tip_core','thumb_ip_tendon_clamp']:
            row=handmeta[name];shape=c.load([row],source)[0]['shape'];target[name]=g.v.relocate(shape,F[row['body']],neutral[row['body']])
        for index in range(2):
            for role,part,shape in [('motor_wrap','thumb_ip_two_tail_spool',motor['thumb_ip_two_tail_spool']),
                ('motor_anchor_tail','thumb_ip_tail_clamp_'+str(index+1),motor['thumb_ip_tail_clamp_'+str(index+1)]),
                ('joint_wrap','thumb_segment3_tip_core',target['thumb_segment3_tip_core']),('joint_anchor_tail','thumb_ip_tendon_clamp',target['thumb_ip_tendon_clamp'])]:
                cord=next(z for z in cords if z['cord']==index and z['role']==role)
                contacts.append(dict(case=label,cord=index,role=role,part=part,distance_mm=cord['shape'].distance(shape)))
    baseline=report['parent_length_study'][0];checks=[]
    # A wrist guide would need a constant palm-frame center. Distal guides
    # would need a constant MCP-parent-frame center; compare both explicitly.
    for cord in range(2):
        for i in range(6):
            points=[]
            for sample in report['parent_length_study']:
                point=np.asarray(sample['guide_loci'][cord][i]['center_mm']);frame='palm' if i<3 else 'thumb_mcp'
                O,R=g.v.frames(g.DESIGN,sample['angles_deg'])[frame];points.append(R.T@(point-O))
            shifts=[float(np.linalg.norm(z-points[0])) for z in points]
            checks.append(dict(cord=cord,guide_index=i+1,putative_support_body=frame,max_center_change_in_body_mm=max(shifts),
                fixed_center_compatible_at_numerical_tolerance=bool(max(shifts)<1e-5),hardware_installed=False))
    p.write(HERE/'reviews/interfaces.json',dict(completed=True,source_sha256=source,contacts=contacts,guide_constraint_diagnostics=checks,nonadjacent_cord_self_checks=selftests,centerline_continuity=continuity,
        nominal_anchor_contacts_all_within_1e_5_mm=all(z['distance_mm']<1e-5 for z in contacts),
        guide_fixed_center_constraints_all_satisfied=all(z['fixed_center_compatible_at_numerical_tolerance'] for z in checks),
        guide_parts_installed=False,tensioner_installed=False,force_equilibrium_verified=False,
        meaning='Distances verify nominal cord/end geometry only,not frictional grip or physical force.Guide-center drift proves that the interpolated bends are not all mounted fixed-axis pulley mechanisms.Do not turn six collision snapshots into complete drive success.'))
    print('contacts',max(z['distance_mm'] for z in contacts),'guide drift',max(z['max_center_change_in_body_mm'] for z in checks),flush=True)


if __name__=='__main__':main()

"""Saved-CAD delta audit with exact baseline binding and isolated-IP expansion.

Prior unchanged-pair results are inherited only at identical twenty-angle poses,
or for unchanged relative configurations at baseline open. New isolated IP poses
also compare every thumb_ip part with every other rigid body. The separate motor
and finite ropes have five declared BRep poses; 421 other samples are equations
only. This is nominal sampled geometry, not complete physical freedom, a whole
hand tendon route, continuous motion, tendon retention or manufacture approval.
"""
from pathlib import Path
import argparse,copy,hashlib,importlib.util,json,traceback,itertools
import numpy as np


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
    ap.add_argument('--generated',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();repo=a.repo.resolve();root=a.generated.resolve();out=a.output.resolve()
    if out.exists():raise FileExistsError(out)
    import cadquery as cq
    from cadquery.occ_impl.shapes import setThreads
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    setThreads(1);hashes={}
    def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
    def bind(p,expected=None):
        p=Path(p).resolve();key=str(p);h=digest(p)
        if expected is not None and h!=expected:raise ValueError('Stale input '+key)
        if key in hashes and hashes[key]!=h:raise ValueError('Changed input '+key)
        hashes[key]=h;return p
    def read(p):return json.loads(bind(p).read_text(encoding='utf-8-sig'))
    def module(n,p):
        p=bind(p);s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
        if Path(m.__file__).resolve()!=p:raise ValueError('Wrong helper identity '+n)
        return m
    def rigid(s,T):
        t=gp_Trsf();t.SetValues(*[float(T[i,j]) for i in range(3) for j in range(4)])
        return cq.Shape.cast(BRepBuilderAPI_Transform(s.wrapped,t,True).Shape())
    def bounds(s):
        b=s.BoundingBox();return np.array([[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]])
    def corners(b):return np.array(list(itertools.product(*zip(b[0],b[1]))))
    def overlap(b,c):return bool(np.all(np.minimum(b[1],c[1])-np.maximum(b[0],c[0])>1e-7))
    report=dict(schema='thumb_ip_saved_delta_audit.v2',completed=False,passed=False,errors=[],source_sha256=hashes,
        criteria=dict(overlap_mm3=1e-4,overlap_mm3_scope='Fresh checks only; inherited baseline uses 0.02 mm3',
            fresh_overlap_mm3=1e-4,inherited_baseline_overlap_mm3=.02,
            relative_transform_cache_decimals=9,hand_maximum_sample_step_deg=5.,
            maximum_sample_step_deg=5.,maximum_sample_step_scope='Hand observations only, not finite-cord/motor BRep poses',
            cord_motor_BRep_joint_poses_deg=[-20.,-15.,0.,45.,85.],cord_motor_maximum_BRep_joint_gap_deg=45.,
            analytical_length_step_deg=.25,analytical_length_pose_count=421),
        cases=[],motor={},manufacturing_approved=False,physical_latency_measured=False,
        whole_hand_tendon_routing=False,hand_motor_cross_collision_checked=False,
        continuous_motion_approved=False,physical_freedom_complete=False,
        passed_meaning='No reportable collision in the stated fresh scopes and explicitly inherited baseline scopes; thresholds differ and this is not a manufacturing or whole-drive approval.')
    def save(final=False):
        # Each file is bound before use. Interim progress writes do not reread
        # every large native file; only the final result can pass the end guard.
        if final:
            changed_inputs=[]
            for p,h in hashes.items():
                try:
                    if digest(p)!=h:changed_inputs.append(p)
                except OSError:changed_inputs.append(p)
            report['input_hash_guard']=dict(passed=not changed_inputs,checked=True,
                phase='final',input_count=len(hashes),changed_or_missing_inputs=changed_inputs)
            if changed_inputs:report['passed']=False;report['completed']=False
        else:
            report['input_hash_guard']=dict(passed=False,checked=False,phase='pending_final',input_count=len(hashes))
        out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    try:
        bind(__file__);geo=module('_saved_geom',repo/'cad/thumb_ip_drive_v1/geometry.py')
        build=read(root/'build_report.json');scene=read(root/'scene.json');design=read(root/'design.json')
        if not build['completed'] or build['errors'] or not build['input_hash_guard']['passed']:raise ValueError('Incomplete build')
        for k,h in build['source_sha256'].items():bind(repo/k,h)
        for k,h in build['generated_sha256'].items():bind(root/k,h)
        # These helpers read their own data files; bind those before invoking
        # them, then bind the catalogue-declared accepted references as well.
        for rel in ('cad/thumb_wrist_v1/profile.json','cad/gesture_hand_v3/poses.py',
                'cad/gesture_hand_v3/design.json','cad/gesture_hand_v3/generated/scene.json'):bind(repo/rel)
        catalog_input=read(repo/'cad/gesture_hand_v3/poses.json')
        for rel,h in catalog_input['reference']['source_sha256'].items():bind(repo/rel,h)
        profile=module('_saved_profile',repo/'cad/thumb_wrist_v1/wrist_profile.py').load(repo)
        if Path(profile['poses'].__file__).resolve()!=(repo/'cad/gesture_hand_v3/poses.py').resolve():raise ValueError('Wrong cached poses module')
        native=module('_saved_gestures',repo/'cad/gesture_hand_v3/audit_gestures.py')
        fk=profile['poses'].forward_kinematics;zero=fk(design,np.zeros(20))['bodies']
        rows={r['name']:r for r in scene['hand']};changed=set(build['modified_names']+build['added_names'])
        if len(rows)!=len(scene['hand']) or len(rows)!=359+len(build['added_names']):raise ValueError('Part count mismatch')
        original=read(repo/'cad/manufacturing_v2/generated/scene.json');old={r['name']:r for r in original['hand']}
        original_design=read(repo/'cad/manufacturing_v2/generated/design.json')
        baseline_path=repo/'cad/manufacturing_v2/reviews/assembly_audit.json';baseline=read(baseline_path)
        bg=baseline.get('input_hash_guard',{});bs=baseline.get('source_sha256',{})
        if (baseline.get('schema')!='manufacturing_assembly_audit.v2' or baseline.get('completed') is not True or baseline.get('errors')
                or bg.get('passed') is not True or bg.get('start')!=bg.get('end') or bg.get('start')!=bs):raise ValueError('Baseline report incomplete or unstable')
        bt=baseline['thresholds'];bsummary=baseline['summary']
        if bt['pose_overlap_mm3']!=.02 or bt['boolean_tolerance_mm']!=1e-6 or bt['maximum_joint_step_deg']!=5.:raise ValueError('Unexpected inherited criteria')
        if any(bsummary[k]!=v for k,v in dict(physical_parts=359,static_pose_count=10,observation_count=58,sampled_hand_path_count=48,pose_collision_count=0,operation_failure_count=0).items()):raise ValueError('Unexpected baseline scope or failures')
        if design!=original_design:raise ValueError('Candidate changed the baseline design')
        if native.kinematic_definition(design)!=native.kinematic_definition(profile['design']):raise ValueError('Profile and saved geometry frames differ')
        # Bind the actual design/FK/pose/scene files, not only a previous success flag.
        inherited_identity_paths=[
            'cad/manufacturing_v2/generated/scene.json','cad/manufacturing_v2/generated/design.json',
            'cad/manufacturing_v2/generated/build_report.json','cad/manufacturing_v2/audit_assembly.py',
            'cad/gesture_hand_v3/poses.py','cad/gesture_hand_v3/poses.json','cad/gesture_hand_v3/design.json',
            'cad/gesture_hand_v3/generated/scene.json','cad/gesture_hand_v3/audit_gestures.py',
            'cad/thumb_wrist_v1/wrist_profile.py','cad/thumb_wrist_v1/profile.json']
        for rel in inherited_identity_paths:bind(repo/rel,bs[rel])
        for rel,h in profile['catalog']['reference']['source_sha256'].items():bind(repo/rel,bs[rel]);bind(repo/rel,h)
        bp={r['name']:r for r in baseline['parts']}
        if len(old)!=359 or set(bp)!=set(old):raise ValueError('Baseline part inventory mismatch')
        modified,added,unchanged=map(set,(build['modified_names'],build['added_names'],build['unchanged_names']))
        if (modified&unchanged) or (modified|unchanged)!=set(old) or (added&set(old)) or set(rows)!=(set(old)|added):raise ValueError('Candidate partition mismatch')
        for n,r in old.items():
            previous=bp[n]
            if previous['body']!=r['body'] or not previous['valid'] or previous['solid_count']!=1 or previous['volume_mm3']<=0 or not previous['mapping_passed']:raise ValueError('Invalid inherited native part '+n)
            for field,hfield in [('step','sha256_step'),('mesh','sha256_stl')]:
                p=(repo/'cad/manufacturing_v2'/r[field]).resolve();rel=p.relative_to(repo).as_posix()
                if bs.get(rel)!=r[hfield]:raise ValueError('Baseline report/source identity mismatch '+n)
                bind(p,r[hfield])
        for n in build['unchanged_names']:
            if any(rows[n][k]!=old[n][k] for k in ('body','sha256_step','sha256_stl','volume_mm3')):raise ValueError('Changed frozen part '+n)
        solids={};boxes={};verts={}
        def load_shape(row):
            base=root if row['location']=='generated' else repo
            bind(base/row['mesh'],row['sha256_stl'])
            return cq.importers.importStep(str(bind(base/row['step'],row['sha256_step']))).val()
        for n,row in rows.items():
            s=load_shape(row)
            geo.require_solid(s,n)
            s=rigid(s,np.linalg.inv(zero[row['body']]));solids[n]=s;boxes[n]=bounds(s);verts[n]=corners(boxes[n])
        gestures=copy.deepcopy(profile['catalog']['gestures'])
        for i,g in enumerate(gestures):
            if g['id']=='thumb_up':gestures[i]=dict(copy.deepcopy(profile['document']['gestures'][2]),id='thumb_up')
        gestures.append(copy.deepcopy(profile['document']['gestures'][1]))
        path,transitions=native.expand_poses(profile['document'],design,profile['poses'].NAMES,'full',5.)
        observations=[dict(id='static_'+g['id'],kind='endpoint',angles_deg=g['angles_deg']) for g in gestures]
        observations.extend(x for x in path if x['kind']=='transition')
        previous_observations=baseline['observations'];previous_cases=baseline['cases']
        if len(previous_observations)!=58 or len(observations)!=58 or transitions!=baseline['full_path_definition']:raise ValueError('Current path differs from inherited 10+48 scope')
        def twenty(values):
            q=np.asarray(values,float)
            if q.shape!=(20,) or not np.isfinite(q).all():raise ValueError('Invalid inherited pose')
            return tuple(q.tolist())
        for obs,prev in zip(observations,previous_observations):
            case=previous_cases[prev['case_id']]
            if (obs['id']!=prev['id'] or obs['kind']!=prev['kind'] or twenty(obs['angles_deg'])!=twenty(prev['angles_deg'])
                    or twenty(case['angles_deg'])!=twenty(prev['angles_deg']) or case['collision_count']!=0 or case['operation_failures']
                    or case['no_reportable_material_collision_and_no_operation_failure'] is not True):raise ValueError('Baseline pose/case identity or completion mismatch')
            obs['baseline_inheritance']=dict(case_id=prev['case_id'],method='exact_twenty_angles',pairs='unchanged versus unchanged only',overlap_threshold_mm3=.02)
        opened=next(x for x in previous_observations if x['id']=='static_open')
        if twenty(opened['angles_deg'])!=(0.,)*20:raise ValueError('Baseline open is not neutral')
        for q in range(-20,86,5):
            angles=np.zeros(20);angles[19]=q
            observations.append(dict(id='isolated_IP_'+str(q),kind='isolated_IP_sample',angles_deg=angles.tolist(),
                baseline_inheritance=dict(case_id=opened['case_id'],method='unchanged_relative_frames_at_baseline_open',
                    pairs='unchanged same-body pairs and unchanged pairs with neither body thumb_ip; all thumb_ip-versus-other-body pairs are fresh',overlap_threshold_mm3=.02)))
        report.update(observations=observations,transitions=transitions,changed_names=sorted(changed),hand_part_count=len(rows),
            unchanged_part_count=len(build['unchanged_names']),baseline_inheritance=dict(report=str(baseline_path),report_sha256=digest(baseline_path),
                completed_and_guarded=True,exact_kinematic_design_match=True,part_source_body_identity_verified=359,
                all_current_native_hand_parts_reimported_valid=True,reimported_hand_part_count=len(rows),
                exact_baseline_observations=58,new_isolated_IP_observations=22,baseline_criteria=bt,
                limitations='Inherited nominal pair results do not revalidate endplay, eccentricity, tilted washers, force, manufacture or new tendon routes.'))
        cache={};checked_exact=0;screened=0
        for obs in observations:
            frames=fk(design,obs['angles_deg'])['bodies'];inverses={b:np.linalg.inv(T) for b,T in frames.items()};pairs=set();hits=[]
            if obs['kind']=='isolated_IP_sample' and any(v!=0 for v in obs['angles_deg'][:19]):raise ValueError('Isolated IP scope moved another joint')
            initiators=changed|({n for n,r in rows.items() if r['body']=='thumb_ip'} if obs['kind']=='isolated_IP_sample' else set())
            for n in sorted(initiators):
                for m in sorted(rows):
                    if n not in changed and m not in changed and rows[n]['body']==rows[m]['body']:continue
                    pair=tuple(sorted((n,m)))
                    if n==m or pair in pairs:continue
                    pairs.add(pair)
                    T=inverses[rows[n]['body']]@frames[rows[m]['body']]
                    key=(n,m,tuple(np.round(T[:3].flatten(),9)))
                    if key not in cache:
                        pts=verts[m]@T[:3,:3].T+T[:3,3];other=np.array([pts.min(0),pts.max(0)])
                        if not overlap(boxes[n],other):cache[key]=0.;screened+=1
                        else:
                            s=rigid(solids[m],T);hit=geo.boolean(cq,'common',solids[n],s)
                            cache[key]=geo.describe(hit)['volume_mm3'];checked_exact+=1
                    if cache[key]>1e-4:hits.append(dict(parts=[n,m],overlap_mm3=cache[key]))
            report['cases'].append(dict(id=obs['id'],collisions=hits,baseline_inheritance=obs['baseline_inheritance'],
                fresh_scope='changed/new versus all, plus every thumb_ip part versus every other body' if obs['kind']=='isolated_IP_sample' else 'changed/new versus all'))
            print(json.dumps(dict(case=obs['id'],collisions=len(hits),exact=checked_exact)),flush=True);save()
        report['hand_summary']=dict(observation_count=len(observations),exact_unique_pair_checks=checked_exact,box_only_unique_checks=screened,
            collision_observations=sum(bool(c['collisions']) for c in report['cases']),collision_count=sum(len(c['collisions']) for c in report['cases']),
            exact_baseline_pose_inheritance_count=58,isolated_IP_expanded_observation_count=22,
            unchanged_pair_inheritance_threshold_mm3=.02,fresh_pair_threshold_mm3=1e-4,
            scope='All changed/new hand parts versus all hand parts. At 22 isolated IP poses, all thumb_ip parts versus all other bodies are fresh. Remaining unchanged relative configurations are inherited only from identity-bound baseline cases; this does not recheck real joint freeplay.')
        print(json.dumps(dict(stage='hand_complete',observations=len(observations),collisions=report['hand_summary']['collision_count'])),flush=True)
        # Separate neutral motor module: only exact declared thread zones may overlap.
        mr={r['name']:r for r in scene['motor_bench']}
        if len(mr)!=len(scene['motor_bench']):raise ValueError('Duplicate motor part')
        ms={n:load_shape(r) for n,r in mr.items()};mb={n:bounds(s) for n,s in ms.items()};motor_validity={}
        for n,s in ms.items():
            facts=geo.describe(s);expected=12 if n=='drive_m077_stator' else 1
            if (not facts['valid'] or not facts['bounded'] or not facts['positive_solids'] or facts['volume_mm3']<=0
                    or facts['solid_count']!=expected or facts['solid_count']!=mr[n]['solid_count']):raise ValueError('Invalid reimported motor part '+n)
            motor_validity[n]=facts
        output_names=sorted(n for n,r in mr.items() if r['body']=='bench_motor_output')
        fixed_names=sorted(n for n,r in mr.items() if r['body']=='bench_motor_frame')
        if not output_names or not fixed_names or set(output_names+fixed_names)!=set(mr):raise ValueError('Unknown or incomplete motor body partition')
        exceptions={tuple(sorted(x['pair'])):x for x in build['motor_interfaces']['declared_thread_overlap_regions']}
        for pair in exceptions:
            if not all(n in mr for n in pair) or mr[pair[0]]['body']!=mr[pair[1]]['body']:raise ValueError('Moving/fixed thread-zone handling needs an explicit frame')
        unexpected=[];thread_hits=[];exact=0;screen=0
        for n,m in itertools.combinations(sorted(ms),2):
            if not overlap(mb[n],mb[m]):screen+=1;continue
            exact+=1;hit=geo.boolean(cq,'common',ms[n],ms[m]);v=geo.describe(hit)['volume_mm3']
            if v<=1e-4:continue
            spec=exceptions.get((n,m))
            if spec:
                zone=cq.Solid.makeCylinder(spec['radius_mm'],spec['length_mm'],cq.Vector(*spec['start_mm']),cq.Vector(*spec['axis']))
                outside=geo.describe(geo.boolean(cq,'cut',hit,zone))['volume_mm3']
                thread_hits.append(dict(parts=[n,m],volume_mm3=v,outside_declared_region_mm3=outside))
                if outside<=1e-4:continue
            unexpected.append(dict(parts=[n,m],volume_mm3=v))
        report['motor']=dict(part_rows=len(mr),native_solid_count=sum(len(s.Solids()) for s in ms.values()),exact_pairs=exact,box_screened=screen,
            expected_thread_contacts=thread_hits,unexpected_collisions=unexpected,
            native_reimport_validity=motor_validity,all_saved_motor_parts_reimported_valid=True,
            rotating_body='bench_motor_output',rotating_parts=output_names,fixed_body='bench_motor_frame',fixed_parts=fixed_names,
            central_retainer_missing=build['motor_interfaces']['spool_to_horn']['central_retainer_missing'],
            same_body_pairs_inherited_from='This run: all-pair neutral motor audit; same rigid-body relative transforms do not change',
            limitations='Five nominal output rotations only. The rear idler is assigned to the fixed display body; its independent freedom, real spline contact and complete assembly are not validated.')
        print(json.dumps(dict(stage='motor_neutral_complete',parts=len(mr),exact_pairs=exact,unexpected_collisions=len(unexpected))),flush=True)
        # The rope is a finite solid, not only a centreline drawing.
        import math
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
        from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
        routing=module('_saved_routing',repo/'cad/thumb_ip_drive_v1/routing.py')
        drive=module('_saved_joint',repo/'cad/thumb_ip_drive_v1/joint_drive.py')
        def rope_arc(x,r,lo,hi):
            if not 0<hi-lo<2*math.pi:raise ValueError('Invalid finite wrap branch')
            ax=gp_Ax2(gp_Pnt(x,0,0),gp_Dir(1,0,0),gp_Dir(0,math.cos(lo),math.sin(lo)))
            sh=cq.Shape.cast(BRepPrimAPI_MakeTorus(ax,r,.3,hi-lo).Shape())
            geo.require_solid(sh,'finite rope arc');return sh
        def cord(a,b):
            av=cq.Vector(*a);delta=cq.Vector(*b)-av
            return cq.Solid.makeCylinder(.3,delta.Length,av,delta.normalized())
        def contact_volume(a,b):return geo.describe(geo.boolean(cq,'common',a,b))['volume_mm3']
        rj=float(routing.JOINT_RADIUS_MM);rm=float(routing.MOTOR_RADIUS_MM);ratio=rj/rm
        translation=np.asarray(scene['motor_to_ip_local_translation_mm'],float)
        tracks=np.asarray(routing.TRACKS_X_MM,float);motor_tracks=np.asarray(build['motor_interfaces']['groove_planes_x_mm'],float)
        diameter=float(build['motor_interfaces']['cord_diameter_mm'])
        if (translation.shape!=(3,) or not np.isfinite(translation).all() or scene['motor_rotation_axis']!='+X'
                or not np.allclose(translation[1:],[ -float(routing.MOTOR_DISTANCE_MM),0.],rtol=0,atol=1e-9)
                or tracks.shape!=(2,) or motor_tracks.shape!=(2,) or not np.allclose(motor_tracks+translation[0],tracks,rtol=0,atol=1e-9)
                or abs(float(scene['motor_rotation_per_ip_rotation'])-ratio)>1e-12
                or abs(float(build['motor_interfaces']['cord_center_radius_mm'])-rm)>1e-12
                or abs(float(drive.PARAMETERS['tendon_centre_radius_mm'])-rj)>1e-12
                or abs(float(drive.PARAMETERS['tendon_nominal_diameter_mm'])-diameter)>1e-12
                or abs(diameter-.6)>1e-12 or list(drive.PARAMETERS['tendon_tracks_x_mm'])!=list(tracks)
                or float(drive.PARAMETERS['anchor_angle_deg'])!=float(routing.JOINT_ANCHOR_DEG)):
            raise ValueError('Motor/hand/rope coordinate or radius contract differs')
        anchors=build['motor_interfaces']['anchors'];nominal=routing.route(0)
        if len(anchors)!=2:raise ValueError('Expected two physical motor anchors')
        for i,anchor in enumerate(anchors):
            if not np.allclose(np.asarray(anchor['tail_start_mm'],float)+translation,nominal[i]['motor_anchor_mm'],rtol=0,atol=1e-8):raise ValueError('Motor anchor-to-IP frame mismatch')
        def ip_to_motor(point):return (np.asarray(point,float)-translation).tolist()
        cord_checks=[];rotation_checks=[]
        def compare_rope(rope,parts,part_boxes,scope,q,track,coordinate_frame):
            rb=bounds(rope)
            for n,s in parts.items():
                tested=overlap(rb,part_boxes[n]);value=contact_volume(rope,s) if tested else 0.
                cord_checks.append(dict(scope=scope,q_joint_deg=q,track=track,part=n,overlap_mm3=value,
                    intersection_tested=tested,bbox_screened=not tested,coordinate_frame=coordinate_frame))
        ip_parts={n:rigid(s,np.linalg.inv(zero['thumb_ip'])@zero[rows[n]['body']]) for n,s in solids.items() if rows[n]['body'] in ('thumb_mcp','thumb_ip')}
        q_samples=report['criteria']['cord_motor_BRep_joint_poses_deg']
        for q in q_samples:
            qm=ratio*q
            placed_motor={n:s.rotate((0,0,0),(1,0,0),qm) if n in output_names else s for n,s in ms.items()}
            placed_motor_boxes={n:bounds(s) for n,s in placed_motor.items()}
            placed_ip={n:s.rotate((0,0,0),(1,0,0),q) if rows[n]['body']=='thumb_ip' else s for n,s in ip_parts.items()}
            placed_ip_boxes={n:bounds(s) for n,s in placed_ip.items()}
            routes=routing.route(q);jm=drive.tendon_geometry(q)
            for i,row in enumerate(routes):
                # Bench-fixed frame: moving hardware and anchor tails rotate;
                # exterior tangent contacts stay fixed. No hand-world transform
                # or undeclared whole-hand motor placement is inferred.
                a,b=row['contact_angle_rad'],row['motor_anchor_angle_rad']
                arc=rope_arc(float(motor_tracks[i]),rm,min(a,b),max(a,b))
                tail=cord(anchors[i]['tail_start_mm'],anchors[i]['tail_exit_mm']).rotate((0,0,0),(1,0,0),qm)
                span=cord(ip_to_motor(row['motor_contact_mm']),ip_to_motor(row['joint_contact_mm']))
                for scope,rope in [('motor_wrap',arc),('motor_tail',tail),('free_span_motor_hardware',span)]:
                    compare_rope(rope,placed_motor,placed_motor_boxes,scope,q,i,'motor_bench_fixed')
                a,b=row['contact_angle_rad'],row['joint_anchor_angle_rad']
                pieces={'joint_wrap':rope_arc(row['track_x_mm'],rj,min(a,b),max(a,b)),
                        'joint_tail':cord(jm[i]['anchor_mm'],jm[i]['tail_end_mm']),
                        'free_span':cord(row['motor_contact_mm'],row['joint_contact_mm'])}
                for scope,rope in pieces.items():
                    compare_rope(rope,placed_ip,placed_ip_boxes,scope,q,i,'neutral_IP_fixed_parent')
            for n in output_names:
                for other in fixed_names:
                    tested=overlap(placed_motor_boxes[n],placed_motor_boxes[other])
                    value=contact_volume(placed_motor[n],placed_motor[other]) if tested else 0.
                    rotation_checks.append(dict(q_joint_deg=q,q_motor_deg=qm,moving=n,fixed=other,overlap_mm3=value,
                        intersection_tested=tested,bbox_screened=not tested))
            print(json.dumps(dict(stage='motor_and_cord_pose_complete',q_joint_deg=q,q_motor_deg=qm,
                cumulative_cord_part_checks=len(cord_checks),cumulative_output_fixed_checks=len(rotation_checks),
                cumulative_collisions=sum(r['overlap_mm3']>1e-4 for r in cord_checks+rotation_checks))),flush=True)
        kinematics=[]
        for q in [-20+i*.25 for i in range(421)]:
            for i,row in enumerate(routing.route(q)):
                kinematics.append(dict(q=q,track=i,length_residual_mm=row['total_working_length_mm']-nominal[i]['total_working_length_mm'],
                    joint_wrap_deg=math.degrees(row['joint_wrap_rad']),motor_wrap_deg=math.degrees(row['motor_wrap_rad'])))
        report['tendon']=dict(checks=cord_checks,selected_motor_rotation_checks=rotation_checks,
            collision_count=sum(r['overlap_mm3']>1e-4 for r in cord_checks+rotation_checks),
            kinematic_sample_count=421,max_length_residual_mm=max(abs(r['length_residual_mm']) for r in kinematics),
            min_joint_wrap_deg=min(r['joint_wrap_deg'] for r in kinematics),min_motor_wrap_deg=min(r['motor_wrap_deg'] for r in kinematics),
            BRep_joint_poses_deg=q_samples,BRep_motor_poses_deg=[ratio*q for q in q_samples],maximum_BRep_joint_gap_deg=max(b-a for a,b in zip(q_samples,q_samples[1:])),
            analytical_only_joint_range_deg=[-20.,85.],analytical_only_step_deg=.25,
            motor_rotation_part_selection='All saved bench_motor_output parts versus all saved bench_motor_frame parts; selected refers only to five sampled angles',
            motor_rotating_parts=output_names,motor_fixed_parts=fixed_names,motor_hardware_excluded_from_five_pose_checks=[],
            joint_cord_checked_parts=sorted(ip_parts),joint_cord_excluded_hand_parts=sorted(set(rows)-set(ip_parts)),
            motor_to_IP_translation_mm=translation.tolist(),motor_rotation_per_IP_rotation=ratio,coordinate_contract_checked=True,
            whole_hand_tendon_routing=False,hand_motor_cross_collision_checked=False,rope_rope_contact_checked=False,
            nominal_groove_contact_and_clamping_force_verified=False,
            scope='Five nominal finite-cord and all-saved-motor-body poses; 421 samples evaluate only length/wrap equations. Joint wrap/tail/free span are checked only against thumb_mcp/thumb_ip parts. Motor wrap/tail/free span are checked against all saved motor hardware in the bench frame. No hand-to-motor collision, other-hand cord routing, tension, groove seating, friction, compression, retention, tolerance or continuous sweep approval.')
        report['completed']=True;report['passed']=not report['hand_summary']['collision_count'] and not unexpected and not report['tendon']['collision_count']
    except Exception as e:report['errors'].append(dict(error=repr(e),traceback=traceback.format_exc()))
    finally:save(final=True)
    print(json.dumps(dict(completed=report['completed'],passed=report['passed'],errors=report['errors'])),flush=True)
    return 0 if report['completed'] and report['passed'] and not report['errors'] else 1


if __name__=='__main__':raise SystemExit(main())

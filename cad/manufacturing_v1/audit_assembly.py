"""Read saved fabrication candidate; test pin insertion and rotating clip envelopes.

Insertion sweeps are exact unions of prismatic pin sections, not pose samples.
Whole-hand pose tests are static samples; they do not prove continuous hand motion.
"""
from pathlib import Path
import argparse,copy,hashlib,importlib.util,json,math,sys,time,traceback
import numpy as np
import cadquery as cq

HERE=Path(__file__).resolve().parent
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    if Path(m.__file__).resolve()!=Path(path).resolve():raise ValueError('Wrong module path')
    return m
def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--repo',type=Path,default=HERE.parents[1])
    ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();repo=a.repo.resolve();out=a.output.resolve()
    if out.exists():raise FileExistsError('Use a new report path')
    base=repo/'cad/gesture_hand_v3';candidate=repo/'cad/manufacturing_v1';generated=candidate/'generated';hashes={}
    def bind(p,expected=None):
        p=Path(p).resolve()
        if not p.is_relative_to(repo) or not p.is_file():raise ValueError('Missing/escaping input '+str(p))
        value=digest(p);key=p.relative_to(repo).as_posix()
        if expected is not None and value!=expected:raise ValueError('Input mismatch '+key)
        if key in hashes and hashes[key]!=value:raise ValueError('Input changed '+key)
        hashes[key]=value;return p
    for p in [Path(__file__),candidate/'build.py',candidate/'design_patch.json',generated/'design.json',generated/'scene.json',
              generated/'build_report.json',generated/'mass_inertia.json',generated/'hand.step',base/'audit_gestures.py',
              base/'fabrication_repairs.py',base/'poses.py',base/'poses.json',base/'design.json',base/'generated/build_report.json',
              base/'generated/scene.json',repo/'cad/articulation_drive_v2/verify.py',
              repo/'cad/thumb_wrist_v1/wrist_profile.py',repo/'cad/thumb_wrist_v1/profile.json']:
        bind(p)
    scene=read(generated/'scene.json');design=read(generated/'design.json');build=read(generated/'build_report.json')
    guard=build.get('input_hash_guard',{})
    if (scene['source_sha256']!=build['source_sha256'] or not build['inputs_unchanged'] or not build.get('completed')
        or build.get('errors') or guard.get('passed') is not True or guard.get('start')!=guard.get('end')
        or guard.get('start')!=build['source_sha256']):raise ValueError('Unstable/incomplete build')
    bind(generated/'hand.step',build['assembly_step_sha256'])
    bind(generated/'design.json',build['generated_design_sha256'])
    for relative,expected in build['source_sha256'].items():bind(repo/relative,expected)
    normalized=copy.deepcopy(scene)
    for row in normalized['hand']:
        for key,hashkey in [('step','sha256_step'),('mesh','sha256_stl')]:
            p=bind(candidate/row[key],row[hashkey]);row[key]=p.relative_to(repo).as_posix()
    if len(normalized['hand'])!=349:raise ValueError('Expected349 parts')
    sys.path.insert(0,str(base))
    native=module('_mfg_native_screen',base/'audit_gestures.py');v=native.load_v2(repo)
    geometry=module('_mfg_geometry_helpers',base/'fabrication_repairs.py')
    builder=module('_mfg_builder_for_sweeps',candidate/'build.py')
    profile=module('_mfg_wrist_profile',repo/'cad/thumb_wrist_v1/wrist_profile.py').load(repo)
    original_scene=read(base/'generated/scene.json');original_rows={r['name']:r for r in original_scene['hand']}
    if [r['name'] for r in scene['hand']]!=[r['name'] for r in original_scene['hand']]:raise ValueError('Changed part order/set')
    allowed={name for digit in ['index','middle','ring','little','thumb'] for j in (1,2)
             for name in (f'{digit}_joint_pin_{j}',f'{digit}_segment{j}_distal_left_bushing')}
    if build.get('modified_part_count')!=20 or {r['name'] for r in build['changed_parts']}!=allowed:raise ValueError('Unexpected changed parts')
    for row in scene['hand']:
        old=original_rows[row['name']]
        if row['body']!=old['body'] or row['step_index']!=old['step_index']:raise ValueError('Changed body/assembly order')
        if row['name'] not in allowed and any(row[k]!=old[k] for k in ('sha256_step','sha256_stl','volume_mm3','bbox_mm')):
            raise ValueError('Unchanged part differs '+row['name'])
    expected_design=copy.deepcopy(design);expected_design.pop('manufacturing_candidate')
    expected_design['joint_ranges_deg']['thumb_ip']=[0,85]
    if expected_design!=read(base/'design.json'):raise ValueError('Undeclared joint/frame change')
    retention=read(base/'generated/build_report.json')['fabrication_repairs']['pin_retention']
    report=dict(schema='manufacturing_assembly_audit.v1',completed=False,errors=[],parts=[],mesh_topology=[],
        pin_insertion=[],clip_envelopes=[],pose_cases={},source_sha256=dict(hashes),
        thresholds=dict(boolean_tolerance_mm=1e-6,insertion_overlap_mm3=1e-4,pose_overlap_mm3=.02),
        scope='10 pin axial insertions from local X=-30 to0; native349 saved CAD with10 clips conservatively expanded to full360-degree annular envelopes at10 static endpoint poses. No continuous hand-motion, clip installation elasticity, tolerances, strength or drive/speed approval.')
    def save():
        after={p:digest(repo/p) for p in hashes};report['input_hash_guard']=dict(start=hashes,end=after,passed=hashes==after)
        if hashes!=after:report['completed']=False
        native.write_json(out,report)
    def check_solid(s):
        if not s.isValid() or len(s.Solids())!=1:raise ValueError('Invalid swept/gauge solid')
    def intersections(moving,obstacles):
        bb=v.bounds(moving);rows=[]
        for name,solid in obstacles.items():
            other=v.bounds(solid)
            if np.any(np.minimum(bb[1],other[1])-np.maximum(bb[0],other[0])<=0):continue
            volume=float(moving.intersect(solid,tol=1e-6).Volume())
            if not math.isfinite(volume) or volume<0:raise ValueError('Invalid intersection volume')
            rows.append(dict(part=name,overlap_mm3=volume,blocked=volume>1e-4))
        return dict(broadphase_candidates=len(rows),checked=rows,blocked_parts=[r for r in rows if r['blocked']],
                    passed=not any(r['blocked'] for r in rows))
    try:
        parts=v.load_parts(normalized,repo,report)
        if not all(r['valid'] and r['solid_count']==1 for r in report['parts']):raise ValueError('Invalid saved part')
        report['assembly_roundtrip']=v.assembly_roundtrip(generated/'hand.step',normalized['hand'],parts)
        if not report['assembly_roundtrip']['passed']:raise ValueError('Assembly STEP mismatch')
        bodies={r['name']:r['body'] for r in scene['hand']};proxy=dict(parts)
        configs={x['name']:x for x in design['fingers']};configs['thumb']=design['thumb']
        for definition in retention:
            ident=definition['id'];digit,j=ident.rsplit('_distal',1);frame=geometry._frame(configs[digit],int(j))
            pin_name=definition['pin'];clip_name=definition['clip'];bush_name=f'{digit}_segment{j}_distal_left_bushing'
            saved_local=geometry._local(parts[pin_name],frame);constructed=builder.make_pin(cq,'round')
            reconstruction=[float(saved_local.cut(constructed).Volume()),float(constructed.cut(saved_local).Volume())]
            if max(reconstruction)>1e-5:raise ValueError('Saved pin differs from swept sections '+ident)
            swept=geometry._world(builder.swept_pin(cq,'round',30.),frame);check_solid(swept)
            old_row=original_rows[bush_name];old_path=bind(base/old_row['step'],old_row['sha256_step'])
            old_bush=cq.importers.importStep(str(old_path)).val()
            old_swept=geometry._world(builder.swept_pin(cq,'old',30.),frame)
            control=float(old_swept.intersect(old_bush,tol=1e-6).Volume())
            if control<=1e-4:raise ValueError('Original insertion obstruction was not reproduced '+ident)
            # The clip is explicitly fitted last. All other components remain present.
            present={n:s for n,s in parts.items() if n not in (pin_name,clip_name)}
            local={n:s for n,s in present.items() if bodies[n] in (definition['parent_body'],definition['child_body'])}
            entry=dict(id=ident,travel_mm=30.,direction='local +X from left; tip first',
                moving_pin=pin_name,installed_last=clip_name,pin_reconstruction_difference_mm3=reconstruction,
                original_pin_through_original_left_bush_overlap_mm3=control,
                new_pin_through_new_left_bush=intersections(swept,{bush_name:parts[bush_name]}),
                joint_subassembly=intersections(swept,local),whole_hand_neutral=intersections(swept,present),
                exclusions_reason='Only the moving pin and its clip, which is installed after pin insertion. No other collision exclusion.',
                continuous_scope='Exact union of constant cross-section axial pin pieces extruded over the full30mm translation; holds for nominal rigid geometry only.')
            report['pin_insertion'].append(entry)
            envelope=geometry._world(geometry._ring(cq,3.5,1.65,13.15,.7),frame);check_solid(envelope)
            difference=float(parts[clip_name].cut(envelope).Volume())
            if difference>1e-5:raise ValueError('Clip not contained by clocking envelope '+ident)
            proxy[clip_name]=envelope
            report['clip_envelopes'].append(dict(id=ident,part=clip_name,original_outside_envelope_mm3=difference,
                outer_radius_mm=3.5,inner_radius_mm=1.65,axial_span_mm=[13.15,13.85],
                meaning='All angular clocking of the saved undeformed clip; elastic installation/load capacity not modeled.'))
            print(json.dumps(dict(joint=ident,old_obstruction_mm3=control,
                local_clear=entry['joint_subassembly']['passed'],in_hand_clear=entry['whole_hand_neutral']['passed'])),flush=True)
            save()
        gestures=copy.deepcopy(profile['catalog']['gestures'])
        for gesture in gestures:
            if gesture['id']=='thumb_up':gesture['angles_deg']=profile['document']['gestures'][2]['angles_deg']
        gestures.append(copy.deepcopy(profile['document']['gestures'][1]))
        report['gestures']=gestures
        for gesture in gestures:profile['poses'].validate_angles(gesture['angles_deg'],design)
        engine=native.PairScreen(v,proxy,scene['hand'],design,1.,True)
        for gesture in gestures:
            result=engine.evaluate(gesture['angles_deg']);report['pose_cases'][gesture['id']]=result
            print(json.dumps(dict(pose=gesture['id'],collisions=result['collision_count'],errors=len(result['operation_failures']))),flush=True)
            save()
        report['summary']=dict(parts=len(parts),modified_parts=20,pin_insertions=len(report['pin_insertion']),
            left_bushing_paths_blocked=sum(not x['new_pin_through_new_left_bush']['passed'] for x in report['pin_insertion']),
            joint_subassembly_paths_blocked=sum(not x['joint_subassembly']['passed'] for x in report['pin_insertion']),
            whole_hand_neutral_paths_blocked=sum(not x['whole_hand_neutral']['passed'] for x in report['pin_insertion']),
            original_obstructions_reproduced=len(report['pin_insertion']),clip_rotation_envelopes=len(report['clip_envelopes']),
            static_pose_count=len(gestures),pose_collision_count=sum(x['collision_count'] for x in report['pose_cases'].values()),
            operation_failure_count=sum(len(x['operation_failures']) for x in report['pose_cases'].values()))
        report['completed']=True
    except Exception as exc:report['errors'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    save();print(json.dumps(dict(completed=report['completed'],summary=report.get('summary'),errors=report['errors'])),flush=True)
    return 0 if report['completed'] and report['input_hash_guard']['passed'] else 1
if __name__=='__main__':raise SystemExit(main())

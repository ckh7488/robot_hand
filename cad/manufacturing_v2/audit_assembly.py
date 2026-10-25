"""Audit nominal retained-pin hardware, declared tool access, and hand motion.

Reads saved native CAD. Thread envelopes are not helical threads. A conservative
pin/keeper/screw/two-washer cluster bounds360-degree coaxial spin and nominal
axial endplay; it is not an extra manufactured part. It replaces five parts
after checking internal intersections, axial constraints and containment.
Washer radial eccentricity/tilt and manufacturing tolerances remain unmodelled.
"""
from pathlib import Path
import argparse,copy,hashlib,importlib.util,json,math,sys,time,traceback
import numpy as np
import cadquery as cq

HERE=Path(__file__).resolve().parent
DIGITS=('index','middle','ring','little','thumb')
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def module(name,p):
    s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
    if Path(m.__file__).resolve()!=Path(p).resolve():raise ValueError('Wrong module path')
    return m
def cylinder(radius,lo,hi):return cq.Solid.makeCylinder(radius,hi-lo,cq.Vector(lo,0,0),cq.Vector(1,0,0))
def hexagon(af,lo,hi):
    r=af/math.sqrt(3)
    wire=cq.Wire.makePolygon([cq.Vector(lo,r*math.cos(i*math.pi/3),r*math.sin(i*math.pi/3)) for i in range(6)],close=True)
    return cq.Solid.extrudeLinear(wire,[],cq.Vector(hi-lo,0,0))
def flat(width,thickness,lo,hi):return cq.Workplane('XY').box(hi-lo,width,thickness).translate(((hi+lo)/2,0,0)).val()
def tool_gauge(spec,kind,travel=0,oversize_tip=False):
    """Exact union of declared constant-profile gauge pieces during approach."""
    if kind=='hex':
        s=spec['hex_driver'];lo=s['tip_start_x_mm'];hi=s['tip_end_x_mm'];end=lo+s['blade_length_mm']
        tip=hexagon(s['tip_AF_mm']+(.1 if oversize_tip else 0),lo,hi+travel)
        shaft=cylinder(s['shaft_radius_mm'],hi,end+travel)
        grip=cylinder(s['handle_radius_mm'],end,end+s['handle_length_mm']+travel)
    else:
        s=spec['counterhold_driver'];lo=s['tip_start_x_mm'];hi=s['tip_end_x_mm'];end=hi-s['blade_length_mm']
        tip=flat(s['tip_width_y_mm'],s['tip_thickness_z_mm']+(.4 if oversize_tip else 0),lo-travel,hi)
        shaft=cylinder(s['shaft_radius_mm'],end-travel,lo)
        grip=cylinder(s['handle_radius_mm'],end-s['handle_length_mm']-travel,end)
    return tip.fuse(shaft).fuse(grip).clean()
def cluster_envelope(endplay):
    # Each circular exterior section is swept through common translation[-endplay,0].
    # Filling internal bores/sockets is conservative for EXTERNAL material only.
    sections=((3.5,-15,-13),(2.,-13,13.15),(3.5,13.15,13.95),(1.9,13.95,15.95))
    answer=None
    for radius,lo,hi in sections:
        part=cylinder(radius,lo-endplay,hi)
        answer=part if answer is None else answer.fuse(part)
    for lo,hi in [(-13-endplay,-12.4),(12.4,13+endplay)]:
        annulus=cylinder(3.7,lo,hi).cut(cylinder(2.05,lo-.01,hi+.01))
        answer=answer.fuse(annulus)
    return answer.clean()

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--repo',type=Path,default=HERE.parents[1]);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();repo=a.repo.resolve();out=a.output.resolve()
    if out.exists():raise FileExistsError('Use a new audit report path')
    candidate=repo/'cad/manufacturing_v2';base=repo/'cad/manufacturing_v1';g=candidate/'generated';legacy=repo/'cad/gesture_hand_v3';hashes={}
    def bind(p,expected=None):
        p=Path(p).resolve()
        if not p.is_relative_to(repo) or not p.is_file():raise ValueError('Missing/escaping input '+str(p))
        sha=digest(p);key=p.relative_to(repo).as_posix()
        if expected is not None and sha!=expected:raise ValueError('Input hash mismatch '+key)
        if key in hashes and hashes[key]!=sha:raise ValueError('Changed input '+key)
        hashes[key]=sha;return p
    for p in [Path(__file__),candidate/'build.py',candidate/'design_patch.json',candidate/'tool_gauges.json',
              g/'scene.json',g/'design.json',g/'mass_inertia.json',g/'build_report.json',g/'hand.step',
              base/'generated/scene.json',base/'generated/design.json',legacy/'audit_gestures.py',
              legacy/'fabrication_repairs.py',legacy/'poses.py',legacy/'poses.json',legacy/'design.json',legacy/'generated/scene.json',legacy/'generated/build_report.json',
              repo/'cad/articulation_drive_v2/verify.py',repo/'cad/thumb_wrist_v1/wrist_profile.py',repo/'cad/thumb_wrist_v1/profile.json']:
        bind(p)
    build=read(g/'build_report.json');scene=read(g/'scene.json');design=read(g/'design.json');spec=read(candidate/'tool_gauges.json')
    guard=build.get('input_hash_guard',{})
    if (not build.get('completed') or build.get('errors') or not build.get('inputs_unchanged') or not guard.get('passed')
        or guard.get('start')!=guard.get('end') or guard.get('start')!=build.get('source_sha256')
        or scene.get('source_sha256')!=build.get('source_sha256')):raise ValueError('Incomplete/unstable build')
    for name,sha in build['source_sha256'].items():bind(repo/name,sha)
    bind(g/'hand.step',build['assembly_step_sha256']);bind(g/'design.json',build['generated_design_sha256'])
    normalized=copy.deepcopy(scene)
    for row in normalized['hand']:
        for key,sha in (('step','sha256_step'),('mesh','sha256_stl')):
            row[key]=bind(candidate/row[key],row[sha]).relative_to(repo).as_posix()
    old=read(base/'generated/scene.json');oldrows={r['name']:r for r in old['hand']};rows={r['name']:r for r in scene['hand']}
    modified={f'{d}_joint_pin_{j}' for d in DIGITS for j in (1,2)}
    removed={f'{d}_distal{j}_retaining_clip' for d in DIGITS for j in (1,2)}
    added={f'{d}_distal{j}_retaining_{kind}' for d in DIGITS for j in (1,2) for kind in ('washer','screw')}
    unchanged=set(oldrows)-modified-removed
    loose_washers={f'{d}_distal{j}_{side}_pin_washer' for d in DIGITS for j in (1,2) for side in ('left','right')}
    if len(loose_washers)!=20 or not loose_washers<=unchanged:raise ValueError('Missing preserved loose washers')
    if len(oldrows)!=349 or len(rows)!=359 or len(scene['hand'])!=359 or set(rows)!=((set(oldrows)-removed)|added):raise ValueError('Wrong part change partition')
    if len(unchanged)!=329 or [r['step_index'] for r in scene['hand']]!=list(range(359)):raise ValueError('Wrong assembly index/unchanged count')
    for name in unchanged:
        if any(rows[name][k]!=oldrows[name][k] for k in ('body','sha256_step','sha256_stl','volume_mm3','bbox_mm')):raise ValueError('Changed preserved part '+name)
    for name in modified:
        if rows[name]['body']!=oldrows[name]['body']:raise ValueError('Changed pin placement body')
    d0=copy.deepcopy(read(base/'generated/design.json'));d1=copy.deepcopy(design)
    d0.pop('manufacturing_candidate',None);d1.pop('manufacturing_candidate',None)
    if d0!=d1:raise ValueError('Undeclared hand proportions/frame/range change')
    sys.path.insert(0,str(legacy));native=module('_mfg2_native',legacy/'audit_gestures.py');v=native.load_v2(repo)
    geometry=module('_mfg2_geometry',legacy/'fabrication_repairs.py');builder=module('_mfg2_builder',candidate/'build.py')
    catalog=read(legacy/'poses.json')
    for name,sha in catalog['reference']['source_sha256'].items():bind(repo/name,sha)
    blanks={r['finished_part']:r for r in build['machining_blanks']}
    if len(build['machining_blanks'])!=10 or set(blanks)!=modified:raise ValueError('Wrong machining blank partition')
    for pin,record in blanks.items():
        if record['assembly_part'] or record['included_in_mass'] or record['finished_thread_geometry']:raise ValueError('Pilot blank incorrectly labelled')
        bind(candidate/record['step'],record['sha256_step'])
    profile=module('_mfg2_profile',repo/'cad/thumb_wrist_v1/wrist_profile.py').load(repo)
    report=dict(schema='manufacturing_assembly_audit.v2',completed=False,errors=[],parts=[],mesh_topology=[],retention=[],
        part_contract=dict(baseline_count=349,candidate_count=359,modified=sorted(modified),removed=sorted(removed),added=sorted(added),unchanged=sorted(unchanged)),
        source_sha256=dict(hashes),tool_gauge_definition=spec,observations=[],cases={},
        thresholds=dict(boolean_tolerance_mm=1e-6,access_overlap_mm3=1e-4,pose_overlap_mm3=.02,maximum_joint_step_deg=5.),
        scope='359 saved nominal CAD parts;10 pin/keeper/thread-outer-envelope approaches;10 pairs of declared screwdriver gauges at nominal seated hardware positions.319 collision entities cover all359 parts including full coaxial retention spin and nominal correlated axial freedom of pins and both existing outer washers.10 static gestures plus wrist-directed forward hand-closing samples. No washer radial eccentricity/tilt, helical-thread installation, tool-vendor-CAD fit, tolerances, continuous hand motion, strength, preload, loosening, physical fabrication or latency approval.')
    def save():
        report['source_sha256']=dict(hashes)
        after={p:digest(repo/p) for p in hashes};report['input_hash_guard']=dict(start=hashes,end=after,passed=hashes==after)
        if hashes!=after:report['completed']=False
        native.write_json(out,report)
    def single(s):
        if not s.isValid() or len(s.Solids())!=1:raise ValueError('Invalid constructed solid')
    def difference(actual,expected):
        values=[float(actual.cut(expected,tol=1e-6).Volume()),float(expected.cut(actual,tol=1e-6).Volume())]
        if not all(math.isfinite(x) and abs(x)<=1e-5 for x in values):raise ValueError('Saved geometry differs from nominal interface '+str(values))
        return values
    def intersections(moving,obstacles):
        bb=v.bounds(moving);checked=[]
        for name,solid in obstacles.items():
            other=v.bounds(solid)
            if np.any(np.minimum(bb[1],other[1])-np.maximum(bb[0],other[0])<=0):continue
            volume=float(moving.intersect(solid,tol=1e-6).Volume())
            if not math.isfinite(volume) or volume<0:raise ValueError('Invalid intersection result')
            checked.append(dict(part=name,overlap_mm3=volume,blocked=volume>1e-4))
        blocked=[x for x in checked if x['blocked']]
        return dict(broadphase_candidates=len(checked),checked=checked,blocked_parts=blocked,passed=not blocked)
    def scoped(solid,obstacles,digit):
        return dict(finger_module=intersections(solid,{n:s for n,s in obstacles.items() if n.startswith(digit+'_')}),whole_hand_neutral=intersections(solid,obstacles))
    try:
        parts=v.load_parts(normalized,repo,report)
        if len(parts)!=359 or not all(x['valid'] and x['solid_count']==1 and x['mapping_passed'] for x in report['parts']):raise ValueError('Invalid native parts')
        report['assembly_roundtrip']=v.assembly_roundtrip(g/'hand.step',normalized['hand'],parts)
        if not report['assembly_roundtrip']['passed']:raise ValueError('Assembly STEP differs')
        configs={r['name']:r for r in design['fingers']};configs['thumb']=design['thumb']
        proxy={n:s for n,s in parts.items() if n not in modified|added|loose_washers};proxyrows=[copy.deepcopy(r) for r in scene['hand'] if r['name'] in proxy]
        for digit in DIGITS:
            for j in (1,2):
                ident=f'{digit}_distal{j}';pin=f'{digit}_joint_pin_{j}';washer=ident+'_retaining_washer';screw=ident+'_retaining_screw'
                left_name=ident+'_left_pin_washer';right_name=ident+'_right_pin_washer'
                frame=geometry._frame(configs[digit],j);core=[pin,washer,screw];members=core+[left_name,right_name];body=rows[pin]['body']
                if any(rows[n]['body']!=body for n in members):raise ValueError('Pin cluster bodies differ')
                local={n:geometry._local(parts[n],frame) for n in members}
                reconstruct={pin:difference(local[pin],builder.make_pin(cq)),washer:difference(local[washer],builder.make_washer(cq)),screw:difference(local[screw],builder.make_screw(cq))}
                for name,lo,hi in [(left_name,-13.,-12.4),(right_name,12.4,13.)]:
                    nominal=cylinder(3.7,lo,hi).cut(cylinder(2.05,lo-.01,hi+.01))
                    reconstruct[name]=difference(local[name],nominal)
                blankpath=bind(g/'machining_blanks'/f'{pin}_pilot_blank.step',blanks[pin]['sha256_step'])
                blank=cq.importers.importStep(str(blankpath)).val();single(blank)
                # Pilot blanks are exported in the same world frame as the named pin.
                blank_local=geometry._local(blank,frame)
                blank_diff=difference(blank_local,builder.make_pin(cq,machining_blank=True))
                outside_blank=float(local[pin].cut(blank_local,tol=1e-6).Volume())
                thread_removed=float(blank_local.cut(local[pin],tol=1e-6).Volume())
                if not math.isfinite(outside_blank) or abs(outside_blank)>1e-5 or not math.isfinite(thread_removed) or thread_removed<=0:raise ValueError('Finished pin is not a strict material subset of pilot blank')
                internal=[]
                for k,n1 in enumerate(members):
                    for n2 in members[k+1:]:
                        vol=float(parts[n1].intersect(parts[n2],tol=1e-6).Volume())
                        if not math.isfinite(vol) or vol<0:raise ValueError('Invalid internal cluster intersection')
                        internal.append(dict(a=n1,b=n2,overlap_mm3=vol,passed=vol<=1e-4))
                if not all(x['passed'] for x in internal):raise ValueError('Nominal cluster parts overlap')
                nominal_float=float(spec['nominal_axial_float_mm'])
                if abs(nominal_float-.15)>1e-9:raise ValueError('Axial domain was derived only for nominal0.15mm endplay')
                freedom_vertices=[]
                for t,s_left,s_right in [(-nominal_float,-nominal_float,0.),(-nominal_float,0.,0.),(0.,0.,0.),(0.,0.,nominal_float)]:
                    shifted={n:local[n].translate((t if n in core else s_left if n==left_name else s_right,0,0)) for n in members}
                    pairs=[]
                    for k,n1 in enumerate(members):
                        for n2 in members[k+1:]:
                            volume=float(shifted[n1].intersect(shifted[n2],tol=1e-6).Volume())
                            if not math.isfinite(volume) or volume<0 or volume>1e-4:raise ValueError('Correlated axial freedom creates internal overlap')
                            pairs.append(dict(a=n1,b=n2,overlap_mm3=volume))
                    freedom_vertices.append(dict(pin_shift_mm=t,left_washer_shift_mm=s_left,right_washer_shift_mm=s_right,pairs=pairs))
                right=geometry._local(parts[ident+'_right_pin_washer'],frame);left=geometry._local(parts[ident+'_left_pin_washer'],frame)
                right_face=float(v.bounds(right)[1][0]);left_face=float(v.bounds(left)[0][0])
                if abs(right_face-13.)>1e-5 or abs(left_face+13.)>1e-5 or abs(13.15-right_face-nominal_float)>1e-5:raise ValueError('Changed retention stack faces')
                envelope=geometry._world(cluster_envelope(nominal_float),frame);single(envelope)
                contains={n:float(parts[n].cut(envelope,tol=1e-6).Volume()) for n in members}
                if any(not math.isfinite(x) or abs(x)>1e-5 for x in contains.values()):raise ValueError('Retention cluster not contained')
                cluster_name=ident+'_retention_freedom_envelope';proxy[cluster_name]=envelope;proxyrows.append(dict(name=cluster_name,body=body,role='conservative_retention_freedom_proxy'))
                entry=dict(id=ident,members=members,parent_body_placement=body,reconstruction_difference_mm3=reconstruct,
                    machining_blank=dict(path=blankpath.relative_to(repo).as_posix(),sha256=digest(blankpath),reconstruction_difference_mm3=blank_diff,
                        assembly_material_outside_blank_mm3=outside_blank,
                        material_removed_by_simplified_thread_envelope_mm3=thread_removed,finished_part=False),
                    nominal_internal_intersections=internal,nominal_stack=dict(left_face_x_mm=left_face,right_face_x_mm=right_face,axial_float_mm=[-nominal_float,0.]),
                    cluster_proxy=dict(name=cluster_name,contains_original_outside_volume_mm3=contains,rotation_deg=[0,360],axial_float_mm=[-nominal_float,0.],
                        left_washer_shift_mm=[-nominal_float,0.],right_washer_shift_mm=[0.,nominal_float],
                        internal_intersections_checked_separately=True),
                    axial_freedom=dict(coaxial_only=True,domain='-d<=t<=0; t<=s_left<=0; 0<=s_right<=d+t; d=0.15mm',
                        vertex_checks=freedom_vertices,
                        continuous_internal_separation='Reconstructed coaxial annuli have Rinner2.05>shaftR2.0. Head right face -13+t<=left washer left face -13+s_left; right washer right face13+s_right<=keeper left face13.15+t. These inequalities hold throughout the declared domain, independently of circular spin. Other cluster interfaces remain rigid.',
                        not_modelled=['radial washer eccentricity','washer tilt','dimensional tolerances']),access={})
                access=entry['access'];travel=float(spec['approach_travel_mm'])
                for key,shape,excluded,meaning in [
                    ('pin_insertion',builder.pin_outer_envelope(cq,travel),core,'Conservative continuous outer pin envelope from local X=-30 to0; keeper/screw installed later. Existing outer washers remain as obstacles at their nominal seated positions.'),
                    ('keeper_insertion',builder.washer_sweep(cq,travel),[washer,screw],'Exact rigid washer translation from local X=+30 to0; screw installed later.'),
                    ('screw_outer_approach',builder.screw_outer_envelope(cq,travel),[screw],'Outer assembly approach envelope only. Actual threads require rotation; smooth-bore CAD is not a helical installation simulation.')]:
                    single(shape);solid=geometry._world(shape,frame);obstacles={n:s for n,s in parts.items() if n not in excluded}
                    access[key]=dict(**scoped(solid,obstacles,digit),excluded_parts=excluded,meaning=meaning)
                toolsolids={}
                for kind,target in [('hex',screw),('flat',pin)]:
                    installed=tool_gauge(spec,kind);swept=tool_gauge(spec,kind,travel);single(installed);single(swept)
                    toolsolids[kind]=geometry._world(installed,frame)
                    access[kind+'_driver_approach']=dict(**scoped(geometry._world(swept,frame),parts,digit),excluded_parts=[],meaning='Declared nominal aligned-tip access gauge; all saved parts including mating pin/screw remain. Unknown actual tip transition requires measurement.')
                    bad=geometry._world(tool_gauge(spec,kind,oversize_tip=True),frame)
                    positive=intersections(bad,{target:parts[target]})
                    if positive['passed']:raise ValueError('Oversize tool positive control was not detected')
                    entry[kind+'_oversize_tool_control']=positive
                simultaneous=float(toolsolids['hex'].intersect(toolsolids['flat'],tol=1e-6).Volume())
                if not math.isfinite(simultaneous) or simultaneous>1e-4:raise ValueError('Declared tools cannot coexist')
                entry['simultaneous_installed_tools_overlap_mm3']=simultaneous
                report['retention'].append(entry);save()
                print(json.dumps(dict(joint=ident,module_blocked=[k for k,x in access.items() if not x['finger_module']['passed']],whole_hand_blocked=[k for k,x in access.items() if not x['whole_hand_neutral']['passed']])),flush=True)
        if len(proxy)!=319 or len(proxyrows)!=319:raise ValueError('Wrong full-material proxy partition')
        report['proxy_partition']=dict(physical_part_count=359,unchanged_part_files=329,noncluster_physical_parts=309,retention_clusters=10,parts_per_cluster=5,collision_entity_count=319,
            loose_washer_names=sorted(loose_washers),covered_names=sorted(set(unchanged)|modified|added),no_material_excluded=True)
        gestures=copy.deepcopy(profile['catalog']['gestures'])
        for i,gesture in enumerate(gestures):
            if gesture['id']=='thumb_up':gestures[i]=dict(copy.deepcopy(profile['document']['gestures'][2]),id='thumb_up')
        gestures.append(copy.deepcopy(profile['document']['gestures'][1]));report['gestures']=gestures
        for gesture in gestures:profile['poses'].validate_angles(gesture['angles_deg'],design)
        path_obs,transitions=native.expand_poses(profile['document'],design,profile['poses'].NAMES,'full',5.)
        observations=[dict(id='static_'+x['id'],kind='endpoint',gesture_id=x['id'],angles_deg=x['angles_deg']) for x in gestures]
        observations.extend(copy.deepcopy([x for x in path_obs if x['kind']=='transition']))
        report['observations']=observations;report['full_path_definition']=transitions
        engine=native.PairScreen(v,proxy,proxyrows,design,1.,True);known={};last=time.monotonic()
        for obs in observations:
            key=tuple(float(x).hex() for x in obs['angles_deg'])
            if key not in known:
                case='pose_'+str(len(known)).zfill(4);known[key]=case;result=engine.evaluate(obs['angles_deg']);report['cases'][case]=result
                print(json.dumps(dict(case=case,observation=obs['id'],collisions=result['collision_count'],errors=len(result['operation_failures']))),flush=True)
            obs['case_id']=known[key]
            if time.monotonic()-last>15:save();last=time.monotonic()
        keys=['pin_insertion','keeper_insertion','screw_outer_approach','hex_driver_approach','flat_driver_approach']
        report['summary']=dict(physical_parts=359,collision_entities=319,modified_parts=10,removed_parts=10,added_parts=20,unchanged_parts=329,retention_sets=len(report['retention']),
            finger_module_blocked={k:sum(not x['access'][k]['finger_module']['passed'] for x in report['retention']) for k in keys},
            whole_hand_neutral_blocked={k:sum(not x['access'][k]['whole_hand_neutral']['passed'] for x in report['retention']) for k in keys},
            oversize_tool_controls_detected=2*len(report['retention']),static_pose_count=len(gestures),observation_count=len(observations),unique_pose_count=len(known),
            sampled_hand_path_count=sum(x['kind']=='transition' for x in observations),maximum_actual_joint_step_deg=transitions[0]['maximum_actual_joint_step_deg'],
            pose_collision_count=sum(x['collision_count'] for x in report['cases'].values()),operation_failure_count=sum(len(x['operation_failures']) for x in report['cases'].values()))
        report['completed']=True
    except Exception as exc:report['errors'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    save();print(json.dumps(dict(completed=report['completed'],summary=report.get('summary'),errors=report['errors'])),flush=True)
    return 0 if report['completed'] and report['input_hash_guard']['passed'] else 1
if __name__=='__main__':raise SystemExit(main())

"""FORM002 gesture definitions and explicit joint-space paths; NumPy only.

No CAD mutation, dynamics, angle clipping or collision approval. The authored
33-frame fist is preserved as a reference. Candidate paths are labelled apart.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
REFERENCE=ROOT/"cad/accepted_form_v1"
DIGITS=["index","middle","ring","little","thumb"]
NAMES=[f"{f}_{j}" for f in DIGITS[:4] for j in ("spread","mcp","pip","dip")]+[
    "thumb_base_sweep","thumb_opposition","thumb_mcp","thumb_ip"]
IDS=["open","fist","v_sign","scissors","thumb_up","middle_finger","mountain","rock_thumb_folded","rock_thumb_extended"]
FIST_THUMB_CORRECTION_DEG=np.array([-6.,3.,-8.,0.])
THUMB_UP_CANDIDATE_DEG=np.array([-10.,20.,40.,0.])


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def rotation(axis,degrees):
    a=math.radians(float(degrees));c,s=math.cos(a),math.sin(a)
    if axis=="x":return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    if axis=="y":return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    if axis=="z":return np.array([[c,-s,0],[s,c,0],[0,0,1]])
    raise ValueError("Unknown axis")


def transform(pos=(0,0,0),R=None):
    T=np.eye(4);T[:3,3]=pos
    if R is not None:T[:3,:3]=R
    return T


def joint_ranges(design):
    return np.array([design["joint_ranges_deg"][name if i>=16 else name.split("_",1)[1]] for i,name in enumerate(NAMES)],float)


def validate_angles(angles_deg,design):
    q=np.asarray(angles_deg,float)
    if q.shape!=(20,) or not np.isfinite(q).all():raise ValueError("Expected twenty finite angles")
    r=joint_ranges(design)
    if np.any(q<r[:,0]-1e-9) or np.any(q>r[:,1]+1e-9):
        bad=[NAMES[i] for i in range(20) if q[i]<r[i,0]-1e-9 or q[i]>r[i,1]+1e-9]
        raise ValueError("Gesture exceeds declared ranges; no clipping: "+", ".join(bad))
    return q


def forward_kinematics(design,angles_deg):
    """Return body frames and link endpoints in world millimetres."""
    q=validate_angles(angles_deg,design)
    bodies={"palm":np.eye(4)};tips={};chains={}
    for fi,f in enumerate(design["fingers"]+[design["thumb"]]):
        a=q[fi*4:fi*4+4];names=NAMES[fi*4:fi*4+4];L=f["links"]
        spread=transform(f["root"],rotation("z",f.get("base_yaw_deg",0)-a[0]))
        mcp=spread@transform(f.get("flexion_offset",[0,0,0]),rotation("x",a[1]))
        pip=mcp@transform([0,L[0],0],rotation("z",f.get("second_yaw_deg",0))@rotation("y",f.get("second_roll_deg",0)))@transform(R=rotation("x",a[2]))
        dip=pip@transform([0,L[1],0],rotation("x",a[3]))
        for name,T in zip(names,[spread,mcp,pip,dip]):bodies[name]=T
        points=[mcp[:3,3]]
        for name,T,length in zip(names[1:],[mcp,pip,dip],L):
            point=(T@np.array([0,length,0,1]))[:3]
            tips[name+"_tip"]=point;points.append(point)
        chains[DIGITS[fi]]=np.array(points)
    return dict(bodies=bodies,tips=tips,chains=chains)


def shape_metrics(design,angles_deg):
    fk=forward_kinematics(design,angles_deg);q=np.asarray(angles_deg,float)
    result={}
    directions={}
    for fi,f in enumerate(design["fingers"]+[design["thumb"]]):
        name=DIGITS[fi];p=fk["chains"][name];v=p[-1]-p[0];d=p[-1]-p[-2];d=d/np.linalg.norm(d);directions[name]=d
        result[name]=dict(tip_world_mm=p[-1].tolist(),joint_centres_world_mm=p[:-1].tolist(),
            root_to_tip_distance_mm=float(np.linalg.norm(v)),
            extension_ratio=float(np.linalg.norm(v)/sum(f["links"])),
            distal_direction_world=d.tolist(),flexion_sum_deg=float(q[4*fi+1:4*fi+4].sum()))
    angle=math.degrees(math.acos(float(np.clip(directions["index"]@directions["middle"],-1,1))))
    return dict(digits=result,index_middle_distal_angle_deg=angle,
        index_middle_tip_separation_mm=float(np.linalg.norm(fk["chains"]["index"][-1]-fk["chains"]["middle"][-1])),
        interpretation="Shape indicators only; centerlines do not test shell collision, clearance or subjective naturalness.")


def waypoint(fraction,q,phase):
    return dict(fraction=float(fraction),angles_deg=np.asarray(q,float).tolist(),phase=phase)


def _reverse(path):
    return [dict(p,fraction=float(1-p["fraction"]),phase="reverse_"+p["phase"]) for p in reversed(path)]


def generate_catalog(design=None,motion=None):
    design=copy.deepcopy(design or read_json(REFERENCE/"design.json"))
    motion=copy.deepcopy(motion or read_json(REFERENCE/"motion.json"))
    accepted=motion["frames"]
    if len(accepted)!=33 or not np.array_equal(accepted[-1]["angles_deg"],design["fist_deg"]):
        raise ValueError("Use the accepted 33-frame path and its exact endpoint")
    if not np.array_equal(np.array(design["fist_path"]),np.array([p["angles_deg"] for p in accepted])):
        raise ValueError("Accepted design and motion paths disagree")
    zero=np.zeros(20);reference_fist=np.asarray(design["fist_deg"],float)
    fist=reference_fist.copy();fist[16:]+=FIST_THUMB_CORRECTION_DEG
    # The thumb base moves outside the index finger, then MCP bends the distal
    # chain upright. IP remains straight. This differs from a wholly straight
    # thumb and does not relax the extension rule for the other gestures.
    extended_thumb=np.array([0.,20.,0.,0.])
    up_thumb=THUMB_UP_CANDIDATE_DEG.copy()
    specifications=[
        ("open","보",DIGITS,[],"open_accepted",zero),
        ("fist","바위",[],DIGITS,"folded_thumb_clearance_candidate",fist),
        ("v_sign","V",["index","middle"],["ring","little","thumb"],"folded",None),
        ("scissors","가위",["index","middle"],["ring","little","thumb"],"folded",None),
        ("thumb_up","엄지척",["thumb"],DIGITS[:4],"raised_with_mcp_bend_and_straight_ip",None),
        ("middle_finger","가운데손가락",["middle"],["index","ring","little","thumb"],"across_folded_digits",None),
        ("mountain","산: 엄지·중지·새끼",["thumb","middle","little"],["index","ring"],"extended_sideways",None),
        ("rock_thumb_folded","록: 엄지 접기",["index","little"],["middle","ring","thumb"],"folded",None),
        ("rock_thumb_extended","록: 엄지 펴기",["thumb","index","little"],["middle","ring"],"extended_sideways",None)]
    gestures=[]
    for ident,label,extended,folded,thumb_role,explicit in specifications:
        q=reference_fist.copy() if explicit is None else np.array(explicit,copy=True)
        if explicit is None:
            for name in extended:
                if name!="thumb":q[DIGITS.index(name)*4:DIGITS.index(name)*4+4]=0
            if "thumb" in extended:q[16:]=up_thumb if ident=="thumb_up" else extended_thumb
            if ident=="v_sign":q[0]=12.;q[4]=-12.
            if ident=="scissors":q[0]=-1.;q[4]=1.
            if ident=="rock_thumb_folded":q[16:]+=FIST_THUMB_CORRECTION_DEG
        validate_angles(q,design)
        gesture=dict(id=ident,label_ko=label,angles_deg=q.tolist(),extended_digits=extended,
            folded_digits=folded,thumb_role=thumb_role,status="shape_candidate_not_collision_verified",
            shape_metrics=shape_metrics(design,q))
        if ident in ("fist","rock_thumb_folded","thumb_up"):
            original=q.copy()
            if ident=="thumb_up":original[16:]=[-55.,45.,0.,0.]
            else:original[16:]=reference_fist[16:]
            gesture["authored_reference_angles_deg"]=original.tolist()
            gesture["pose_origin"]="thumb_only_corrected_candidate"
        gestures.append(gesture)
    byid={g["id"]:g for g in gestures}
    # Keep the complete authored path below in reference.authored_fist_path.
    # The candidate uses the same 33 fractions and four finger trajectories,
    # adding only a thumb offset proportional to reference MCP flexion. A
    # fraction-only offset would make the initially straight MCP negative;
    # the proportional definition avoids that by construction, without clips.
    # It has NOT been collision checked as
    # a transition; endpoint evidence must not be promoted to a path approval.
    candidate_fist_path=[]
    for i,p in enumerate(accepted):
        weight=float(p["angles_deg"][18])/reference_fist[18]
        q=np.array(p["angles_deg"],float);q[16:]+=weight*FIST_THUMB_CORRECTION_DEG
        candidate_fist_path.append(dict(p,angles_deg=q.tolist(),phase="candidate_thumb_corrected_"+str(i)))
    paths={"fist":candidate_fist_path}
    clear=zero.copy();clear[16:]=[25.,55.,0.,0.]
    for ident in IDS[2:]:
        target=np.array(byid[ident]["angles_deg"]);digits=target.copy();digits[16:]=clear[16:]
        paths[ident]=[waypoint(0,zero,"open"),waypoint(.2,clear,"thumb_clear_candidate"),
            waypoint(.75,digits,"digit_shape"),waypoint(1,target,"thumb_finish")]
    transitions=[]
    def add(source,target,path,method):
        for p in path:validate_angles(p["angles_deg"],design)
        transitions.append(dict(id=source+"__"+target,source=source,target=target,waypoints=path,
            method=method,status="candidate_path_not_collision_verified"))
    for target,path in paths.items():
        add("open",target,path,"candidate_33_frames_thumb_offset_proportional_mcp" if target=="fist" else "thumb_clear_then_digits_then_thumb")
        add(target,"open",_reverse(path),"exact_reverse_of_closing_path")
    a=np.array(byid["v_sign"]["angles_deg"]);b=np.array(byid["scissors"]["angles_deg"])
    add("v_sign","scissors",[waypoint(0,a,"v"),waypoint(1,b,"scissors")],"spread_only")
    add("scissors","v_sign",[waypoint(0,b,"scissors"),waypoint(1,a,"v")],"spread_only")
    lifted=fist.copy();lifted[16:]=[25.,55.,0.,0.]
    path=[waypoint(0,fist,"fist"),waypoint(.45,lifted,"unfold_thumb_candidate"),
          waypoint(1,byid["thumb_up"]["angles_deg"],"thumb_up")]
    add("fist","thumb_up",path,"thumb_only_two_stage")
    add("thumb_up","fist",_reverse(path),"exact_reverse_of_thumb_path")
    return dict(schema_version="gesture_hand_v3.poses.v1",joint_names=NAMES,units="degrees",
        reference=dict(name="accepted_form_v1 / FORM002",commit_prefix="a77ba87",
            authored_fist_deg=reference_fist.tolist(),authored_fist_path=copy.deepcopy(accepted),
            original_thumb_up_angles_deg=[*reference_fist[:16].tolist(),-55.,45.,0.,0.],
            source_sha256={p.relative_to(ROOT).as_posix():digest(p) for p in [REFERENCE/"design.json",REFERENCE/"motion.json",REFERENCE/"preview.xml"]}),
        candidate_pose_corrections=dict(
            fist_and_rock_folded_thumb_delta_deg=FIST_THUMB_CORRECTION_DEG.tolist(),
            thumb_up_angles_deg=THUMB_UP_CANDIDATE_DEG.tolist(),
            affected_gestures=["fist","rock_thumb_folded","thumb_up"],
            first_four_finger_angles_unchanged=True,
            selection_basis="Pose-only saved-STEP search before final fabrication repair; final all-part endpoint audit is separate.",
            local_clearance_goal_is_manufacturing_requirement=False,
            candidate_fist_path="Original 33 fractions and finger angles; thumb delta multiplied by reference MCP angle / final reference MCP angle. Transition collision status remains unverified."),
        kinematic_design={k:design[k] for k in ["units","fingers","thumb","joint_ranges_deg","palm_target_envelope_mm"]},
        gestures=gestures,transitions=transitions,
        interpolation="Piecewise joint-angle path. Linear angle interpolation for collision sampling; optional minimum-jerk reparameterization follows the same geometric segments.",
        limitations=["Angles are not silently clipped. A range violation is an error.",
            "No collision, cable routing, torque, latency, contact force or naturalness approval is implied.",
            "The authored reference 33-frame fist is preserved separately; the candidate fist path changes only thumb angles and is not collision verified.",
            "Authored fractions are not measured human timings. The thumb-up semantic rule permits MCP bend with straight IP; other extension rules are unchanged.",
            "Staged thumb-clear paths are explicit candidates, not proven safe paths.",
            "The FORM002 visual meshes remain a read-only reference; parent CAD reconstruction is separate."])


def load_catalog(path=None,verify_sources=True):
    c=read_json(path or HERE/"poses.json")
    if c["joint_names"]!=NAMES or [g["id"] for g in c["gestures"]]!=IDS:raise ValueError("Unexpected gesture/axis order")
    if verify_sources:
        for rel,sha in c["reference"]["source_sha256"].items():
            if digest(ROOT/rel)!=sha:raise ValueError("Accepted reference changed: "+rel)
    for g in c["gestures"]:validate_angles(g["angles_deg"],c["kinematic_design"])
    return c


def pose(name,catalog=None):
    c=catalog or load_catalog()
    return np.array(next(g["angles_deg"] for g in c["gestures"] if g["id"]==name),float)


def transition(source,target,mode="staged",catalog=None):
    c=catalog or load_catalog()
    if mode=="direct":return [waypoint(0,pose(source,c),"source"),waypoint(1,pose(target,c),"target")]
    if mode!="staged":raise ValueError("Expected direct/staged mode")
    found=[t for t in c["transitions"] if t["source"]==source and t["target"]==target]
    if found:return copy.deepcopy(found[0]["waypoints"])
    if source==target:return [waypoint(0,pose(source,c),"unchanged"),waypoint(1,pose(target,c),"unchanged")]
    # Other pairs are explicit paths through the accepted open pose.
    a=transition(source,"open",catalog=c);b=transition("open",target,catalog=c)
    return [dict(p,fraction=p["fraction"]*.5) for p in a]+[dict(p,fraction=.5+p["fraction"]*.5) for p in b[1:]]


def sample_path(path,max_step_deg=5.,design=None):
    """Spatial samples; every waypoint retained, no rounding/clipping of angles."""
    if not math.isfinite(max_step_deg) or max_step_deg<=0:raise ValueError("Positive sample step required")
    result=[]
    for segment,(a,b) in enumerate(zip(path,path[1:])):
        qa=np.asarray(a["angles_deg"],float);qb=np.asarray(b["angles_deg"],float)
        if b["fraction"]<=a["fraction"]:raise ValueError("Path fractions must increase")
        if design is not None:validate_angles(qa,design);validate_angles(qb,design)
        count=max(1,math.ceil(float(np.max(np.abs(qb-qa)))/max_step_deg))
        for j in range(count+1):
            if segment and j==0:continue
            u=j/count;q=qa+(qb-qa)*u
            result.append(dict(segment=segment,segment_fraction=u,
                fraction=(1-u)*a["fraction"]+u*b["fraction"],angles_deg=q.tolist()))
    return result


def _xml_forward(xml,angles_deg):
    """Independent XML body/joint/site interpreter, without importing active app."""
    q=dict(zip(NAMES,np.deg2rad(angles_deg)));bodies={};sites={}
    def visit(node,parent):
        T=np.eye(4);T[:3,3]=np.fromstring(node.get("pos","0 0 0"),sep=" ")*1000
        w,x,y,z=np.fromstring(node.get("quat","1 0 0 0"),sep=" ");n=math.sqrt(w*w+x*x+y*y+z*z);w,x,y,z=np.array([w,x,y,z])/n
        T[:3,:3]=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        W=parent@T
        for j in node.findall("joint"):
            axis=np.fromstring(j.get("axis","0 0 1"),sep=" ");axis/=np.linalg.norm(axis)
            x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);a=q[j.get("name")]
            p=np.fromstring(j.get("pos","0 0 0"),sep=" ")*1000
            W=W@transform(p)@transform(R=np.eye(3)+np.sin(a)*K+(1-np.cos(a))*(K@K))@transform(-p)
        bodies[node.get("name")]=W
        for s in node.findall("site"):
            sites[s.get("name")]=(W@np.r_[np.fromstring(s.get("pos","0 0 0"),sep=" ")*1000,1])[:3]
        for child in node.findall("body"):visit(child,W)
    for b in xml.findall("./worldbody/body"):visit(b,np.eye(4))
    return bodies,sites


def validate_catalog(catalog):
    c=catalog;d=c["kinematic_design"];motion=read_json(REFERENCE/"motion.json")["frames"]
    xml=ET.parse(REFERENCE/"preview.xml");max_pos=max_rot=max_tip=0.
    candidate_path=transition("open","fist",catalog=c)
    poses=[p["angles_deg"] for p in motion]+[p["angles_deg"] for p in candidate_path]+[g["angles_deg"] for g in c["gestures"]]
    for q in poses:
        fk=forward_kinematics(d,q);xb,xs=_xml_forward(xml,q)
        for n in NAMES:
            max_pos=max(max_pos,float(np.max(np.abs(fk["bodies"][n][:3,3]-xb[n][:3,3]))))
            max_rot=max(max_rot,float(np.max(np.abs(fk["bodies"][n][:3,:3]-xb[n][:3,:3]))))
        for n,tip in fk["tips"].items():max_tip=max(max_tip,float(np.linalg.norm(tip-xs[n])))
    if max_pos>1e-7 or max_rot>1e-9 or max_tip>1e-7:raise ValueError("FK differs from accepted XML")
    if c["reference"]["authored_fist_path"]!=motion or c["reference"]["authored_fist_deg"]!=motion[-1]["angles_deg"]:
        raise ValueError("Authored reference fist path was altered")
    if len(candidate_path)!=33:raise ValueError("Candidate fist must retain the 33 reference fractions")
    for p,original in zip(candidate_path,motion):
        if p["fraction"]!=original["fraction"] or p["angles_deg"][:16]!=original["angles_deg"][:16]:
            raise ValueError("Candidate changed reference fractions or first-four-finger angles")
        weight=float(original["angles_deg"][18])/float(motion[-1]["angles_deg"][18])
        expected=np.asarray(original["angles_deg"])[16:]+weight*FIST_THUMB_CORRECTION_DEG
        if not np.array_equal(p["angles_deg"][16:],expected):raise ValueError("Unexpected candidate thumb interpolation")
    for path in c["transitions"]:
        if path["waypoints"][0]["angles_deg"]!=pose(path["source"],c).tolist() or path["waypoints"][-1]["angles_deg"]!=pose(path["target"],c).tolist():
            raise ValueError("Transition endpoint differs from named gesture")
    shapes={g["id"]:shape_metrics(d,g["angles_deg"]) for g in c["gestures"]}
    semantic=[]
    for g in c["gestures"]:
        m=shapes[g["id"]]
        for digit in g["extended_digits"]:
            if g["id"]=="thumb_up" and digit=="thumb":
                # Morphological indicator only: a bent MCP can point a straight
                # distal thumb up without making the whole chain 95% straight.
                if abs(g["angles_deg"][19])>5 or m["digits"][digit]["distal_direction_world"][1]<math.cos(math.radians(45)):
                    raise ValueError("Thumb-up requires near-straight IP and distal direction within 45 degrees of +Y")
                continue
            if m["digits"][digit]["extension_ratio"]<.95:raise ValueError("Extended digit is curled: "+g["id"]+" "+digit)
        for digit in g["folded_digits"]:
            if digit!="thumb" and m["digits"][digit]["extension_ratio"]>.5:raise ValueError("Folded finger is too extended")
        semantic.append(dict(gesture=g["id"],extension_categories_match=True,
            thumb_up_rule=(dict(ip_absolute_max_deg=5,distal_world_y_min=math.cos(math.radians(45)),
                whole_thumb_extension_ratio_is_not_this_rule=True,manufacturing_or_naturalness_requirement=False)
                if g["id"]=="thumb_up" else None)))
    if shapes["v_sign"]["index_middle_distal_angle_deg"]<15 or shapes["scissors"]["index_middle_distal_angle_deg"]>5:
        raise ValueError("V/scissors distinction lost")
    samples=sum(len(sample_path(t["waypoints"],design=d)) for t in c["transitions"])
    bad=np.zeros(20);bad[0]=23
    try:validate_angles(bad,d)
    except ValueError:pass
    else:raise AssertionError("Out-of-range gesture was clipped/accepted")
    return dict(gestures=len(c["gestures"]),transitions=len(c["transitions"]),path_samples_at_5deg=samples,
        fk_pose_count=len(poses),max_xml_body_position_difference_mm=max_pos,max_xml_rotation_component_difference=max_rot,
        max_xml_tip_difference_mm=max_tip,accepted_33_fist_frames_exact=True,
        accepted_frames_location="reference.authored_fist_path",candidate_fist_frames=33,
        candidate_first_four_finger_frames_exact=True,candidate_thumb_correction_explicit=True,
        all_transition_endpoints_match=True,range_clipping_rejected=True,
        shape_category_checks=semantic,shape_metrics=shapes,
        status="kinematic_shape_consistency_only",collision_passed=False,naturalness_approved=False,
        source_sha256={**c["reference"]["source_sha256"],Path(__file__).relative_to(ROOT).as_posix():digest(__file__)})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--generate",action="store_true")
    p.add_argument("--output",type=Path,default=HERE/"poses.json");p.add_argument("--report",type=Path,required=True)
    a=p.parse_args();c=generate_catalog() if a.generate else load_catalog(a.output)
    report=validate_catalog(c)
    if a.generate:a.output.write_text(json.dumps(c,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
    report["poses_sha256"]=digest(a.output)
    a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({k:report[k] for k in ["gestures","transitions","fk_pose_count","max_xml_tip_difference_mm","status"]}),flush=True)

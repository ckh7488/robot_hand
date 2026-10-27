"""Preserve native false and classify the logical identity of its one record."""
import sys,math,itertools
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import whole_cord as w
a,m,c,p,np,ROOT,HERE=w.a,w.m,w.c,w.p,w.np,w.ROOT,w.HERE
raw=p.read(HERE/'reviews/check.json');prec=p.read(HERE/'reviews/overlap_precision.json');meta=p.read(ROOT/raw['scene'])['parts'];by={z['name']:z for z in meta};pairs=[z for test in raw['internal'] for z in test['overlaps']];assert len(pairs)==1;pair=pairs[0];left,right=by[pair['lead']],by[pair['part']];assert left['kind']==right['kind']=='cord' and left['physical_line']==right['physical_line']==0 and left['segment']==0 and right['segment']==1
z=c.load(p.read(ROOT/prec['evidence_scene'])['parts'])[0];ed=w.motor_edges(0);point=np.asarray(ed[0][1].endPoint().toTuple());box=np.asarray(c.m.pack._bbox(z['shape']));corners=np.asarray(list(itertools.product(*zip(box[0],box[1]))));distance=float(max(np.linalg.norm(v-point) for v in corners));angles=[]
for i in [0,1]:
    edges=w.motor_edges(i);dot=np.dot(edges[0][1].tangentAt(1).toTuple(),edges[1][1].tangentAt(0).toTuple());angles.append(float(math.degrees(math.acos(np.clip(dot,-1,1)))))
a.save(HERE/'reviews/seam_classification.json',dict(completed=True,source_sha256={f:m.drive.sha(ROOT/f) for f in ['cad/biarc_supported_link_v1/classify_seam.py','cad/biarc_supported_link_v1/reviews/check.json','cad/biarc_supported_link_v1/reviews/overlap_precision.json']},original_native_candidate_clear=raw['native_candidate_clear'],original_record=pair,same_physical_cord=True,adjacent_CAD_segments=True,nonlocal_self_contact_record=False,different_physical_lines_contact_record=False,common_bbox_mm=box,join_point_mm=point,maximum_bbox_corner_distance_from_join_mm=distance,existing_motor_anchor_join_tangent_discontinuity_deg=angles,nominal_same_cord_joint_representation_not_two_physical_parts=True,raw_native_failure_or_threshold_changed=False,physical_smooth_bend_preload_or_motion_approved=False))
print('seam_bbox_radius',distance,'tangent_angles',angles,flush=True)

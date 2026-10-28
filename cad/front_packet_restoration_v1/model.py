"""Original front packets, with the completed special module kept as an obstacle."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm078_prior',ROOT/'cad/individual_base_nut_assembly_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
OLD,NAME=a.OLD,a.NAME
translate,moved,bbox,project_bounds,outside_start,inward_steps,bench_vectors=a.translate,a.moved,a.bbox,a.project_bounds,a.outside_start,a.inward_steps,a.bench_vectors


def context():
    rows,membership,source,packets=a.context()
    for f in [Path(__file__),ROOT/'cad/individual_base_nut_assembly_v1/reviews/check.json',ROOT/'cad/eight_bank_layout_v1/reviews/check.json',ROOT/'cad/wire_guides_v1/generated/scene.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source,packets


def corners(bb):
    return np.array([[x,y,z] for x in [bb[0][0],bb[1][0]] for y in [bb[0][1],bb[1][1]] for z in [bb[0][2],bb[1][2]]])


def enrich_vectors(vectors,moving,fixed,boxes):
    """Add projected AABB entry/mid/exit events, without claiming a swept proof."""
    vectors=[np.array(v,dtype=float) for v in vectors];out=[dict(translation_mm=vectors[0],event_boundary=True,events=[])];all_events=[]
    for j,(start,end) in enumerate(zip(vectors,vectors[1:])):
        length=float(np.linalg.norm(start-end))
        if length<1e-9:continue
        axis=(start-end)/length;seed=np.eye(3)[int(np.argmin(abs(axis)))];u=np.cross(axis,seed);u/=np.linalg.norm(u);v=np.cross(axis,u);events=[]
        for lead in moving:
            mc=boxes[lead]+end
            for part in fixed:
                fc=boxes[part]
                if any((mc@ax).max()<(fc@ax).min()-1e-8 or (fc@ax).max()<(mc@ax).min()-1e-8 for ax in [u,v]):continue
                lo=float((fc@axis).min()-(mc@axis).max());hi=float((fc@axis).max()-(mc@axis).min())
                if hi<0. or lo>length:continue
                for kind,t in [('entry',hi),('middle',(lo+hi)/2.),('exit',lo)]:
                    if 1e-8<t<length-1e-8:events.append(dict(segment=j,lead=lead,part=part,kind=kind,distance_to_segment_end_mm=t,translation_mm=end+t*axis))
        positions={round(z['distance_to_segment_end_mm'],8) for z in events}
        for t in sorted(positions,reverse=True):
            same=[z for z in events if round(z['distance_to_segment_end_mm'],8)==t];out.append(dict(translation_mm=end+t*axis,event_boundary=True,events=same))
        out.append(dict(translation_mm=end,event_boundary=j==len(vectors)-2,events=[]));all_events.extend(events)
    return out,all_events


def replay_samples(path):
    ii={0,len(path['samples'])-1}
    if not path['completed']:ii.update(range(len(path['samples'])))
    elif path['stage']=='world':ii.update(range(max(0,len(path['samples'])-5),len(path['samples'])))
    return [path['samples'][i] for i in sorted(ii)]

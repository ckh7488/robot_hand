"""Conservative saved neutral STEP boxes;optional validity fields stay optional."""
import itertools
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as g
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def hand_boxes(objects,source=None):
    v=g.module('_bus_bbox_fk',ROOT/'cad/articulation_drive_v2/verify.py')
    catalog=p.read(ROOT/'cad/functional_motion_v3/generated/catalog.json')
    scene=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    neutral=v.frames(catalog['design'],np.zeros(20));cases=[];cache={}
    if source is not None:
        for z in scene['hand']:source.update({z['step']:z['sha256_step'],z['mesh']:z['sha256_stl']})
        source.update(catalog['input_sha256'])
    bounds=[np.asarray(m.pack._bbox(z['shape'])) for z in objects]
    for case in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases']:
        F=v.frames(catalog['design'],case['angles_deg']);candidates=[];exact=[];gaps=[]
        for row in scene['hand']:
            box=np.asarray(row['bbox_mm']);assert box.shape==(2,3) and np.all(np.isfinite(box)) and np.all(box[1]>=box[0])
            if 'valid' in row:assert row['valid']
            corners=np.array(list(itertools.product(*zip(box[0]-.002,box[1]+.002))))
            origin,R=F[row['body']];oldorigin,oldR=neutral[row['body']]
            relative=R@oldR.T;t=origin-relative@oldorigin
            pts=corners@relative.T+t;posed=np.array([pts.min(axis=0),pts.max(axis=0)])
            for obj,other in zip(objects,bounds):
                overlap=np.minimum(posed[1],other[1])-np.maximum(posed[0],other[0])
                if np.any(overlap<=0):gaps.append(float(np.max(-overlap)));continue
                candidates.append([obj['name'],row['name']])
                if row['name'] not in cache:cache[row['name']]=cq.importers.importStep(str(ROOT/row['step'])).val()
                moved=v.relocate(cache[row['name']],F[row['body']],neutral[row['body']])
                exact.append(c.material([obj],[dict(name=row['name'],shape=moved)]))
        cases.append(dict(id=case['id'],angles_deg=case['angles_deg'],pairs_in_scope=len(objects)*364,
            transformed_saved_neutral_bbox=True,numerical_bbox_padding_mm=.002,native_candidates=candidates,native_refinements=exact,
            minimum_separating_axis_gap_mm=min(gaps),clear=all(g.clear(z) for z in exact)))
    return dict(cases=cases,hand_parts_hashed=364,hand_parts_native_imported=len(cache),
        method='Eight corners of saved neutral STEP AABBs,rigid FK transform,0.002mm numerical padding.Native refinement for every overlapping box only.Optional metadata validity flag is checked when present.No full hand self-collision/path or wrist/arm movement replay.')

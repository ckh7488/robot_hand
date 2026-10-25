"""Explicit module insertion samples against the parts installed at that stage."""
import math
import numpy as np
from common import bounds,overlap

def run(cq,geo,rows,parts,declarations):
    results=[]
    for d in declarations:
        moving=d['moving_parts']; installed=d['installed_obstacles']; absent=d['absent_at_stage']
        if not moving or not installed or len(set(moving+installed+absent))!=len(rows) or set(moving+installed+absent)!=set(rows):
            raise ValueError('Group assembly must partition all named physical parts exactly')
        start=np.asarray(d['initial_translation_mm'],dtype=float); end=np.asarray(d['final_translation_mm'],dtype=float)
        step=float(d['maximum_step_mm']); length=float(np.linalg.norm(start-end))
        if start.shape!=(3,) or end.shape!=(3,) or not np.isfinite([*start,*end,step]).all() or np.linalg.norm(end)>1e-9 or step<=0 or length<=0:
            raise ValueError('Invalid insertion path')
        count=max(1,math.ceil(length/step)); boxes={n:bounds(parts[n]) for n in installed}; hits=[]; checks=0; exact=0
        def aggregate(group):
            values=[bounds(parts[n]) for n in group]
            return [min(b[i] for b in values) for i in range(3)]+[max(b[i+3] for b in values) for i in range(3)]
        moving_box=aggregate(moving); obstacle_box=aggregate(installed)
        shifted=[moving_box[i]+start[i%3] for i in range(6)]
        gaps=[max(shifted[i]-obstacle_box[i+3],obstacle_box[i]-shifted[i+3]) for i in range(3)]
        aggregate_gap=max(gaps)
        # A union's AABB fills empty space between the common plate and parent
        # coupon. Require separation of every actual moving/installed pair,
        # rather than treating that empty region as solid material.
        start_gaps=[]
        for name in moving:
            mb=bounds(parts[name]); sb=[mb[k]+start[k%3] for k in range(6)]
            for other in installed:
                ob=boxes[other]
                start_gaps.append(max(max(sb[k]-ob[k+3],ob[k]-sb[k+3]) for k in range(3)))
        gap=min(start_gaps)
        if gap<1.-1e-6: raise ValueError('Actual saved module does not start fully outside with 1mm AABB margin')
        for i in range(count+1):
            delta=start+(end-start)*i/count
            for name in moving:
                shape=parts[name].translate(delta.tolist()); box=bounds(shape)
                for other in installed:
                    checks+=1
                    if overlap(box,boxes[other]):
                        exact+=1; v=geo.describe(geo.boolean(cq,'common',shape,parts[other]))['volume_mm3']
                        if v>1e-4: hits.append(dict(sample=i,moving=name,obstacle=other,overlap_mm3=v))
        results.append(dict(d,checks=checks,exact_pairs=exact,samples=count+1,maximum_actual_step_mm=length/count,
            actual_start_AABB_gap_mm=aggregate_gap,actual_start_pairwise_AABB_gap_mm=gap,
            actual_moving_final_AABB_mm=moving_box,actual_obstacle_AABB_mm=obstacle_box,
            collisions=hits,passed=not hits,continuous_sweep=False,
            scope='Actual saved group, nominal straight insertion samples only. Fixtures/hands used to hold the subassembly and tolerance-dependent maneuvering are not modelled.'))
    return results

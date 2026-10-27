"""Separate supported turnback layout on the existing motor and saddle seats.

Motor axes, rack, accepted hand and electrical geometry stay unchanged.
These are rigid guide candidates, not tensioners or prescribed free curves.
"""
import sys, math
from pathlib import Path
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'cad/parent_wrist_material_motion_v1'))
import run_motion as old
n, k = old.n, old.k
np, cq = n.np, n.cq
b = n.m.drive.module('_pack049_motor_basis', ROOT/'cad/support_attachment_basis_v1/bend_model.py')
feed = n.m.drive.module('_pack049_saddle_basis', ROOT/'cad/wrist_external_feed_v1/feed_model.py')


def motor_edges():
    A, B, M, D = [cq.Vector(*v) for v in [(0,-12,0),(0,-24,0),(0,-64,-40),(0,-24,-80)]]
    return [cq.Edge.makeLine(A,B), cq.Edge.makeThreePointArc(B,M,D)]


def saddle_edges(i):
    # Nested R40/R60 bends keep the two real OD5 bores apart as they turn.
    # A common R40 with a Z5 translation would merge at the vertical mouths.
    radius = [40.,60.][i]
    z = [-3.,2.][i]
    A = cq.Vector(100,-44-radius,z-radius)
    B = cq.Vector(100,-44,z)
    M = cq.Vector(100,-44-radius/math.sqrt(2),z-radius+radius/math.sqrt(2))
    D = cq.Vector(100,-4,z)
    return [cq.Edge.makeThreePointArc(A,M,B),cq.Edge.makeLine(B,D)]


def build():
    source = {}
    rel = 'cad/sheath_retention_v1/selected/generated/parts/scene.json'
    source[rel] = n.m.drive.sha(ROOT/rel)
    names = {'motor_housing_clamp_'+str(i)+'_jaw_-1' for i in [0,1]}
    original_pack = n.c.load([z for z in n.p.read(ROOT/rel)['parts'] if z['name'] in names],source)
    assert {z['name'] for z in original_pack}==names
    rows, design = [], []
    for i in [0,1]:
        name = 'motor_housing_clamp_'+str(i)+'_jaw_-1'
        jaw = next(z for z in original_pack if z['name']==name)
        Q = b.frame(i)
        tube = b.sweep(motor_edges(),2.5).cut(b.sweep(motor_edges(),1.6)).clean()
        sh = jaw['shape'].fuse(n.g.original.transform(tube,Q)).clean()
        lost = jaw['shape'].cut(sh,tol=1e-6)
        removed = lost.Volume() if lost.Solids() else 0.
        assert sh.isValid() and len(sh.Solids())==1 and removed<1e-8
        rows.append(dict(name='compact_motor_turnback_'+str(i),shape=sh,body='forearm',
            physical_line=i,installed_part_candidate=True,replaces='motor_'+str(i)+'_jaw_with_fixed_bend',
            preserved_original_jaw=name,color=[.93,.48,.1]))
        edges = motor_edges()
        design.append(dict(id='motor_'+str(i),Q=Q,original_jaw_removed_mm3=removed,
            centerline_radius_mm=40.,turn_degrees=180.,lead_mm=12.,
            shoulder_world_mm=(Q@np.array([0,5,0,1]))[:3],
            free_port_world_mm=(Q@np.r_[edges[-1].endPoint().toTuple(),1.])[:3],
            free_tangent_toward_hand=Q[:3,:3]@np.array(edges[-1].tangentAt(1).toTuple()),
            shoulder_to_free_port_length_mm=17.+sum(e.Length() for e in edges),
            nominal_reaction_path='OD3 housing -> OD5/bore3.2 integral lower jaw -> existing opposite jaw/two M2 -> captured adjuster head -> terminal support/two M3 -> original forearm motor mount',
            fixed_motor_and_fastener_seats_preserved=True))
    box, cyl = feed.box, feed.cyl
    # Retain the existing saddle contact faces, screw seats and bridge endpoint.
    # Replace the old long sleeves, rather than adding a second guide body.
    sh = box([45,-9,-6],[62,-6,6]).fuse(box([54,-12,-6],[102,-9,4]))
    sh = sh.fuse(box([54,-9,-58],[62,-6,6])).fuse(box([45,-9,-58],[62,-6,-46]))
    sh = sh.fuse(box([54,-6,-58],[56,0,-46]))
    for i in [0,1]:
        sh = sh.fuse(b.sweep(saddle_edges(i),2.5))
    for i in [0,1]:
        sh = sh.cut(b.sweep(saddle_edges(i),1.6))
    for z in [-55.,-49.]:
        sh = sh.cut(cyl(59,z,-14,6,1.1))
    sh = sh.clean()
    assert sh.isValid() and len(sh.Solids())==1
    rows.append(dict(name='compact_saddle_nested_turns',shape=sh,body='palm',
        installed_part_candidate=True,replaces='wrist_paired_rear_S_guide',
        attachment_body='Palm_wrist_saddle',color=[.93,.48,.1]))
    for i in [0,1]:
        edges = saddle_edges(i)
        design.append(dict(id='saddle_'+str(i),centerline_radius_mm=[40.,60.][i],turn_degrees=90.,
            free_port_world_mm=edges[0].startPoint().toTuple(),free_tangent_toward_hand=edges[0].tangentAt(0).toTuple(),
            front_port_world_mm=edges[-1].endPoint().toTuple(),front_tangent_toward_hand=edges[-1].tangentAt(1).toTuple(),
            centerline_length_mm=sum(e.Length() for e in edges),
            nominal_reaction_path='housing sliding bore -> integral nested tubes/bridge/spine -> existing back/front saddle clamp faces/two M2 -> Palm_wrist_saddle',
            existing_saddle_faces_y_mm=[-6,0],existing_screw_axes_xz_mm=[[59,-55],[59,-49]],
            guide_moves_with_wrist_H=True))
    return rows,design,source


def posed(rows,case):
    return [dict(z,shape=n.g.original.transform(z['shape'],np.array(case['H']))) if z['body']=='palm' else z for z in rows]


def export(rows,label):
    return n.a.a.prior.export([{key:value for key,value in z.items() if key not in ['step','mesh','sha256_step','sha256_stl']} for z in rows],HERE/'generated'/label)

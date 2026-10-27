"""Follow the same material witness beyond the first straight bore section."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_motion as d
n=d.n;ROOT=d.ROOT;HERE=d.HERE


def evaluate(case,profile):
    witness=profile['axis_sliding_witness'];station=witness['location_from_rear_mm']
    guide=n.fixed_edges(case,profile['line'],profile['kind'])[1];length=sum(z.Length() for z in guide)
    point=n.np.array(witness['world_mm']);inside=0<=station<=length
    expected=n.np.array(n.t.point_at(guide,station)) if inside else None
    error=float(n.np.linalg.norm(point-expected)) if inside else None
    segment_index=None;segment_kind=None;tangent=None;segment_station=None
    if inside:
        assert error<1e-5
        remaining=station
        for j,edge in enumerate(guide):
            size=edge.Length()
            if remaining<=size+1e-8:
                segment_index=j;segment_kind=edge.geomType();segment_station=remaining
                tangent=edge.tangentAt(min(1.,remaining/size)).toTuple();break
            remaining-=size
    return dict(kind=profile['kind'],line=profile['line'],material_s_mm=witness['material_s_mm'],world_mm=point,
        guide_path_station_mm=station,real_PAIR_centerline_length_mm=length,inside_actual_PAIR_path=inside,
        inside_first_40mm_straight=bool(0<=station<=40),actual_guide_centerline_point_mm=expected,centerline_coincidence_error_mm=error,
        actual_guide_segment_index=segment_index,actual_guide_segment_type=segment_kind,segment_station_mm=segment_station,actual_path_tangent_world=tangent,
        station_or_material_point_changed_to_keep_inside=False,friction_or_force_verified=False)


def main():
    out=HERE/'reviews/guide_witnesses.json';assert not out.exists();raw=n.p.read(HERE/'reviews/check.json');assert raw['completed']
    cases={z['id']:z['case'] for z in raw['states']};values=[]
    for state in raw['profiles']:
        values.append(dict(id=state['id'],witnesses=[evaluate(cases[state['id']],z) for z in state['materials']]))
    source={rel:n.m.drive.sha(ROOT/rel) for rel in ['cad/parent_wrist_material_motion_v1/reviews/check.json','cad/parent_wrist_material_motion_v1/guide_witnesses.py',
        'cad/coupled_material_continuation_v1/continuation.py']}
    n.a.save(out,dict(completed=True,source_sha256=source,states=values,material_witness_count=sum(len(z['witnesses']) for z in values),
        native_Boolean_groups_reexecuted=0,old_first_40mm_axis_test_not_promoted_to_whole_guide_straightness=True,
        same_material_points_not_reset=True,actual_physical_sliding_force_or_contact_not_verified=True))


if __name__=='__main__':main()

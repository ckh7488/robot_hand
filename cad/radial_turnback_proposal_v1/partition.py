"""Locate reported material within the retained jaw, without repeating full groups."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm055_partition_post', Path(__file__).with_name('postprocess.py'))
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
n, np, ROOT, HERE, a = p.n, p.np, p.ROOT, p.HERE, p.a


def setup(raw, evidence):
    source = {}
    _, base, _ = p.v.metadata()
    pack = n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    current = [z for z in base if z['name'] not in pack['replaced_hardware']]+n.p.read(ROOT/pack['scene'])['parts']
    old = n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')
    rows = [z for z in current if z['name'] != 'Forearm_twenty_axis_module_rack' and z['name'] not in a.OLD_NAMES]
    rows += n.p.read(ROOT/old['scene'])['parts']+n.p.read(ROOT/raw['scene'])['parts']
    delta = {name: np.array(z['transform_from_current']) for z in old['placements'] for name in z['rows']}
    hits = [z for test in raw['native'].values() for z in test['overlaps']]
    names = {z[k] for z in hits for k in ['lead','part']}
    selected = [z for z in rows if z['name'] in names]
    assert len(selected) == len(names)
    loaded = n.c.load(selected,source)
    shapes = {z['name']:n.g.original.transform(z['shape'],delta.get(z['name'],np.eye(4))) for z in loaded}
    jawmeta = n.p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts']
    jaws = n.c.load([z for z in jawmeta if z['name'] in {'motor_housing_clamp_'+str(i)+'_jaw_-1' for i in [0,1]}],source)
    thumb = np.array(next(z for z in old['placements'] if z['module'] == 'thumb_ip_actuator')['transform_from_current'])
    jawshapes = {i:n.g.original.transform(next(z['shape'] for z in jaws if z['name']=='motor_housing_clamp_'+str(i)+'_jaw_-1'),thumb) for i in [0,1]}
    witness = evidence['actual_common'][0]
    common = n.c.load(n.p.read(ROOT/witness['scene'])['parts'],source)[0]['shape']
    return shapes,jawshapes,witness,common,source


def classify(raw,evidence):
    shapes,jaws,witness,saved_common,source = setup(raw,evidence)
    result = []
    for label,test in raw['native'].items():
        for record in test['overlaps']:
            assert label == 'new2__retained870_neutral'
            i = int(record['lead'].rsplit('_',1)[1])
            reused = record == witness['record']
            common = saved_common if reused else shapes[record['lead']].intersect(shapes[record['part']],tol=1e-6)
            outside = common.cut(jaws[i],tol=1e-6)
            volume = common.Volume() if common.Solids() else 0.
            outside_volume = outside.Volume() if outside.Solids() else 0.
            result.append(dict(group=label,record=record,reused_postprocess_common_STEP=reused,
                reconstructed_common_volume_mm3=volume,
                common_volume_difference_from_raw_mm3=volume-record['material_intersection_mm3'],
                common_outside_original_unbent_jaw_mm3=outside_volume,
                outside_original_jaw_solids=len(outside.Solids()),
                full_native_groups_reexecuted=0))
    return result,source


def main():
    out = HERE/'reviews/partition.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/check.json')
    evidence = n.p.read(HERE/'reviews/evidence.json')
    rows,source = classify(raw,evidence)
    for f in [Path(__file__),HERE/'reviews/check.json',HERE/'reviews/evidence.json']:
        source[f.relative_to(ROOT).as_posix()] = n.m.drive.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,records=rows,
        records_classified=len(rows),new_pair_common_constructions=sum(not z['reused_postprocess_common_STEP'] for z in rows),
        reused_existing_common_count=sum(z['reused_postprocess_common_STEP'] for z in rows),
        common_minus_retained_jaw_queries=len(rows),
        all_reported_common_inside_retained_jaw=all(z['outside_original_jaw_solids']==0 for z in rows),
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        full26_native_groups_reexecuted=False,
        all_surface_trim_or_full_assembly_equivalence_proven=False,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items())))
    print('partition',[(z['record']['lead'],z['record']['part'],z['common_outside_original_unbent_jaw_mm3']) for z in rows],flush=True)


if __name__ == '__main__':
    main()

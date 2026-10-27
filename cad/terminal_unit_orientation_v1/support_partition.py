"""Locate the saved maximum common material in the rotated original support."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm056_partition_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,ROOT,HERE=a.n,a.np,a.ROOT,a.HERE


def query(raw,evidence):
    source={}
    rel='cad/motor_tendon_terminal_v1/selected/generated/parts/scene.json'
    source[rel]=n.m.drive.sha(ROOT/rel)
    old=n.c.load([z for z in n.p.read(ROOT/rel)['parts'] if z['name']==a.FRAME],source)[0]['shape']
    layout=n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')
    T=np.array(next(z for z in layout['placements'] if z['module']=='thumb_ip_actuator')['transform_from_current'])
    S=np.array(raw['design']['transform_from_ARM055'])
    old=n.g.original.transform(old,S@T)
    witness=evidence['actual_common'][0]
    assert witness['group']=='changed19__unchanged853_neutral'
    common=n.c.load(n.p.read(ROOT/witness['scene'])['parts'],source)[0]['shape']
    outside=common.cut(old,tol=1e-6)
    return dict(record=witness['record'],common_volume_mm3=common.Volume(),
        common_outside_rotated_original_frame_mm3=outside.Volume() if outside.Solids() else 0.,
        outside_solids=len(outside.Solids()),new_native_common_pairs=0,stored_common_reused=1,
        native_difference_queries=1),source


def main():
    out=HERE/'reviews/partition.json'
    assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json')
    evidence=n.p.read(HERE/'reviews/evidence.json')
    result,source=query(raw,evidence)
    for f in [Path(__file__),HERE/'reviews/check.json',HERE/'reviews/evidence.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    n.a.save(out,dict(completed=True,result=result,source_sha256=source,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),
        scope='One saved maximum common minus rotated original terminal support; does not replay28groups or classify other contacts.'))


if __name__=='__main__':
    main()

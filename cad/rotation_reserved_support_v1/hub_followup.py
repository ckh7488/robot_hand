"""Test the central-hub location after the thumb-bridge hypothesis was rejected."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm060_hub_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,ROOT,HERE=a.n,a.ROOT,a.HERE


def calculate():
    raw=n.p.read(HERE/'reviews/followup.json')
    source={}
    common=n.c.load(n.p.read(ROOT/raw['scene'])['parts'],source)[0]['shape']
    hub=a.cq.Solid.makeCylinder(38.,3.,a.cq.Vector(0,-162,0),a.cq.Vector(0,1,0)).cut(
        a.cq.Solid.makeCylinder(34.,3.02,a.cq.Vector(0,-162.01,0),a.cq.Vector(0,1,0)))
    outside=common.cut(hub,tol=1e-6)
    assert outside.isValid()
    return dict(completed=True,id='actual_plus90_common__outside_front_hub_annulus',operation='cut',
        volume_mm3=outside.Volume() if outside.Solids() else 0.,solids=len(outside.Solids()),
        hub_axis=[0,1,0],hub_radii_mm=[34.,38.],hub_Y_mm=[-162.,-159.],
        source_sha256=source,native_boolean_operations=1,previous_three_followup_queries_repeated=0,
        thumb_bridge_hypothesis_rejected_preserved=True,installation_or_geometry_changed=False)


def main():
    out=HERE/'reviews/hub_followup.json'
    assert not out.exists()
    result=calculate()
    for f in [Path(__file__),HERE/'reviews/followup.json']:
        result['source_sha256'][f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    result['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in result['source_sha256'].items())
    n.a.save(out,result)
    print(result['id'],result['volume_mm3'],result['solids'],flush=True)


if __name__=='__main__': main()

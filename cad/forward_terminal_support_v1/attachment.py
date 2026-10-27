"""Read the real existing M3 stack, without assuming adapter contact from its bbox."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm057_attach_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,ROOT,HERE=a.n,a.np,a.ROOT,a.HERE
post=n.m.drive.module('_arm057_attach_metadata',HERE/'postprocess.py')


def query(raw):
    _,_,proposed,_,delta,*_=post.metadata(raw)
    names=[a.FRAME,'thumb_ip_actuator_wire_guide_base','thumb_ip_actuator_adapter',
        'thumb_ip_actuator_rack_screw_1','thumb_ip_actuator_rack_screw_2',
        'thumb_ip_actuator_rack_nut_1','thumb_ip_actuator_rack_nut_2']
    source={}
    loaded=n.c.load([z for z in proposed if z['name'] in names],source)
    lookup={z['name']:n.g.original.transform(z['shape'],delta.get(z['name'],np.eye(4))) for z in loaded}
    frame=lookup[a.FRAME]
    distances=[dict(part=name,distance_mm=float(frame.distance(lookup[name]))) for name in names[1:]]
    T=np.array(delta['thumb_ip_actuator_adapter'])
    pads=[]
    for y in [-287.,-261.]:
        sh=a.p.terminal.cyl(3.,-82.,-81.,[-11.,y]).cut(a.p.terminal.cyl(1.7,-82.01,-80.99,[-11.,y]))
        sh=n.g.original.transform(sh,T)
        outside=sh.cut(frame,tol=1e-6)
        pads.append(dict(original_seat_world_mm=(T@np.array([-11.,y,-81.,1.]))[:3],
            retained_seat_outside_new_frame_mm3=outside.Volume() if outside.Solids() else 0.,outside_solids=len(outside.Solids())))
    return dict(distances=distances,pads=pads,native_distance_queries=6,native_pad_difference_queries=2,
        scope='Six actual stack distances and two original-seat ring inclusion queries; no force/preload/thread/tool approval or replay of36groups.'),source


def main():
    out=HERE/'reviews/attachment.json'
    assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json')
    result,source=query(raw)
    for f in [Path(__file__),HERE/'postprocess.py',HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    n.a.save(out,dict(completed=True,result=result,source_sha256=source,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__': main()

"""Save actual remaining overlap solids for position correction, not hand cuts."""
import importlib.util
from pathlib import Path
import sys

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_wire_locations',HERE/'build_screen.py')
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
p=m.p


def main():
    out=HERE/'reviews/overlap_locations.json'
    if out.exists():
        raise FileExistsError('Preserve locations')
    report=p.read(HERE/'reviews/screen.json')
    scene=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    hand={r['name']:r for r in scene['hand']}
    pack={r['name']:r for r in p.read(HERE/'generated/packaging.json')['parts']}
    source={r:m.sha(ROOT/r) for r in ['cad/wiring_layout_v1/localize.py','cad/wiring_layout_v1/reviews/screen.json',
        'cad/wiring_layout_v1/generated/packaging.json']}
    items=[]
    for index,row in enumerate(report['proposed'][0]['native']['overlaps']):
        h=hand[row['hand']]
        q=pack[row['packaging']]
        if h['body']!='palm':
            raise ValueError('Only fixed-palm obstruction assumed here')
        for r in [h,q]:
            source[r['step']]=r['sha256_step']
            if m.sha(ROOT/r['step'])!=r['sha256_step']:
                raise ValueError('Changed geometry')
        common=cq.importers.importStep(str(ROOT/h['step'])).val().intersect(cq.importers.importStep(str(ROOT/q['step'])).val(),tol=1e-6)
        if abs(common.Volume()-row['material_intersection_mm3'])>1e-5:
            raise ValueError('Changed intersection')
        folder=HERE/'generated/overlap_locations'
        folder.mkdir(parents=True,exist_ok=True)
        step=folder/(str(index)+'.step')
        mesh=folder/(str(index)+'.stl')
        cq.exporters.export(common,str(step))
        cq.exporters.export(common,str(mesh),tolerance=.05,angularTolerance=.1)
        items.append(dict(**row,centre_mm=list(common.Center().toTuple()),
            bbox_mm=np.asarray(m.pack._bbox(common)).tolist(),solid_count=len(common.Solids()),
            evidence_step=step.relative_to(ROOT).as_posix(),evidence_mesh=mesh.relative_to(ROOT).as_posix(),
            sha256_step=m.sha(step),sha256_stl=m.sha(mesh)))
    p.write(out,dict(completed=True,source_sha256=source,items=items,
        input_hash_guard=all(m.sha(ROOT/r)==h for r,h in source.items()),
        interpretation='Exact common-volume evidence on unadopted fixed motor placement; not installed parts or a hand cut plan. Fixed palm versus fixed modules cannot be solved by finger target angles.'))


if __name__=='__main__':
    main()

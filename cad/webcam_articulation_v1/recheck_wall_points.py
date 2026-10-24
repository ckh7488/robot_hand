"""Recheck saved minimum-thickness locations from the saved STEP, not a full rescan."""
import hashlib
import json
import numpy as np
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Pnt, gp_Dir, gp_Lin
from manufacturing_audit import load, save, OUT


def main():
    _, _, parts, hashes = load()
    source = OUT / 'walls.json'
    report = json.loads(source.read_text())
    assert report['completed'] and len(report['parts']) == 24
    rows = []
    for part in report['parts']:
        shape = parts[part['name']]
        ray = IntCurvesFace_ShapeIntersector()
        ray.Load(shape.wrapped, 1e-7)
        for sample in part['thinnest_samples'][:3]:
            p = np.array(sample['point_mm'])
            n = np.array(sample['normal_into_material'])
            assert abs(np.linalg.norm(n) - 1) < 1e-8
            origin = p - n * .002
            assert not shape.isInside(origin, tolerance=1e-8)
            ray.Perform(gp_Lin(gp_Pnt(*origin), gp_Dir(*n)), 0, 50)
            hits = []
            for i in range(1, ray.NbPnt() + 1):
                point = ray.Pnt(i)
                t = float((np.array([point.X(), point.Y(), point.Z()]) - origin) @ n)
                if t >= -1e-8:
                    hits.append(t)
            unique = []
            for t in sorted(hits):
                if not unique or t - unique[-1] > 1e-6:
                    unique.append(t)
            assert len(unique) >= 2 and abs(unique[0] - .002) < 1e-4
            thickness = unique[1] - unique[0]
            assert shape.isInside(origin + n * sum(unique[:2]) / 2, tolerance=1e-8)
            assert not shape.isInside(origin + n * (unique[1] + 1e-5), tolerance=1e-8)
            error = abs(thickness - sample['thickness_mm'])
            assert error < 1e-6, (part['name'], error)
            rows.append(dict(name=part['name'], point_mm=p.tolist(), thickness_mm=thickness,
                             reference_mm=sample['thickness_mm'], error_mm=error))
    save('wall_point_recheck.json', dict(completed=True, input_sha256=hashes,
        walls_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        scope='Three saved thinnest sample locations per part; not a full UV rescan',
        checked_locations=len(rows), maximum_error_mm=max(r['error_mm'] for r in rows), rows=rows))
    print('Rechecked', len(rows), 'wall locations; maximum error', max(r['error_mm'] for r in rows), flush=True)


if __name__ == '__main__':
    main()

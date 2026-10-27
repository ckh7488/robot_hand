"""Measure the four actual rack post top faces against the current beam."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import audit as d
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def probe(frames):
    lookup = {z['name']: z for rows in frames.values() for z in rows}
    rack = lookup['Forearm_twenty_axis_module_rack']['shape']
    beam = lookup['Forearm_box_beam']['shape']
    faces = []
    for face in rack.Faces():
        b = face.BoundingBox()
        if face.geomType() == 'PLANE' and max(abs(b.zmin+44.), abs(b.zmax+44.)) < 1e-7:
            faces.append(face)
    assert len(faces) == 4
    result = []
    for face in sorted(faces, key=lambda f: (round(f.Center().x, 6), round(f.Center().y, 6))):
        query = BRepExtrema_DistShapeShape(face.wrapped, beam.wrapped)
        query.SetMultiThread(False)
        query.Perform()
        assert query.IsDone() and query.NbSolution() > 0
        points = [[p.X(), p.Y(), p.Z()] for p in
                  [query.PointOnShape1(1), query.PointOnShape2(1)]]
        result.append(dict(post_face_center_mm=face.Center().toTuple(),
            post_face_area_mm2=face.Area(),
            actual_post_face_to_beam_distance_mm=query.Value(), closest_points_mm=points))
    return result


def main():
    out = d.HERE/'reviews/attachment.json'
    assert not out.exists()
    frames, source = d.inputs()
    for rel in ['cad/arm_packaging_audit_v1/attachment_probe.py',
                'cad/form_drive_v1/actuator_packaging.py']:
        source[rel] = d.n.m.drive.sha(d.ROOT/rel)
    rows = probe(frames)
    d.n.a.save(out, dict(completed=True, source_sha256=source, post_faces=rows,
        support_material_faces_used=True, nominal_post_seat_z_mm=-44.,
        min_post_to_beam_distance_mm=min(z['actual_post_face_to_beam_distance_mm'] for z in rows),
        new_hardware=0, mount_clamps_or_bolts_designed=False,
        supports_already_touch_current_beam=False,
        note='Actual planar6x6 rack post faces, not bore centers or imagined contact points. Distances concern the current beam only; not a validated mounting bridge or a prescribed offset for a replacement.',
        input_hash_guard=all(d.n.m.drive.sha(d.ROOT/f) == h for f, h in source.items())))
    print('actual post face distances', [z['actual_post_face_to_beam_distance_mm'] for z in rows], flush=True)


if __name__ == '__main__':
    main()

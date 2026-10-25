"""Render actual saved fixture meshes and nominal undeformed cord meshes."""
from pathlib import Path
import argparse
import numpy as np
from common import CAD, SOURCE, Inputs, digest, write_json, placed_matrix


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument('--generated', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    repo, generated, out = a.repo.resolve(), a.generated.resolve(), a.output.resolve()
    if out.exists():
        raise FileExistsError('Preserve previous rendering')
    out.mkdir(parents=True)
    inputs = Inputs(repo, generated)
    inputs.bind(__file__)
    inputs.bind(repo / CAD / 'common.py')
    scene = inputs.read(generated / 'scene.json')
    build = inputs.read(generated / 'build_report.json')
    if not build['completed'] or build['errors'] or not build['input_hash_guard']['passed']:
        raise ValueError('Incomplete build')
    for key, sha in build['generated_sha256'].items():
        inputs.bind(generated / key, sha)
    for key, sha in build['source_sha256'].items():
        inputs.bind(repo / key[5:], sha)
    helper = inputs.module('_bench_render_helper', repo / 'cad/manufacturing_v1/render.py')
    legacy_inputs = helper.Inputs(repo)
    profile, renderer = helper.read_profile(repo, legacy_inputs)
    design = inputs.read(repo / SOURCE / 'generated/design.json')
    import mujoco
    cache = {}
    assemblies = []
    for q in (0., -15.):
        parts = []
        for row in scene['parts'] + scene['display_cords'][str(q)]:
            path = inputs.resolve_part(row, mesh=True)
            if path not in cache:
                cache[path] = renderer.read_stl_points(path)
            parts.append(dict(name=row['name'], body='palm', mesh_path=path,
                points=cache[path], color=row['color'][:3],
                neutral_inverse=placed_matrix(row, q) if not row['name'].startswith('display_') else np.eye(4)))
        assemblies.append(dict(parts=parts, design=design, label=f'IP {q:g}deg'))
    zero = dict(angles_deg=np.zeros(20))
    cameras = {label: renderer.view_fit(assemblies, [zero], np.array(normal,dtype=float), 1.)
               for label, normal in [('top',[0.,0.,1.]), ('iso',[.8,-.5,.85])]}
    tiles = []
    for index, view, label in [(0,'top','분리형 1축 시험대 · 위에서'),
                               (0,'iso','부모 링크 받침 · 모터 공통판'),
                               (1,'iso','엄지 끝 -15° · 명목 줄 경로')]:
        picture, _ = renderer.render_one(assemblies[index], np.zeros(20), view, cameras, 850, mujoco)
        tiles.append((label, picture))
    renderer.sheet(tiles,3,850,'저장 CAD · 엄지 끝관절을 검증하기 위한 분리형 시험대',
        '전체 손 배치가 아님 · 줄 예압/중앙 혼 고정/센서와 실제 강도·지연 검증은 남아 있음').save(out/'bench_overview.png')
    for assembly in assemblies:
        assembly['_render_state']['renderer'].close()
    for name, sha in legacy_inputs.hashes.items():
        inputs.bind(repo / name, sha)
    guard = inputs.guard()
    report = dict(completed=guard['passed'], source_sha256=inputs.hashes,
        input_hash_guard=guard, saved_mesh_only=True, detached_coupon=True,
        physical_parts=len(scene['parts']), nominal_display_cord_segments_per_pose=10,
        poses_deg=[0.,-15.], output_sha256={'bench_overview.png':digest(out/'bench_overview.png')})
    write_json(out/'render_report.json',report)
    if not guard['passed']:
        raise ValueError('Render inputs changed')
    print('Saved bench render complete', flush=True)


if __name__ == '__main__':
    main()

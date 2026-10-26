"""Resolve inherited external-bench paths without adding it to the moving hand.

generated/scene.json is the immutable motion-audit template. Its hand paths are
already repository-relative; inherited bench-local paths retain the old base.
This public catalog resolves both explicitly and binds them to saved hashes.
"""
import argparse, copy, hashlib, json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
SOURCE=ROOT/'cad/thumb_ip_drive_v1/generated'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    source=HERE/'generated/scene.json';scene=copy.deepcopy(json.loads(source.read_text(encoding='utf-8')))
    for collection in ['hand','motor_bench']:
        for row in scene[collection]:
            for key,hk in [('step','sha256_step'),('mesh','sha256_stl')]:
                path=((ROOT/row[key]) if row['location']=='repo' else SOURCE/row[key]).resolve()
                if not path.is_relative_to(ROOT) or sha(path)!=row[hk]:raise ValueError('Unresolved/changed asset')
                row[key]=path.relative_to(ROOT).as_posix()
            row['location']='repo'
    scene.update(schema='motion_clearance_v2.public_scene.v1',
        installed_hand_part_count=len(scene['hand']),external_motor_bench_installed_in_hand=False,
        audit_template=dict(path=source.relative_to(ROOT).as_posix(),sha256=sha(source)),
        scope='Only hand[] belongs to the 364-part native assembly and motion audit. motor_bench[] is a separate preserved external reference, with resolved source paths.')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(scene,indent=2),encoding='utf-8')
    print(json.dumps(dict(hand_parts=len(scene['hand']),separate_bench_parts=len(scene['motor_bench']),all_asset_paths_and_hashes_verified=True)))


if __name__=='__main__':main()

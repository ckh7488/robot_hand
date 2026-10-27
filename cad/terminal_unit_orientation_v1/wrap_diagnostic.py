"""Domain of the declared one-turn cord branch; not a permitted motor limit."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def evaluate(routes):
    branches=[]
    for z,side in zip(routes,[-1.,1.]):
        offset=z['anchor_angle_deg']-z['actual_tangent_angle_deg']
        ends=sorted([-offset,360./side-offset])
        branches.append(dict(line=z['line'],signed_wrap_deg_at_command0=side*offset,
            command_branch_open_interval_deg=ends,
            zero_wrap_command_deg=-offset,one_turn_command_deg=360./side-offset))
    common=[max(z['command_branch_open_interval_deg'][0] for z in branches),
            min(z['command_branch_open_interval_deg'][1] for z in branches)]
    return dict(branches=branches,common_open_interval_deg=common,
        branch_is_not_a_new_motor_or_joint_angle_limit=True,
        plus30_command_line0_wrap_deg=0.,
        scope='Only the declared zero-to-one-turn wrap branch. At +30 motor degrees line0 loses this wrap. Does not establish all physical cable paths or permit restricting a joint. Rephasing the spool was not performed.')


def main():
    out=HERE/'reviews/wrap_diagnostic.json'
    assert not out.exists()
    f=HERE/'reviews/check.json'
    raw=json.loads(f.read_text(encoding='utf-8-sig'))
    result=evaluate(raw['local_routes'])
    result.update(completed=True,source_sha256={z.relative_to(ROOT).as_posix():hashlib.sha256(z.read_bytes()).hexdigest() for z in [Path(__file__),f]})
    out.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode())


if __name__=='__main__':
    main()

"""Analytic domain of the explicitly clocked one-turn wrap branch."""
import hashlib
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def evaluate(routes):
    rows=[]
    for z,side in zip(routes,[-1.,1.]):
        offset=z['anchor_installation_phase_deg']-z['tangent_angle_deg']
        rows.append(dict(line=z['line'],wrap_at_command0_deg=side*offset,
            open_branch_command_interval_deg=sorted([-offset,360./side-offset])))
    return dict(lines=rows,common_open_branch_interval_deg=[max(z['open_branch_command_interval_deg'][0] for z in rows),min(z['open_branch_command_interval_deg'][1] for z in rows)],
        is_new_motor_or_joint_limit=False,spool_installation_clocking_canonical_X_deg=-90.,
        scope='Only the declared0<wrap<360 branch,with explicitly clocked physical spool/anchors. Command geometry is not continuous whole-material motion or a motor-to-joint mapping.')


def main():
    out=HERE/'reviews/wrap_diagnostic.json'
    assert not out.exists()
    f=HERE/'reviews/check.json'
    raw=json.loads(f.read_text(encoding='utf-8-sig'))
    result=evaluate(raw['local_routes'])
    result.update(completed=True,source_sha256={z.relative_to(ROOT).as_posix():hashlib.sha256(z.read_bytes()).hexdigest() for z in [Path(__file__),f]})
    out.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode())


if __name__=='__main__': main()

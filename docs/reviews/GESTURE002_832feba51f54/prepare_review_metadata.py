"""Prepare GESTURE002 review prose/metadata after a completed bounded replay.

This does not execute the replay, stage files, commit, push or certify fabrication.
CLI: python prepare_review_metadata.py REVIEW_DIRECTORY [--output NEW_JSON]
Only the caller-chosen new metadata JSON is written; existing files are preserved.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import subprocess

ROOT=Path('D:/robot_hand').resolve()
WORK=Path(__file__).resolve().parent
REVIEWS=(WORK.parent/'reviews').resolve()
GIT='C:/Program Files/Git/cmd/git.exe'
ID='GESTURE002'
LABELS=['archive','rebuild','core_collisions','core_transition_samples','interfaces']
EVIDENCE=['core_collisions.json','core_transition_samples.json','interfaces.json','selected_transition_poses.json']
SELECTION_POLICY='path_quarters_plus_staged_thumb_waypoints_v1'
PATH_FRACTIONS=[0.,.25,.5,.75,1.]
STAGED_PATHS=['open__thumb_up','fist__thumb_up']


def digest(raw):return hashlib.sha256(raw).hexdigest()


def complete_audit(value,label):
    guard=value.get('input_hash_guard',{})
    if value.get('completed') is not True or value.get('errors') or guard.get('passed') is not True or not guard.get('start') or guard.get('start')!=guard.get('end'):
        raise ValueError(label+' is incomplete or has a failed input guard')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('review_directory',type=Path)
    parser.add_argument('--output',type=Path,default=WORK/'review_metadata.json')
    args=parser.parse_args()
    directory=args.review_directory.resolve();output=args.output.resolve()
    if directory==REVIEWS or not directory.is_relative_to(REVIEWS) or not directory.is_dir():
        raise ValueError('Expected an existing individual clean-copy review directory on D:')
    if output.parent!=WORK or output.suffix.lower()!='.json' or os.path.lexists(args.output):
        raise ValueError('Use a new metadata JSON directly inside this task work directory')
    raw={name:(directory/name).read_bytes() for name in ['review.json',*EVIDENCE]}
    review=json.loads(raw['review.json'])
    if review.get('completed') is not True or review.get('reproduction_passed') is not True or review.get('transition_sample_reproduction_passed') is not True or review.get('errors'):
        raise ValueError('Bounded reproduction, including selected transition samples, is not complete/passed')
    sha=review.get('target_sha')
    if not isinstance(sha,str) or not re.fullmatch('[0-9a-f]{40}',sha) or review.get('resolved_commit_sha')!=sha:
        raise ValueError('Expected the exact reproduced commit SHA')
    if review.get('manufacturing_approved') is not False:
        raise ValueError('The replay must explicitly withhold manufacturing approval')
    guard=review.get('source_hash_guard',{})
    if guard.get('passed') is not True or not guard.get('start') or guard.get('start')!=guard.get('end') or review.get('reviewer_source_unchanged') is not True:
        raise ValueError('Replay source/reviewer guard is incomplete')
    runner=(WORK/'review_commit.py').read_bytes()
    if digest(runner)!=review.get('reviewer_sha256'):raise ValueError('Executed runner differs from the source being preserved')
    if [r['label'] for r in review.get('commands',[])]!=LABELS or any(r.get('exit_code')!=0 for r in review['commands']):
        raise ValueError('Expected five completed bounded replay commands')
    geometry=review['geometry_reproduction'];masses=review['mass_tensor_reproduction']
    part_names=[r['name'] for r in geometry]
    if not part_names or len(set(part_names))!=len(part_names) or len(masses)!=len(geometry) or {r['name'] for r in masses}!=set(part_names) or any(r.get('passed') is not True for r in geometry+masses):
        raise ValueError('CAD/mass comparison is incomplete or failed')
    core=review['core_collision_reproduction'];selected=review['transition_sample_reproduction']
    count=review['selected_transition_samples'];selection=review['transition_sample_selection']
    if len(core)!=3 or {r['gesture'] for r in core}!={'open','fist','thumb_up'}:
        raise ValueError('Expected the three endpoint replay comparisons')
    if type(count) is not int or count<=0 or len(selected)!=count or len({tuple(r['angles_hex']) for r in selected})!=count:
        raise ValueError('Selected exact-angle replay coverage differs')
    for row in core+selected:
        if row.get('passed') is not True or row.get('missing_pairs') or row.get('extra_pairs') or row.get('original_operation_errors') or row.get('recomputed_operation_errors'):
            raise ValueError('Collision/error outcomes were not reproduced')
    if (selection.get('selected_transition_samples')!=count or selection.get('full_transition_rerun') is not False or
            selection.get('reverse_paths_executed') is not False or selection.get('original_maximum_joint_step_deg')!=2.5 or
            selection.get('forward_paths')!=['open__fist','open__thumb_up','fist__thumb_up'] or
            selection.get('selection_policy')!=SELECTION_POLICY or selection.get('requested_path_fractions')!=PATH_FRACTIONS or
            selection.get('authored_waypoint_paths')!=STAGED_PATHS or selection.get('authored_waypoint_counts')!={p:4 for p in STAGED_PATHS} or
            selection.get('full_original_arithmetic_reconciled') is not True):
        raise ValueError('Selected replay scope differs from the intended three forward paths')
    if (review.get('interface_summary_reproduced') is not True or review.get('interface_changed_checks') or
            type(review.get('interface_individual_checks_compared')) is not int or review['interface_individual_checks_compared']<=0 or
            review.get('interface_original_summary')!=review.get('interface_recomputed_summary')):
        raise ValueError('Individual fabrication outcomes were not reproduced')
    audits={name:json.loads(raw[name]) for name in EVIDENCE if name!='selected_transition_poses.json'}
    for name,value in audits.items():complete_audit(value,name)
    if set(audits['core_collisions.json']['gesture_results'])!={'open','fist','thumb_up'}:
        raise ValueError('Static evidence does not match the core scope')
    selected_ids={r['sample_id'] for r in selected};selected_audit=audits['core_transition_samples.json']
    document=json.loads(raw['selected_transition_poses.json'])
    if (selected_audit.get('mode')!='endpoints' or selected_audit.get('transitions') or
            set(selected_audit['gesture_results'])!=selected_ids or len(selected_audit['cases'])!=count or
            {g['id'] for g in document['gestures']}!=selected_ids or document.get('transitions') or
            digest(raw['selected_transition_poses.json'])!=review.get('selected_transition_document_sha256')):
        raise ValueError('Selected-sample evidence/document mismatch')
    for key in ('GIT_DIR','GIT_WORK_TREE','GIT_INDEX_FILE'):
        if os.environ.get(key):raise ValueError('Refuse inherited '+key)
    env=os.environ.copy();env.update(GIT_NO_REPLACE_OBJECTS='1',GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never')
    p=subprocess.run([GIT,'rev-parse','HEAD'],cwd=ROOT,env=env,capture_output=True,text=True,
        stdin=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW,shell=False,check=True)
    head=p.stdout.strip()
    if not re.fullmatch('[0-9a-f]{40}',head):raise ValueError('Current HEAD did not resolve')
    manifest_count=review['manifest_files_checked']
    if type(manifest_count) is not int or manifest_count<=0 or review.get('frozen_files_checked')!=222:
        raise ValueError('Manifest/frozen verification scope differs')
    # Count each tested pose/pair occurrence, not distinct part pairs over a path.
    endpoint_hits=sum(r['recomputed_pairs'] for r in core)
    selected_hits=sum(r['recomputed_pairs'] for r in selected)
    if any(type(r['recomputed_pairs']) is not int or r['recomputed_pairs']<0 for r in core+selected):
        raise ValueError('Invalid reproduced collision count')
    part_count=len(geometry);interface_count=review['interface_individual_checks_compared']
    body=f'''# GESTURE002 커밋 후 핵심 재현

대상 커밋: `{sha}`

해당 커밋을 별도의 깨끗한 D: 사본으로 내보내 manifest {manifest_count}개와 동결 증거 222개를 확인했다. 전체 CAD {part_count}개 부품을 입력부터 재생성했고, 부품별 체적·경계·질량·무게중심·관성 비교가 모두 선언된 허용오차 안에 들었다.

보·바위·엄지척의 실제 STEP 겹침 검사를 다시 실행해 원래의 부품쌍과 겹침 부피 판정을 재현했다. 재검사한 세 자세의 보고 기준 초과 겹침은 자세별 합산 {endpoint_hits}건이다. 공칭 체결 검사의 개별 결과 {interface_count}개도 모두 일치했다. 결과의 일치는 남아 있는 기하·제작 실패를 통과로 바꾸는 뜻이 아니다.

보→바위, 보→엄지척, 바위→엄지척의 커밋된 `core_paths.json`과 원래 2.5° 보고서의 **전체 표본**을 연결해 구간·비율·각도 배열의 산술 대응을 대조했다. 새 CAD 재검사는 세 경로 각각의 진행 비율 0·0.25·0.5·0.75·1에 가장 가까운 기존 표본과, 보→엄지척·바위→엄지척의 작성된 경유점 네 개씩을 골랐다. 거리 동률은 앞 표본을 고르고, 기록된 각도를 재계산하거나 반올림하지 않은 채 동일 각도를 정확한 부동소수점 값으로 중복 제거한 **{count}자세**를 새 STEP에서 검사했다. 재검사한 선택 표본의 보고 기준 초과 겹침은 표본별 합산 {selected_hits}건이며, 원래의 부품쌍·부피 판정이 재현됐다. 이는 **대표 전환 표본의 정확한 재현(selected transition samples exact replay)**이다.

이는 앞서 완료한 원래 전체 경로 검사를 축소한 것이 아니라 그 뒤의 커밋 후 대표 재현이다. 주먹 경로의 33경유점 및 전체 2.5° 전환 표본을 CAD에서 모두 다시 계산하지 않았다. 역방향 경로, 나머지 여섯 끝자세, 최소거리, 렌더, 동역학도 이번 깨끗한 사본에서 다시 실행하지 않았다. 입력 소스와 재검사 실행기의 해시는 실행 전후 동일했다. 표본 사이의 연속 간섭, 실물 공차·강도·100ms 지연, 제작 가능성을 승인하지 않는다. 같은 작업의 재현 확인이며 외부 독립 감사가 아니다.

이번 단계의 자세·동작 그림과 검사 범위를 보여준 뒤 결과를 정리한다. 추가 형상·성능 개선이나 실물 동작을 실행한 기록은 아니다.

[상세 재현 결과](review.json) · [핵심 세 자세](core_collisions.json) · [선택 전환 표본 검사](core_transition_samples.json) · [선택한 각도 배열](selected_transition_poses.json) · [공칭 체결](interfaces.json) · [체크포인트](../../../checkpoints/GESTURE002.md)
'''
    metadata=dict(id=ID,expected_head=head,candidate_sha=sha,review_directory=str(directory),review_markdown=body,
        progress_lines=[
            f'- GESTURE002 `{sha[:12]}`의 깨끗한 사본에서 전체 {part_count}부품·질량/관성, 보·바위·엄지척, 공칭 체결 결과를 재현했다.',
            f'- 세 전환 경로의 선택 표본 {count}자세를 정확한 각도로 재검사했다. 전체 2.5° 경로 재실행은 아니다. 기록: docs/reviews/GESTURE002_{sha[:12]}/README.md.',
            '- 자세·동작 그림과 이번 단계 결과를 정리한다. 연속 간섭·실물 제작·100ms 성능은 미승인이다.'],
        evidence_files=EVIDENCE,
        preparation_source_sha256=digest(Path(__file__).read_bytes()),
        prepared_evidence_sha256={name:digest(value) for name,value in raw.items()})
    # Do not prepare a success narrative if the reviewed data changed mid-read.
    for name,value in raw.items():
        if (directory/name).read_bytes()!=value:raise ValueError('Review evidence changed during metadata preparation: '+name)
    if (WORK/'review_commit.py').read_bytes()!=runner:raise ValueError('Review runner changed during metadata preparation')
    with output.open('x',encoding='utf-8') as stream:
        stream.write(json.dumps(metadata,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(metadata=str(output),candidate_sha=sha,head=head,selected_transition_samples=count)),flush=True)


if __name__=='__main__':main()

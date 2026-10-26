"""Save completed posture-only evidence; freeze a new snapshot, not old ones."""
import argparse
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
    import hashlib
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--main-audit',type=Path);ap.add_argument('--object-audit',type=Path);ap.add_argument('--freeze',action='store_true');a=ap.parse_args()
    if a.main_audit:
        main=read(a.main_audit);tip=read(HERE/'reviews/tip_refined.json');obj=read(a.object_audit)
        for report in [main,tip,obj]:
            if not report['completed'] or not report['input_hash_guard']:raise ValueError('Incomplete evidence')
            if not all(sha(ROOT/p)==h for p,h in report['source_sha256'].items()):raise ValueError('Stale source')
        out=HERE/'reviews/posture_initial.json'
        if out.exists():raise FileExistsError('Preserve initial report')
        shutil.copy2(a.main_audit,out)
        combined=dict(main);combined['cases']=main['cases']+tip['cases'];combined['paths']=main['paths']+tip['paths'];combined['selected']=main['selected']+tip['selected']
        combined['source_reports']=['cad/posture_correction_v1/reviews/posture_initial.json','cad/posture_correction_v1/reviews/tip_refined.json']
        combined['source_sha256']={**main['source_sha256'],**tip['source_sha256'],**{p:sha(ROOT/p) for p in combined['source_reports']}}
        combined['elapsed_s']=main['elapsed_s']+tip['elapsed_s'];combined['near_tip_goal_changed_from_palmar_contact']=True
        write(HERE/'reviews/audit.json',combined)
        old=ROOT/'cad/object_contact_v1/reviews';old.mkdir(exist_ok=False);shutil.copy2(a.object_audit,old/'audit.json')
        (ROOT/'cad/object_contact_v1/README.md').write_text('''# 이전 물체 접촉 연구 · 채택 후보 없음

앞 단계의 실패 결과를 보존한다. 엄지와 네 손끝의8mm 구, 엄지/검지의25mm 구라는5목표·48시작점 탐색과 선택15끝자세를 검사했다.12끝자세에서 손364부품과 구의 보고 기준 이상 겹침은 없었으나 실행4접근/풀기 경로가 모두 실패했다. 새끼의3끝자세도 손 부품 간섭이 있었다. 물체 유지나 자연스러운 잡기 성공으로 쓰지 않는다.

관절 끝과 역굽힘에 기대는 후보가 포함되어 사람 손 느낌을 유지하는 채택 자세로 삼지 않았다. STEP 구는 시험 물체이고 손 설치 부품이 아니다. 손364부품의 형상·축·길이를 바꾸지 않았다. 이후 자세 교정은 `../posture_correction_v1/`에서 진행한다. `generated/search.json`과 `reviews/audit.json`에 출처·모든 실패·검사 범위를 보존한다.
''',encoding='utf-8')
        return
    if not a.freeze:raise ValueError('Choose save or freeze')
    manifest_path=ROOT/'checkpoints/POSTURE005.manifest.json'
    if manifest_path.exists():raise FileExistsError('Do not overwrite committed/saved manifest')
    report=read(HERE/'reviews/audit.json');search=read(HERE/'generated/search.json');render=read(HERE/'images/report.json')
    if not render['completed'] or not all(sha(ROOT/p)==h for p,h in render['input_sha256'].items()):raise ValueError('Stale render')
    if not all(sha(HERE/'images'/p)==h for p,h in render['outputs'].items()):raise ValueError('Changed image')
    if not all(sha(ROOT/p)==h for p,h in report['source_sha256'].items()):raise ValueError('Stale native result')
    selected={r['group']:r for r in report['selected']};byid={r['id']:r for r in report['cases']};pathmap={r['id']:r for r in report['paths'] if 'id' in r}
    cross=selected.get('cross_front_middle');tip=selected.get('index_thumb_distal_contact')
    if not cross:raise ValueError('No corrected cross approach/release')
    crosscase=byid[cross['case']];crosspath=pathmap[cross['path']];tipcase=byid[tip['case']] if tip else None;tippath=pathmap[tip['path']] if tip else None
    scene=read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    for r in scene['hand']:
        if sha(ROOT/r['step'])!=r['sha256_step'] or sha(ROOT/r['mesh'])!=r['sha256_stl']:raise ValueError('Hand geometry changed')
    exported=[]
    for s in report['selected']:
        path=pathmap[s['path']];exported.append(dict(id=s['group'],angles_deg=s['angles_deg'],source_case=s['case'],source_path=s['path'],waypoints_deg=path['waypoints_deg']))
    write(HERE/'generated/selected_postures.json',dict(joint_names=read(ROOT/'cad/gesture_hand_v3/poses.json')['joint_names'],
        joint_names_source='cad/gesture_hand_v3/poses.json',postures=exported,cad_reference='cad/motion_clearance_v2/generated/assembly_catalog.json',
        geometry_changed=False,hardware_execution_approved=False,scope='Engineering posture/finite geometric route candidates; tip posture is unloaded near-touch, not a grasp release.'))
    close=next(c for c in crosscase['crossings'] if c['distance_is_local_to_cross'])
    tipline=(f"손끝 모으기는 실제 둥근 끝캡의 앞면을 목표로 바꾸고 검지 안쪽 벌림을 줄여 끝자세 겹침0, 끝캡 간격{tipcase['native_tip_surface_gap_mm']:.3f}mm를 얻었다. 선택한 접근/풀기{tippath['total_approach_release_samples']}표본에서도 보고 기준 이상 겹침0·연산 실패0이다." if tip else '손끝 모으기는 근접 끝자세를 찾았으나 접근/풀기 경로는 미해결로 보존한다.')
    text=f'''# POSTURE005 · CAD를 유지한 자세 교정

사용자 지시: 조금 부딪히는 한 후보 때문에 바로 CAD를 고치지 않는다. 사람다운 손 비율·손모양을 유지하고 자세·관절 협응·접근/풀기 순서로 먼저 해결한다.

## 이번 실제 변경

손364부품의 STEP/STL·축·길이를 그대로 유지했다. 한 손가락을 고정한 교차 대신 두 손가락의 굽힘을 나누고, 손끝 모으기의 과도한 굽힘과 검지 안쪽 벌림을 줄였다. 모터 설치 위치나 CAD 외피를 수정하지 않았다. 이 단계는 실제 모터 구동 시험이 아니다.

![같은 CAD의 자세 교정](images/posture_comparison.png)

- 중지 앞쪽 교차: 이전 첫마디 겹침1쌍을0쌍으로 줄였다. 검지 PIP40°, 중지 MCP28°·PIP5°·벌림18°, 나머지 두 손가락/엄지는 기존 가위 자세를 유지한다. 뒤로 꺾는 PIP/DIP에 기대지 않는다. 교차점의 중심선 앞뒤 차이{abs(close['signed_depth_mm']):.2f}mm, 해당 두 마디의 실제 가까운 표면 간격{close['segment_surface_distance_mm']:.3f}mm를 기록했다. 기존 먼 교차 참조49.93mm에 비해 가까우며, 수치만으로 생체 자연스러움의 표준을 주장하지 않는다.
- 가위→앞뒤 분리→두 손가락 굽힘→교차→정확 역순 풀기: 전진{crosspath['forward_sample_count']}표본, 왕복{crosspath['total_approach_release_samples']}표본, 관절 간격 최대5°에서 보고 기준 이상 겹침0·연산 실패0이다. 같은 전진 배열의 역순이며 별도 동역학 검증이 아니다.
- {tipline}

## 비교에서 지켜야 할 조건

오른쪽 새 목표는 빈손에서 손끝끼리 가깝게 모으는 자세다. 이전 중앙 손바닥면 랜드마크를 강제로 일치시키는 목표와 구별한다. 약0.25mm의 비접촉 근접 자세이며, 실제 접촉·힘·물체 집기/유지 성공으로 쓰지 않는다. 사람다운 모습은 수용한 외형을 보존한 CAD 그림으로 검토했고 생체 동작 전체나 사용자 승인 범위를 주장하지 않는다.

반대 교차 방향의 선택 후보들은 검지 뿌리 외피와 손바닥 앞덮개가 겹쳐 미해결이다. 이것만으로 CAD 수정의 필요가 확정된 것은 아니다. 두 손가락의 벌림/굽힘을 더 나누는 경로를 계속 찾는다. 실패 경로는 첫 실제 실패 지점에서 중단하고 그 표본을 남겼다.

이전 `object_contact_v1` 연구는5물체 목표·15끝자세와4경로를 실제 검사했으나 모든 실행 경로가 실패해 채택하지 않았다. 시험용 구15개는 손 설치 부품이 아니다. 원통·열쇠·필기·판·구 유지와 GRASP33분류는 완료되지 않았다. 작은 공칭 여유 수치를 이유로 CAD를 미세 절삭하는 작업은 하지 않았다.

## 범위와 재현

전체364부품의66066쌍을 검사한다. 보고 기준은 재료 겹침0.02mm³ 초과·Boolean 허용오차1e−6mm다. 모든 작은 겹침이나 표본 사이의 연속 충돌이 없다는 증명은 아니다. 실제 공차·강도·줄/모터·실물 성능은 미검증이다.

`generated/search.json`은 양방향 교차 각864조합의 유한 메시 가이드와16시작점 손끝 탐색이다. `generated/root_posture_map.json`은 실제 앞덮개/검지 첫마디99조합의2부품 지도이며 전체 손 검사로 세지 않는다. `generated/tip_refined.json`은 그 관측 간섭을 이용한16시작점 재탐색이다. 캡슐/보간/메시는 후보 선택용이고 최종 native 검사에서 부품쌍을 제외하지 않았다. 임의 각도 제한으로 원래 실패를 지우지 않았다.

원래 결과는 `reviews/posture_initial.json`, 재교정은 `reviews/tip_refined.json`, 묶음은 `reviews/audit.json`, 사용 후보 각도·경로는 `generated/selected_postures.json`에 있다. CAD는 [같은364부품](../motion_clearance_v2/generated/assembly_catalog.json)과 [기존 전체 STEP](../motion_clearance_v2/generated/hand.step.zip)을 참조한다. [교차 동작](images/cross_motion.gif).

커밋·푸시 뒤 정확 SHA의 새 깨끗한 clone에서 성공/실패 핵심 자세·경로점·표면 간격·동결222개를 다시 검사하고 후속 커밋에 범위를 기록한다. 같은 에이전트 재검토이며 독립 감사나 실물 승인이 아니다.
'''
    (HERE/'README.md').write_text(text,encoding='utf-8')
    (ROOT/'checkpoints/POSTURE005.md').write_text(f'# POSTURE005\n\n손364부품과 축·길이 유지. 중지 앞쪽 교차1→0쌍과 왕복{crosspath["total_approach_release_samples"]}표본을 자세·순서만으로 교정. 손끝 모으기 결과·미해결 반대 교차·물체 파지는 [실제 범위](../cad/posture_correction_v1/README.md)에 기록. 전체 가동·제작·실물 승인은 없음. 정확SHA 핵심 재검토는 후속 커밋.\n',encoding='utf-8')
    cp=read(ROOT/'CHECKPOINT.json');cp.update(id='POSTURE005',candidate='cad/posture_correction_v1',manifest='checkpoints/POSTURE005.manifest.json',previous_checkpoint='FUNCTION003',
        status='Same364part CAD; posture/coordination correction first. Corrected crossing route and recorded unloaded near-tip findings; opposite crossing and real object grasps unresolved. No physical approval.',
        baseline='cad/motion_clearance_v2',post_commit_results_reproduced=False,post_commit_review_target=None,post_commit_review_report=None,post_commit_reproduction_scope=None)
    write(ROOT/'CHECKPOINT.json',cp)
    with (ROOT/'PROGRESS.md').open('a',encoding='utf-8') as f:
        f.write(f'\n- POSTURE005 수정: CAD364부품·축·길이를 유지하고 두 손가락 교차 협응과 손끝 모으기 자세/순서를 교정.\n- 검사: 중지 앞쪽 교차1→0쌍, 접근/풀기{crosspath["total_approach_release_samples"]}표본 겹침0. 손끝 모으기 선택 경로는 상세 결과에 기록. 커밋 뒤 재검토는 별도 수행.\n- 남은 일: 반대 교차 방향·실제 접촉/물체 파지·실물 구동. 작은 간섭 후보만으로 CAD를 고치지 않고 자연스러운 자세와 경로를 먼저 조정.\n')
    root=ROOT/'README.md';raw=root.read_bytes().decode('utf-8');start=raw.index('현재는 **손 전체의');end=raw.index('엄지의 [두 줄',start)
    new=f'''현재는 **CAD를 유지하면서 사람이 자연스럽게 할 만한 자세와 움직이는 순서를 먼저 교정하는 단계**입니다. 최신 [POSTURE005](cad/posture_correction_v1/README.md)는364부품·축·길이를 보존하고 중지 앞쪽 교차의1쌍 간섭을0으로 줄였으며 접근·풀기{crosspath['total_approach_release_samples']}표본에서 보고 기준 이상 겹침0을 확인했습니다. 손끝 모으기는 빈손 근접 자세로 구분하고 실제 물체 집기 성공으로 쓰지 않습니다.

**한 후보가 부딪힌다는 이유로 바로 CAD를 고치지 않습니다.** 자연스러운 각도·협응·접근/풀기로 먼저 해결하고, 필요한 동작이 기계적으로 막히는 근거가 있을 때 형상 수정을 판단합니다. 반대 교차 방향과 실제 물체 잡기는 남아 있습니다. [그림과 동작](cad/posture_correction_v1/README.md) · [이전 전체 기능 검사](cad/functional_motion_v3/README.md)

'''
    root.write_bytes((raw[:start]+new+raw[end:]).encode('utf-8'))
    paths=set(read(ROOT/'checkpoints/FUNCTION003.manifest.json')['files']);frozen=read(ROOT/'AUDIT_EVIDENCE_INDEX.json')
    for key in ['geometry_sha256','code_sha256','report_sha256']:paths.update(frozen[key])
    for folder in [HERE,ROOT/'cad/object_contact_v1',ROOT/'docs/reviews/FUNCTION003',ROOT/'docs/reviews/CLARIFY004',ROOT/'docs/explanations/collision_locations_20261026']:
        paths.update(p.relative_to(ROOT).as_posix() for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    paths.update(['README.md','AGENTS.md','checkpoints/POSTURE005.md','checkpoints/FUNCTION003.manifest.json'])
    paths.difference_update(['CHECKPOINT.json','PROGRESS.md','checkpoints/POSTURE005.manifest.json'])
    write(manifest_path,dict(checkpoint='POSTURE005',geometry_changed=False,baseline_assembly='cad/motion_clearance_v2/generated/assembly_catalog.json',
        exclusions=['self','CHECKPOINT.json','PROGRESS.md'],files={p:dict(sha256=sha(ROOT/p),bytes=(ROOT/p).stat().st_size) for p in sorted(paths)}))
    print(json.dumps(dict(completed=True,manifest_files=len(paths),selected=report['selected'])),flush=True)


if __name__=='__main__':main()

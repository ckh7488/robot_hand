"""Freeze guide candidates and keep axial restraint and assembly limits visible."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def main():
    manifest=ROOT/'checkpoints/WIRE011.manifest.json'
    if manifest.exists():raise FileExistsError('Preserve prior snapshots')
    check=read(HERE/'reviews/check.json');service=read(HERE/'reviews/service.json');order=read(HERE/'reviews/assembly_order_v2.json');images=read(HERE/'images/report.json')
    for report in [check,service,order]:
        if not report['completed'] or not report['input_hash_guard'] or not all(sha(ROOT/r)==h for r,h in report['source_sha256'].items()):
            raise ValueError('Incomplete/changed evidence')
    if not all(check[key] for key in ['final_nominal_clear','all_mounts_nominal_seated','all_wires_nominal_supported']) or not order['conditional_sampled_order_clear']:
        raise ValueError('Resolve required nominal geometry/service paths before freezing')
    if not images['completed'] or not all(sha(ROOT/r)==h for r,h in images['input_sha256'].items()) or not all(sha(HERE/'images'/r)==h for r,h in images['outputs'].items()):
        raise ValueError('Changed render')
    blocked=sum(len(r['native']['overlaps']) for r in service['full_population_base_attempt'])
    (HERE/'README.md').write_text(f'''# WIRE011 · 받침에 체결하는 분리형 전선 가이드

기존 볼트 자리를 사용하는 가이드16개로 전선48개의 위치를 잡는 후보를 만들었다. 손364부품·관절축·길이·모터 위치·랙과48전선 경로는 바꾸지 않았다. **옆으로 빠지는 것을 막는 가이드이며, 축 방향 당김을 막는 스트레인 릴리프는 아직 미검증이다.** 전선을 꽉 압착했다고 쓰지 않는다.

![가이드 추가 전후](images/guides.png)

초록은 새 가이드와 덮개다. 그림은16개 중 모터1개의 국부 보기이며, 랙은 그림에서만 잘랐다. 실제 검사에서는 전체 랙·포장 부품을 썼다. 기존 모터 체결면 아래에서 연결된 받침으로 가이드를 지지하고 기존 두 M3 축을 사용한다. 단순 떠 있는 결속구나 끝점을 고정 완료로 세지 않았다.

## 실제 구성과 받침

모터마다 연결된 가이드 하부1개·분리형 덮개1개·M2 나사/너트2쌍을 추가했다. 기존32개 M3×8 외곽 대신 M3×12 외곽과1mm 스페이서32개를 별도 후보로 모델링했다. 새 부품160개와 기존193개 참조로 포장 후보353개이며, 전선48개/커넥터 외곽16개와 손364부품은 별도다. 이전225부품 후보나 모터 설치 수에 새 가이드를 모터로 더하지 않는다.

M3 머리는 기존 일반 외곽을 승계한 것이며 실제 공급 나사 품번·머리/드라이브·나사산·토크/축력 검증이 아니다. 스페이서는 별도1mm 링 후보, M2×6/너트는 비나선 외곽이다. M3 축 끝은 Z−70mm, 기존 너트는 Z−73~−70.6mm로 명목상0.6mm 돌출한다. 이 값을 실제 완전산/체결 강도 승인으로 쓰지 않는다.

가이드 발의 윗면은 기존 어댑터 아랫면 Z−79mm에 닿는다.16개 모두 BRep 거리0을 확인했고 각 두 볼트 주변의12위치에서, 그 면 아래는 가이드 재료·바로 위는 어댑터 재료인지 원본 솔리드의 안팎 분류로 확인했다. 전선48개도 명목 외경1.9mm의 바닥점이 홈 바닥 Z−69.75mm에 닿는 거리0을 확인했다. 단순 경계상자 접촉만을 지지 근거로 쓰지 않았다. 그루브 폭7.5mm·높이2.4mm와 덮개가 횡방향 이탈을 억제하지만 축 방향 미끄럼·마찰·당김 하중·피로·절연체 마모·인쇄 강도/공차는 미검증이다.

## 간섭과 실제 조립 순서

새160부품 대 기존193부품30880쌍, 새 부품끼리12720쌍, 새 부품 대48전선7680쌍/16커넥터 외곽2560쌍에서 보고 기준 이상 재료 겹침0·연산 실패0이다.5손끝자세 각각 손364부품 전체와 새 부품을 대조했고 중립 팔 참고10부품과도 같은 결과다. AABB 후 native Boolean, 보고 문턱0.001mm³ 초과·허용오차1e−6mm다. 기존 부품끼리의 미해결 손바닥8쌍은 이 검사에 포함해 없어진 것으로 세지 않았다.

조립 순서가 필요하다. 좌우 모터를 모두 채운 뒤 가이드16개를 왼쪽에서 한꺼번에 넣는 시도는5표본 합계{blocked}겹침 기록으로 막혔다. 좌측 모터를 뺀 뒤 우측 가이드를 X−20mm부터 수평 이동하는 시도도6쌍 기록으로 랙에 걸렸다. 기존 모터 하부 M2 나사/너트가 설치된 상태에서 M3를 축 방향으로 넣는 시도도128쌍의 표본 겹침 기록으로 막혔다. 서로 다른 위치에서 같은 부품이 반복된 기록이며 그 수만큼 독립 결함이 있다는 뜻이 아니다. 원본 `service.json`의 required_sampled_paths_clear는 false로 보존한다. 최종 위치가 맞는다는 이유로 조립 가능하다고 승계하지 않는다.

대안은 우측 가이드를 X−15/Y+8mm 옮긴 위치에서20mm 아래로 두고 올린 뒤, Y를0으로 옮기고 X를0으로 넣는 세 구간이다. 첫 Y+10mm 대안은 첫 줄 가이드가 랙 가로대와25.037009mm³ 겹쳐 따로 보존했다. Y+8mm 진입은 좌측 모터가 없는 조건에서 각 구간5표본, 합계15행에서 보고 기준 이상 겹침0을 확인했다. 좌측 가이드의 원래5표본 삽입은 통과했다. M3는 모터/크래들의 하부 M2 나사·너트를 아직 달지 않은 단계에 먼저 넣어5표본을 확인했다. 나중에 M2 나사/너트를 각각 ±X8mm에서 넣는5표본씩은 **새 가이드/체결품과의 대조만** 통과했다. 원래 모터 내부/자체 체결·공구 경로까지 통과한 것이 아니다. 모듈과 부품을 잡을 조립 지그도 아직 모델링/검증하지 않았다. 이 조건부 순서는 전체 모터 설치/조립 승인과 구별한다. `assembly_order.json`과 `assembly_order_v2.json`에 실패와 후속을 나누어 남긴다.

하부 M2 체결품이 없는 조건의 M3 삽입은 Z−12/−9/−6/−3/0mm, 덮개 나사 분리는 Z0/2/4/6/8mm, 나사 제거 후16덮개의 들기는 Z0/3/6/9/12mm에서 확인했다. 덮개 두 경로는 최종 고정 가이드·전선·포장 부품·중립 팔 대조에서 보고 기준 이상 겹침0이다. 원본 서비스30행과 대안 순서30행은 별도다. 이산 표본이고 실제 공구/손가락 접근·선 교체의 유연 변형·연속 충돌·하중·전체 분해 순서 승인과 구별한다. 하부 가이드 교체에는 모터 하부 체결품 분해와 위 조립 순서가 필요하며, 덮개만 열어 전선을 다루는 작업과 다르다.

![덮개를 분리한 모습](images/lid_access.png)

## 남은 범위와 다음

전선 외경1.9mm·반경10mm·직선5mm는 기존 기하 모델의 가정이다. 실제 공급 케이블/단자와 굽힘 수명은 미확정이며 선 색은 핀 순서가 아니다. 커넥터16개는 중공 하우징이 아닌 외곽 상자이고 모터 전체/헤더와 약39.270093mm³ 겹치는 이전 미해결 결합을 보존한다. ROBOTIS21AWG/JST 단자 최대22AWG 차이를 이 모델로 인증하지 않는다. 출처는 [기존 제조사 자료](../wiring_layout_v1/references_current.json)에 보존했다.

다음은 조건부 순서에 필요한 모터 하부 체결/모듈 설치 경로와 조립 지지 계획, 실제 공급 케이블/커넥터 근거·축 방향 당김 방지·연결 버스/보드용 임시 종단 공간이다. 가이드의 작은 틈 최적화로 돌아가지 않는다. WIRE008의 손바닥 고정 모터8쌍은 여전히 미해결이고 전체20모터 배치/장착·혼/스풀·구동 텐던·팔/손목 연속 케이블·보드·전원/전류·50/100ms 실물 성능은 미완이다. 현재 전선 끝은 연결되지 않은 공간 후보다.

코드·입력/구성·160새 부품 STEP/STL·기존 참조·검사·실패 조립 순서·조건부 대안 순서·전후 그림을 커밋/정상 푸시한다. 정확SHA의 새 깨끗한 clone에서 새160형상을 재생성/재수입해 해석적 면/경계/꼭짓점/속성을 대조하고 주요 재료·받침·전선 접촉·5손끝자세·중립 팔과 원본/대안의 핵심 서비스 표본을 재현한다. 원본/대안60행 전부와 그림 렌더를 다시 수행한다고 쓰지 않는다. 같은 에이전트 검토이며 독립 감사가 아니다. 이전 WIRE009의 일치 경계면 Boolean 판정 불가는 보존하고 이를 손 CAD 결함이나 전체 core 통과로 바꾸지 않는다.
''',encoding='utf-8')
    (ROOT/'checkpoints/WIRE011.md').write_text('# WIRE011\n\n기존 두 M3 자리로 지지하는 분리형 전선 가이드16개,48전선 명목 받침 후보. 손/모터/랙/선 경로 유지. 새160+기존193포장 참조. 최종 재료/받침/표본 분리·선조립 순서 확인. [그림·수치·미완 범위](../cad/wire_guides_v1/README.md). 축 방향 당김 방지·공급 나사/케이블·전체 조립·모터 설치는 미검증. 정확SHA 재현은 후속 기록.\n',encoding='utf-8')
    cp=read(ROOT/'CHECKPOINT.json')
    if cp['id']!='WIRE010':raise ValueError('Checkpoint superseded')
    cp.update(id='WIRE011',previous_checkpoint='WIRE010',candidate='cad/wire_guides_v1',manifest='checkpoints/WIRE011.manifest.json',
        status='16removable transverse wire guide candidates,160new parts plus193retained packaging.16bearing seats/48wire contacts and final material checks clear. Original M3/base insertion paths blocked;conditional alternate order sampled, later factory M2 paths compared to NEW parts only, assembly fixtures/full module paths unverified. Hand/motor/rack/wires preserved. No axial strain relief/full assembly/harness/20motor installation approval;8fixed palm clashes remain.',
        nominal_guide_mounts_seated=True,axial_strain_relief_validated=False,supplier_fasteners_selected=False,candidate_adopted=False,
        next_stage='Verify complete module/base hardware insertion and assembly support for the conditional guide order; verify supplied cable/connector, axial strain relief and bus/provisional board termination; retain fixed palm8issue. Close guide micro-optimization and solved hand motion.',
        post_commit_results_reproduced=False,post_commit_review_target=None,post_commit_review_report=None,post_commit_reproduction_scope=None)
    write(ROOT/'CHECKPOINT.json',cp)
    with (ROOT/'PROGRESS.md').open('a',encoding='utf-8') as f:
        f.write('\n- WIRE011 작업: 기존 볼트 자리로 전선 가이드16개/분리 덮개 후보 추가. 손/모터/랙/48전선 경로 보존, 축 방향 당김 방지는 미검증.\n- 검사: 새 부품 재료 겹침0·받침16/전선48접촉·분리/선조립 표본 확인. 모터를 다 채운 뒤 삽입은 막혀 순서 필요, 전체 설치/실물 승인 아님.\n- 저장/다음: 정확SHA 새 clone 핵심 재현은 후속 수행. 공급 케이블·당김 방지·연결 버스/보드 임시 공간과 남은 손바닥8쌍을 진행.\n')
    rp=ROOT/'README.md';raw=rp.read_bytes().decode('utf-8');start=raw.index('**배선 준비 현황:**');end=raw.index('\n\n',start)
    raw=raw[:start]+'**배선 준비 현황:** [WIRE011](cad/wire_guides_v1/README.md)에 기존 볼트 자리를 이용하는 분리형 가이드16개를 넣어48전선의 명목 받침과 분리/선조립 표본을 확인했습니다. 손/모터/랙/전선 경로는 유지했습니다. 축 방향 당김 방지·공급 케이블/체결품·연결 버스/보드·전체 모터 설치는 미완이고 손바닥 고정8쌍도 남아 있습니다.'+raw[end:];rp.write_bytes(raw.encode('utf-8'))
    paths=set(read(ROOT/'checkpoints/WIRE010.manifest.json')['files']);paths.update(['checkpoints/WIRE010.manifest.json','checkpoints/WIRE011.md'])
    for report in [check,service,order]:paths.update(report['source_sha256'])
    paths.update(images['input_sha256'])
    for folder in [HERE,ROOT/'docs/reviews/WIRE010']:
        paths.update(f.relative_to(ROOT).as_posix() for f in folder.rglob('*') if f.is_file() and '__pycache__' not in f.parts)
    paths.difference_update(['CHECKPOINT.json','PROGRESS.md','checkpoints/WIRE011.manifest.json'])
    write(manifest,dict(checkpoint='WIRE011',whole_design_adopted=False,axial_strain_relief_validated=False,
        exclusions=['self','CHECKPOINT.json','PROGRESS.md'],files={r:dict(sha256=sha(ROOT/r),bytes=(ROOT/r).stat().st_size) for r in sorted(paths)}))
    print(json.dumps(dict(completed=True,manifest_files=len(paths),new_parts=160,retained_packaging=193,guides=16,full_population_interference_records=blocked)))


if __name__=='__main__':main()

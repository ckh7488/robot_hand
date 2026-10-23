# CP002 준비 상태 — PC 연결 중단으로 CAD 커밋 보류

2026-10-24 KST. **완료된 CP002가 아닙니다. 제작/구동 승인본도 아닙니다.**

## 저장소에 반영된 것

- CP001 기준: c738a40ec8b56985a8c6785470f66c14b709fcc1
- CP001의 깨끗한 clone 재검토: 1a8a9b5fd1b414373982642bf41d6368047d91dd
- 재검토 보고서: docs/reviews/CP001_c738a40.md
- V9 동결 해시 222개, manifest 816개, 6자세 FK, 123개 기하 배선 곡선, 엄지 실제 STEP 공구 통로 검사 재현.
- CP001 재검토는 같은 에이전트의 새 사본 검사이며 독립 감사가 아닙니다.

## PC에서 수정·확인했으나 아직 업로드하지 못한 것

로컬 저장소: E:/hand_recog/robot_hand

후보 경로: cad/assembly_fixes_v13/

- V13_hand_packaging.FCStd: V12에서 덮개와 덮개 나사 두 객체만 바꾼 손 조립 후보.
- parts/Palm_front_cover_recessed.step 및 STL
- parts/M3x8_button_cover_screws.step 및 STL
- build_cover_fix.py, check_cover_motion.py, render_cover_fix.py
- reviews/build_geometry.json, cover_motion.json, cover_brep.json, cover_fix_preview.png
- safe_joint_limits.json, tools/offline_pose_guard.py, tools/test_offline_pose_guard.py
- tools/review_cover_checkpoint.py, tools/write_checkpoint_manifest.py
- README.md, .gitignore, tools/check_checkpoint.py의 후속 수정

PC 연결이 끊겨 이 파일들의 커밋·푸시와 커밋 후 새 사본 검토는 끝내지 못했습니다. 따라서 GitHub에 현재 V13 CAD가 있는 것으로 해석하면 안 됩니다.

## 연결 중단 전에 실제 실행 결과에서 확인한 수치

### 덮개 나사 변경

5개 모두 M3×8 ISO 7380-1 낮은 머리 후보로 바꾸고 Ø6.3mm 머리 자리를 1.95mm 가공했습니다.

- 머리 외경 최대 5.7mm, 높이 최대 1.65mm: [Accu 공식 치수](https://www.accu.co.uk/socket-button-screws/8102-SSB-M3-8-A2).
- 머리 상면은 덮개보다 0.30mm 안쪽.
- 남는 완전한 받침 고리 두께 2.05mm.
- 너트 2.4mm 전체 물림, 나사 끝 1.45mm 돌출.
- 덮개에서 약 215.40919mm³ 제거. 새 재료 추가 0, 덮개–나사 겹침 0.
- 새 덮개는 유효 솔리드 한 개이며 닫힌 단일 메시.
- 나머지 250개 조립 객체의 BRep/배치 비교 통과. 최대 수치 직렬화 차이 1.11023e-16.
- 나사 머리는 보수적인 원통 외곽으로 모델링했습니다. 실제 둥근 머리·소켓 형상, 체결 토크, FDM 크리프 검증은 아닙니다.

### 감사의 깊은 주먹 접촉 재현

받은 e2_range.py의 실제 실행 경로는 네 손가락 [벌림,MCP,PIP,DIP]=[0,90t,100t,80t]°, 엄지는 Open 고정입니다. 코드 주석의 엄지 Close와 실제 실행이 다릅니다. t는 시간 아닌 경로 매개변수입니다.

저장 자세 Close는 60/45/30°이므로 같은 경로가 아닙니다.

t=.99에서 실제 STEP 재계산:

| 항목 | 결과 |
| --- | --- |
| 기존 나사–소지 끝 겹침 | 11.92769898mm³ |
| 수정 나사–소지 끝 겹침 | 0 |
| 수정 나사–소지 끝 간격 | 2.66059544mm |
| 수정 덮개–소지 끝 간격 | 1.74286820mm |

수정 부위에 해당하는 움직이는 부품쌍 390개를 대상으로:
- 저장 자세 6개: 메시 최소 4.142638mm.
- Open→저장 Close 401표본: 최소 4.486190mm.
- 감사의 깊은 주먹 401표본: t=1에서 덮개–소지 끝 0.664887mm, 나사–소지 끝 0.964887mm.
- 깊은 주먹 표본 2개가 메시 1.1mm 기준 미달입니다. **전체 깊은 주먹 경로는 통과가 아닙니다.**

이 390쌍 검사는 추가 V10/V11 전선·가이드를 포함하지 않습니다. 모든 부품쌍이나 임의 관절 공간 검사로 확대 해석하지 마세요.

### 벌림 제한 코드

부분 오프라인 범위 검사 15건 통과를 확인했습니다.
- V9의 네 벌림 관절 동시 오차 상자: Open ±1°, Close ±3°, Splay ±0.5°.
- 명령 편차와 오차 구간이 함께 근거 상자에 들어가야 합니다.
- 다른 굽힘·엄지·팔 좌표는 근거 자세에 고정하며, 그 관절의 비영 오차는 거부합니다.
- Backbend, V13 등 다른 형상 버전, 오래된 증거 해시, 근거 없이 넓힌 허용 범위, NaN/누락 입력은 거부합니다.
- 모든 결과의 physical_actuation_approved와 continuous_path_approved는 false입니다.

전체 관절 제한이나 실제 구동 코드가 완성된 것이 아닙니다.

## 미확인 실행 결과

마지막에 특정 경로 t∈[0,.99]의 표본 사이 거리 하한을 추가한 스크립트를 실행했습니다.
- 마지막 로그: E:/hand_recog/tmp/V13_current/v13_cover_motion_final.log
- 출력을 읽기 전에 연결이 끊겼습니다.
- 앞선 실행은 numpy.bool 직렬화 오류로 결과 저장에 실패했고, 그 오류를 고쳐 재실행한 상태였습니다.
- partial_path_continuous_bound의 존재·수치·계산식 검토를 다시 확인해야 합니다. 이 문서는 연속 하한 통과를 주장하지 않습니다.

## 연결 복구 후 이어갈 순서

1. AGENTS.md와 이 문서부터 읽고 git status/log 및 로컬 미커밋 파일을 보존합니다. 원본 V9나 로컬 변경을 초기화하지 않습니다.
2. 원격에 이 중단 기록 커밋이 추가되어 있습니다. git fetch 후 확인하고 fast-forward로 합칩니다. 로컬 변경과 충돌하면 해당 파일을 별도 보존하고 해결합니다. force-push하지 않습니다.
3. 위 로그와 V13 결과를 읽고 동결 V9 해시 222개를 다시 확인합니다.
4. CP002.md를 이 문서의 확인된 결과와 최종 실행 결과로 작성하고, README의 로컬 CP002 링크를 완성합니다. CHECKPOINT.json은 아직 CP001 상태일 수 있으니 확인합니다.
5. CP002 manifest를 생성하고 주요 검사를 실행한 뒤 코드·CAD·이미지·결과·미해결 항목을 함께 커밋·푸시합니다.
6. 그 SHA의 별도 깨끗한 사본에서 형상/접촉/해시/제한 검사들을 재실행하고 대상 SHA가 명시된 후속 검토 커밋을 남깁니다.
7. 다음 수정은 깊은 주먹 끝의 0.665mm 여유, 열린 너트 포켓 낙하 방지, 전체 하네스, 팔 구동 결정입니다. 이번에 너트 낙하 문제는 해결하지 않았습니다.

주의: CP001.manifest.json은 c738a40의 역사적인 전체 파일 스냅샷입니다. 이후 검토 도구를 바꾼 HEAD에서 이 manifest로 전체 파일 동일성을 검사하면 변경된 검토 도구가 잡힙니다. 이를 V9 증거 변경으로 오해하거나 CP001 manifest를 덮어써 감추지 마세요. CP002를 완성할 때 새 manifest를 만들어 현재 후보를 검사합니다.

FreeCAD는 pythonw.exe + 숨김 프로세스로 실행합니다. FreeCADCmd나 새 GUI/콘솔 창을 띄우지 않습니다.

# 설계 전환점

## 손의 비율을 유지하며 움직이는 질량을 줄이기

손바닥을 크게 만들거나 엄지 밖으로 모터를 빼는 후보는 외형 기준을 충족하지 못했습니다. 수용한 외형을 기준으로 삼고, 무거운 구동부의 위치와 줄 전달 구조를 검토하는 방향으로 바꿨습니다. 20축 독립 제어는 기본 비교안입니다. [외형 기준](cad/accepted_form_v1/README.md) · [개발 방향](docs/REMOTE_DRIVE_PLAN.md)

## 형상을 깎기 전에 자세와 조립 순서를 검사하기

일부 손가락 교차와 손끝 접근 문제는 관절 자세와 이동 순서를 바꿔 풀었습니다. 25 mm 고정 구 접근·풀기는 시험 경로의 CAD 확인이며, 실제 파지 시험이 남아 있습니다. 제작 검토에서는 전체 손 조립 후 일부 핀과 공구가 들어갈 공간이 막혀 손가락 모듈 선조립이 필요했습니다. [자세 수정](cad/posture_correction_v1/README.md) · [손끝 접근](cad/object_pinch_v2/README.md) · [모듈 조립 검사](cad/manufacturing_v2/README.md)

## 손과 팔의 통합

실제 부품 기준으로 모터에서 손가락까지 힘이 전달되는 경로, 줄과 전선의 고정, 손목·전완·팔꿈치의 운동 공간을 검토합니다. ARM080은 개별 조립 경로와 막힌 조건을 기록한 CAD 검사입니다. 팔의 전체 조립·구동과 실물 성능은 통합 검증이 필요합니다. [손쪽 통합 상태](docs/HAND_INTEGRATION_STATUS.md) · [ARM080 검사](cad/dogleg_electrical_entry_v1/README.md)

회차별 작업·실패·재현 이력은 [문서 정리 전 Git 스냅샷](https://github.com/ckh7488/robot_hand/commit/14e535f63b6973f79b42cf282e2b57e9da53d48d)의 `PROGRESS.md`와 각 검사 파일에 있습니다.

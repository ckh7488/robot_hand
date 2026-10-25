# GESTURE002 — 곧게 편 엄지척과 세 정방향 전환

같은349개 부품 형상에서 엄지 목표를 [-28,0,0,0]°, 검지를 [8,90,109.97777233225946,90]°로 바꿨다. 중지·약지·새끼는 원래 주먹각이다. 이전 [-10,20,40,0]°의 마디 굽힘을 사진처럼 곧게 편 모습으로 바꾸기 위한 자세 수정이며 길이·관절축을 바꾸지 않았다. 원본 FORM002와 [GESTURE001](https://github.com/ckh7488/robot_hand/tree/73cbc99cde3b2aa94034d211ddb17a9b445858f2) 기록은 보존한다.

349개 STL의 바이트 해시와 관절 정의가 GESTURE001과 같고, 부품 체적·경계·질량·COM·관성 비교도 선언 허용치 안이었다. [형상 비교](../cad/gesture_hand_v3/reviews/geometry_identity.json)는 JSON 수치·STL 비교이며 STEP 위상 동등성 증명이 아니다.

## 실제 검사 범위

새 엄지척을 포함한9개 끝자세에서 보고 기준 이상의 재료 겹침0건·Boolean 연산 실패0건을 확인했다. 세 정방향 경로는 각 구간을 관절별 최대2.5° 이하로 나누어 총228개의 경로 표본을 검사했다. 별도의 세 끝자세 관측을 합친 보고서 관측 수는 231개다. 중복 자세는 같은 정확 각도에서 CAD 결과를 재사용하며 아래 표본 수와 고유 자세 수를 혼동하지 않는다.

| 정방향 경로 | 경로 표본 수 | 겹침 표본 | 연산 실패 | 실제 최대 관절 간격(°) |
| --- | ---: | ---: | ---: | ---: |
| 보 → 바위 | 74 | 0 | 0 | 2.479134425 |
| 보 → 엄지척 | 91 | 0 | 0 | 2.500000000 |
| 바위 → 엄지척 | 63 | 0 | 0 | 2.500000000 |

재료 교집합0.02mm³ 초과를 보고하며 Boolean 공차는0.000001mm다. 같은 강체·다른 강체를 포함한349개 저장 STEP 부품쌍을 대상에 넣었다. [끝자세](../cad/gesture_hand_v3/reviews/gesture_endpoints.json) · [세 경로](../cad/gesture_hand_v3/reviews/core_transitions.json) · [공칭 체결](../cad/gesture_hand_v3/reviews/fabrication_interfaces.json)

현재 엄지척의 선택된8개 부품쌍 간 최소 거리는 2.130950mm다. 0.5mm는 이 국부 비교의 참고값이며 전체 손의 최소 간극이나 제조 공차 기준이 아니다. 검지 MCP·DIP가 명목 상한90°에 있으므로 실물 가동 범위·조립 공차·하중 시 여유는 미확정이다. [국부 거리](../cad/gesture_hand_v3/reviews/thumb_up_clearance.json)

## 그림과 경로

[이전/현재 엄지척](../cad/gesture_hand_v3/generated/motion_review/thumb_up_previous_vs_current.png) · [현재9자세](../cad/gesture_hand_v3/generated/gestures_oblique.png) · [대표 중간 자세](../cad/gesture_hand_v3/generated/motion_review/core_paths_oblique.png)

[보→바위](../cad/gesture_hand_v3/generated/motion_review/open__fist.gif) · [보→엄지척](../cad/gesture_hand_v3/generated/motion_review/open__thumb_up.gif) · [바위→엄지척](../cad/gesture_hand_v3/generated/motion_review/fist__thumb_up.gif)

실제 CAD의 각도경로 미리보기다. 원래 경유점을 보존하며 GIF 재생시간은 실물속도가 아니다. 전체20개 전환, 역방향 실행, 표본 사이의 연속 충돌, 전 가동 범위, 줄·모터 통합 및 로봇 단독50/100ms 성능은 검증하지 않았다. 공칭 체결 검사는 제조 출고·강도·공차 승인과 다르다.

## 저장 후 재검토

이 체크포인트를 커밋·푸시한 뒤 정확SHA의 새 사본에서 전체 manifest를 확인하고349개 부품의 형상·질량을 재생성하며, 보·바위·엄지척과 공칭 체결을 재검사한다. 전환은 원래2.5° 전량의 경로·각도를 산술 대조한다. CAD 재검사는 세 경로 각각의 경로 위치0·0.25·0.5·0.75·1에 가장 가까운 기존 검사 표본과, 경유점을 둔 두 엄지 경로(보→엄지척·바위→엄지척)의4개 경유점 전부를 대상으로 한다. 정확히 같은20축 각도는 중복 제거한다. 33프레임 주먹의 모든 구간 첫·끝·중간을 재검사하거나 전체 전환 CAD를 다시 실행하는 범위가 아니다. 원래 전체 표본 검사와 커밋 후 대표 표본 재검토를 구분한다. 아직 커밋 후 재현을 완료한 기록은 아니며 실제 결과는 후속 검토 커밋에 남긴다. 이 한 단계의 결과를 보여준 뒤 추가 개선은 멈춘다.

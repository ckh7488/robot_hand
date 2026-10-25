# 손목 방향 엄지 끝마디 후보 — 정확 SHA 대표 재현

대상 커밋: `2d532b4c8fc0256c309adfdbc8bf38c902c3fb95`

커밋의 manifest와 입력 바이트를 새 D: 사본에 읽고, 기존 349개 저장 STEP 부품을 사용하는 대표 검사를 다시 실행했다. 원래 전체 경로 48표본의 각도·관측 정의가 재구성 결과와 일치했다. 실제 CAD 재실행은 대표 9개 관측, 중복을 제외한 5자세이며 원보고서와 충돌쌍·부피·연산 오류 결과가 일치했다.

재현 범위의 보고 기준 이상 재료 겹침과 연산 실패는 0건이다. 전체 경로 CAD 재실행, 형상 재생성, 거리·최소 유격, 렌더, 역방향, 연속 충돌, 실물 가동범위, 동역학·100ms 성능은 이번 재현에 포함하지 않았다. 같은 작업의 새 사본 재현이며 외부 독립 감사나 제조 승인이 아니다.

- [재현 보고서](review.json) · [대표 저장 STEP 검사](representative_audit.json)
- [대상 후보](https://github.com/ckh7488/robot_hand/tree/2d532b4c8fc0256c309adfdbc8bf38c902c3fb95/cad/thumb_wrist_v1)
- 재현기 SHA256: `b5635dece99f60f5d9ae43d2bfb61adc120ecb612b1307f6a562c9e755289a6c`
- 후보 저장기 SHA256: `e839c362398fbc5a22837b9aba8e388e4997c14614f80f5ae4f0090ae15e74a9`
- 이 후속 저장기 SHA256: `e6c795c209231d41bbe35c86f5a656a96a908b272064a1c70b30609997819bc1`

CHECKPOINT의 GESTURE002 기준과 기존 PROGRESS 작업 사본은 유지했다. 전체 경로 검사와 대표 재현의 범위를 구분한다.

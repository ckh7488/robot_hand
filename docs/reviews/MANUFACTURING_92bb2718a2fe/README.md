# 제작 후보 — 정확 SHA 저장 CAD 재검사

대상 커밋: `92bb2718a2fe2ec3556039f9798f01771c1f3638`

커밋 manifest의 전체 입력을 새 D: 사본에 정확한 바이트로 읽고, 저장된 native STEP 349개를 다시 검사했다. 원본 자료 222개의 해시도 대상 커밋과 기존 기준에서 확인했다. CAD 재생성이나 이미지 재렌더는 수행하지 않았다.

핀 10개마다 공칭 30mm 축방향 삽입 sweep을 다시 계산했다. 이전 D형 통로의 막힘 10건이 재현됐고, 새 좌측 부싱 통로 막힘 0건, 관절 부분조립에서 막힘 0건, 완성 손 중립자세에서 막힘 5건이 원보고서와 일치했다. 완성 손 상태의 막힘은 해소된 것으로 표시하지 않으며 부분조립 순서의 추가 검증 대상으로 남긴다.

클립 10개의 전체 회전 포락체를 포함한 열 가지 정적 끝자세에서 보고 기준 이상 겹침 0건, 연산 실패 0건을 재현했다. 손목 방향 −15° 자세는 이전 엄지척 슬롯을 대체한 자세다. 연속 손 동작, 최소 유격·공차, 클립 탄성 삽입, 강도·마찰·하중, 구동 및 100ms 성능은 이번 검토 범위가 아니다. 같은 작업의 새 사본 재현이며 외부 독립 감사나 제조 승인이 아니다.

남은 제작 문제:

- Five pin insertions are blocked by neighboring fingers in the fully assembled neutral hand; finger pin preassembly is required. Other component/tool/module insertion paths are unverified.
- Current custom retaining clips lack material/elastic installation/pull-out approval. Catalog E-ring groove width, end margin and installed envelope require a new CAD revision.
- Nominal pin, bushing and thrust clearances have no manufacturing tolerance stack, process approval, friction or fatigue validation.
- Simplified fasteners, screwdriver/nut/washer access, fastening strength and complete assembly/service sequence are unverified.
- Motors, tendons, pretension hardware, wires and actual joint sensing are not integrated into this hand. Drive torque must act on the child link, not the free pin.
- Only ten static hand poses and ten axial pin insertion paths are checked. Continuous hand transitions, full physical ROM, strength and 50/100ms performance remain unverified.

- [정확 SHA 재현 보고서](review.json) · [전체 저장 CAD 조립 검사](assembly_audit.json)
- [실행 로그](assembly_audit.log) · [후보 원본](https://github.com/ckh7488/robot_hand/tree/92bb2718a2fe2ec3556039f9798f01771c1f3638/cad/manufacturing_v1)
- 검토 실행기 SHA256: `d0744b66202307667d8880a7f9ae22d4e8ff121f414ad9e3d9d0c05db3ccc365`
- 공용 검증 계약 SHA256: `4791cf0aad2ab430c19ade63a5eea6674140f3623df3aa0a19901904157a8a75`
- 이 후속 저장기 SHA256: `1efbada8082d652ac793037ad291b4a9d8734e5c2e991d8a56f4f3d97c14d80f`

기존 CHECKPOINT와 PROGRESS 작업 사본, 무관한 기존 변경 파일은 보존했다. 이 커밋에는 이 검토 폴더만 추가한다.

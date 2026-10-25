# 제작 후보 — 정확 SHA 저장 CAD 재검사

대상 커밋: `a469af796d4e8300ddde6b9dee06db28c10b3591`

커밋 manifest의 전체 입력을 새 D: 사본에 정확한 바이트로 읽고, 저장된 native STEP 부품359개와 별도 드릴 가공용 블랭크10개를 다시 검사했다. 블랭크는 완성 부품이나 조립 질량에 포함하지 않았다. 원본 자료222개의 해시도 대상 커밋과 기존 기준에서 확인했다. CAD 재생성이나 이미지 재렌더는 수행하지 않았다.

원위 핀10세트마다 핀·받침 와셔·나사의 공칭 접근과 육각/일자 공구 접근 다섯 종류를 재계산했다. 자기 손가락 부분조립의 막힘은 {'pin_insertion': 0, 'keeper_insertion': 0, 'screw_outer_approach': 0, 'hex_driver_approach': 0, 'flat_driver_approach': 0}이며, 완성 손 중립자세의 막힘은 {'pin_insertion': 5, 'keeper_insertion': 6, 'screw_outer_approach': 6, 'hex_driver_approach': 6, 'flat_driver_approach': 6}로 원보고서와 일치했다. 공구를 크게 만든 양성대조20건도 재현했다. 완성 손 상태의 막힘은 해소된 것으로 표시하지 않으며 부분조립 순서의 추가 검증 대상으로 남긴다.

핀·받침·나사·기존 양쪽 와셔의 다섯 부품을 포함한 회전/축방향 자유도 포락체10개와 나머지309부품을 합쳐319개 충돌 대상으로 검사했다. 세트 내부 명목10쌍과 상관된 축방향 이동의 네 꼭짓점도 검사한다. 열 가지 정적 자세와 정방향 경로48표본(관절 최대간격5°)에서 보고 기준 이상 겹침 0건, 연산 실패 0건이 재현됐다. 이는 연속 손 동작이나 최소 유격, 편심·기울기·공차, 실제 나사산의 체결, 조임력·풀림·강도, 구동 및100ms 성능의 승인이 아니다. 같은 작업의 새 사본 재현이며 외부 독립 감사나 제조 승인이 아니다.

남은 제작 문제:

- Whole-hand neutral access obstructions by approach type remain: {"flat_driver_approach": 6, "hex_driver_approach": 6, "keeper_insertion": 6, "pin_insertion": 5, "screw_outer_approach": 6}. Nominal own-finger access does not establish a complete assembly or service sequence.
- The actual 0.4 mm pilot chamfer permits a nominal 4.0 mm full-form internal thread and 2.8 mm screw overlap before incomplete-tip allowances. Smooth major-diameter envelopes do not certify tap/runout manufacture, actual thread fit, engagement strength, preload or loosening resistance.
- The declared aligned screwdriver gauges are not full vendor tool CAD. Actual tip transitions, tolerances, tool clocking and complete-hand assembly/service access require measurement.
- The existing outer washers have only coaxial nominal rotation and correlated axial freedom screened. Radial eccentricity, tilt, dimensional tolerances, wear, friction, fatigue and load-bearing strength remain unverified.
- Twenty inner thrust washers and child link/bushing axial play were screened only at their nominal saved positions. Their local gaps, correlated motion and surrounding restraints require a further joint-support study; body assignment is not physical fixation.
- The ten pilot-only STEP blanks are machining-operation references, not finished parts. Manufacturing process, material, tolerances, inspection and procurement specifications remain incomplete.
- Motors, tendons, pretension hardware, wiring and joint sensing remain unintegrated. The free hinge pin is not the child-link drive shaft.
- Ten static poses and 48 forward-path observations at maximum 5 degree joint spacing are sampled only. Continuous clearance, other transitions, complete physical range and robot-only 50/100 ms performance remain unverified.

- [정확 SHA 재현 보고서](review.json) · [전체 저장 CAD 조립 검사](assembly_audit.json)
- [실행 로그](assembly_audit.log) · [후보 원본](https://github.com/ckh7488/robot_hand/tree/a469af796d4e8300ddde6b9dee06db28c10b3591/cad/manufacturing_v2)
- 검토 실행기 SHA256: `52eef913c4a0e5a2d530cbfcc639c119867e5ce48d666ea130298aeba4304250`
- 공용 검증 계약 SHA256: `47c2eb9414afbc0b3dbf94a5df8537909c3c25840a0c8b3653a8d31c4fdd7657`
- 이 후속 저장기 SHA256: `2be3358b1522974339217ce81071e13d37015f56cc87143091e67898bb77e9ea`

기존 CHECKPOINT와 PROGRESS 작업 사본, 무관한 기존 변경 파일은 보존했다. 이 커밋에는 이 검토 폴더만 추가한다.

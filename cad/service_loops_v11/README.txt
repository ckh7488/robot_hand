V11 로봇 손 CAD 검토 — 별도 후보 / 제작 승인 아님

먼저 볼 파일
- cad/service_loops_v11/reviews/v11_review.html : 그림과 검사 결과
- cad/service_loops_v11/V11_open_hand_loop_packaging.FCStd / .step : 손 전체 Open CAD + 두 출선 + 한 관절 루프 후보
- cad/service_loops_v11/parts/Little_root_cable_saddle.step / .stl
- cad/service_loops_v11/parts/Little_MCP_link_cable_saddle.step / .stl

이번 수정
1. 새끼손가락 출력 부품2개에 얇은 막대 대신 넓은 일체형 전선 받침을 추가했습니다.
2. 받침–프레임 접촉과 받침–출선 곡선 겹침을 실제 형상으로 찾아 고쳤습니다. 원래 체결 구멍/축은 그대로입니다.
3. PIP 출선을 MCP와 같은 손등 방향으로 바꿨습니다.
4. 전선3가닥이 교차하던 개별 루프를 함께 계산하고, 자세 사이 형상이 튀던 분기도 수정했습니다.

검사의 범위
- 받침 추가부:1458관절 표본 +28정밀 BRep 부품 검사, 최소0.5mm.
- 고정 출선:20모터×6자세=120정밀 BRep 검사. 모터 자체 커넥터 체결은 종전과 같이 미승인.
- 새끼 MCP 루프:6배경 자세×41MCP각도=246표본. 곡선을 보수적으로 감싼 캡슐과 실제 CAD 메시에 거리 검사를 했습니다.
- 전선의 연속곡선은 제어점 볼록껍질 기준0.01mm 이내로 분할해 감쌌습니다. 관절각 표본 사이 전체를 증명하지는 않습니다.
- 루프의 각 가닥 자유구간 길이는140mm입니다. 이는 연결점 사이 기하학 후보 길이이며, 커넥터부터의 전체 구매/절단 길이가 아닙니다.
- 반경10mm와 OD1.9mm는 설계 가정입니다. 실제 전선의 반복 굽힘 사양이 검증됐다는 뜻이 아닙니다.
- 최종 루프는 베지어 기하 모델입니다. 실제 전선 탄성/중력/마찰/히스테리시스/케이블타이 고정 상태를 예측한 유연체 시뮬레이션이 아닙니다.
- 받침은 아직 개방형 홈입니다. 케이블타이 등 고정 방법과 실제 응력 완화 기능은 실측해야 합니다.

아직 확정하지 않은 것
- 손20개 모터 전체의 연결 배선과 전원 분배, 중간 분기/두 번째 커넥터 사용.
- 기구의 FDM 강도/수명, 실제 서보 속도, 손 전체 추종 동역학.
- 팔 모터의 정격 여유:V9의 초과/부족 결과는 해결되지 않았습니다.
이 묶음은 위 문제를 해결했다는 최종본이 아닙니다. 추가된 두 출력 부품은 먼저 소량 시제품으로 검사할 후보입니다.

자료 구성과 재실행
V9 감사본은 수정하지 않았습니다. 이 묶음에는 V10와 V11 폴더를 포함합니다.
FCStd/STEP은 자체로 열 수 있습니다. 파이썬 검사를 재실행하려면 V9 감사본을 프로젝트 루트에 먼저 풀고 이 묶음을 같은 위치에 덮어 풀어 주세요.
기존 modular_roots_v9 폴더를 덮어쓰는 파일은 없습니다.
필요 환경:FreeCAD Part/Mesh, numpy, scipy, trimesh, python-fcl, rtree, mujoco, Pillow.
모든 *.py를 한꺼번에 실행하지 마세요. *_rejected와 초기 fit_* 파일은 실패 후보 기록입니다.
현재 검증 대상은 reviews/service_loop_curves.json과 parts의 두 가이드입니다.
전선 STL은16각 원단면과0.005mm 제어점 볼록껍질 분할을 쓰는 표시용입니다. 충돌 검사에는 이 표시 메시를 사용하지 않고 곡선 제어점과 보수적 캡슐을 사용합니다.
검사 순서:review_guide_motion.py → prepare_exact_cases.py → review_guides_exact.py,
validate_selected_routes.py, check_tail_pairs.py, review_loop_geometry.py,
validate_loop_mesh.py, check_loop_connections.py, check_integrity.py.
CAD/이미지 재생성:export_loop_cad.py → make_loop_display_meshes.py → render_v11.py.
곡선 전체 재탐색이 필요할 때만 fit_free_quintic_loops.py → smooth_loop_branch.py를 실행합니다.
Windows에서는 pythonw.exe와 숨김 프로세스를 사용합니다. FreeCADCmd 창을 띄울 필요가 없습니다.

벤치 계획
hardware_bench_addendum.json은 기존 V9 계획에 배선 저항 토크 측정을 더합니다.
actuator_reference_corrections.json은 공식 XL330 문서의 PWM Slope/입력 전류/모드5 관련 해석을 바로잡습니다.
하드웨어는 연결하거나 조작하지 않았습니다.

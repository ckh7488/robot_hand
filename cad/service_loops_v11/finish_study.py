"""Collect current V11 evidence and package a separate review checkpoint; preserve V9/V10."""
import json,hashlib,zipfile,base64,html,datetime
from pathlib import Path
D=Path(__file__).resolve().parent;P=D.parent.parent;W=D.parent/'wiring_access_v10'
def read(n):return json.loads((D/'reviews'/n).read_text(encoding='utf-8'))
geom=read('guide_geometry.json');motion=read('guide_motion_mesh.json');exact=read('guide_exact.json');fixed=read('selected_exact.json');pairs=read('fixed_tail_pairs.json');loops=read('loop_mesh_clearance.json');qc=read('loop_geometry_verified.json');joins=read('loop_connections.json');integrity=read('candidate_integrity.json');roundtrip=read('cad_roundtrip.json');smooth=read('loop_smoothing.json');mixed=read('mixed_fingers.json')
assert motion['failed_samples']==0 and exact['failed_part_cases']==0
assert len(fixed['rows'])==120 and fixed['all_selected_fixed_exits_passed']
assert pairs['all6sampled_fixed_tail_pairs_passed']
assert loops['cases']==246 and loops['failed_cases']==0 and loops['minimum_wire_wire_gap_mm']>=.3
assert joins['self_collision_cases']==0 and joins['own_exit_collision_cases']==0
assert qc['minimum_dense_sampled_radius_mm']>=10 and qc['maximum_absolute_length_error_mm']<1e-4 and qc['largest_matched_curve_point_shift_per_2deg_mm']<=6.001
assert smooth['passed'] and roundtrip['valid'] and integrity['V9_frozen_hashes_unchanged']==222
assert all(r['removed_original_volume_mm3']<1e-7 and r['valid_single_solid'] for r in geom['parts'])
added_mass_g=sum(r['added_volume_mm3'] for r in geom['parts'])*1.25/1000
summary={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'status':'Geometric and CAD assembly checkpoint; not a production or fast-tracking approval.','baseline':'V9 audit unchanged; V10 thumb channel retained as a candidate.','design_changes':['Two broad integral cable saddles in little-finger Compact_root and55mm MCP link. Existing hole/axis geometry retained.','Little PIP fixed exit changed from back to outboard, matching little MCP.','Candidate3-wire constant-length140mm service-loop family, with inter-wire clearance and branch continuity corrected.','Manufacturer-grounded actuator reference and cable-drag bench addendum added.'],'guide_body_mesh_cases':motion['samples'],'guide_body_exact_part_cases':exact['part_cases'],'guide_body_minimum_gap_mm':exact['minimum_gap_mm'],'fixed_exit_exact_cases':len(fixed['rows']),'connected_loop_sampled_cases':loops['cases'],'loop_reserved_body_minimum_gap_mm':loops['minimum_body_gap_mm'],'loop_actual_wire_minimum_gap_mm':loops['minimum_wire_wire_gap_mm'],'loop_minimum_dense_sampled_radius_mm':qc['minimum_dense_sampled_radius_mm'],'loop_free_length_per_wire_mm':qc['target_length_mm'],'loop_maximum_length_error_mm':qc['maximum_absolute_length_error_mm'],'loop_maximum_matched_point_shift_per_2deg_mm':qc['largest_matched_curve_point_shift_per_2deg_mm'],'full_solid_polymer_density_assumption_g_cm3':1.25,'added_guide_mass_estimate_g':added_mass_g,'CAD_roundtrip_solids':roundtrip['roundtrip_solids'],'full_harness_validated':False,'manufacturing_release':False,'measured_servo_response':False,'remaining':['Cable ties/retainers and actual plug mating are not qualified. The open saddles are packaging candidates, not a demonstrated strain-relief clamp.','Bezier curves are feasible geometric candidates. They are not the shape predicted by a cable constitutive/dynamic model; no gravity, friction, creep, hysteresis or fatigue approval.','Only one little-finger inter-joint free span is connected here. One-port test/tap topology remains unapproved; this is not a complete20-motor electrical harness.','Other background and continuous joint states, print/servo errors and dynamic motor following remain unverified.','V9 arm gravity/rating shortfall and unmeasured real-time response remain unresolved.']}
(D/'reviews'/'V11_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
readme='''V11 로봇 손 CAD 검토 — 별도 후보 / 제작 승인 아님

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
'''
(D/'README.txt').write_text(readme,encoding='utf-8')
img=base64.b64encode((D/'reviews'/'v11_preview.png').read_bytes()).decode();metrics=[('추가 받침 운동 표본',str(motion['samples'])+'개, 충돌0'),('받침 정밀 표면 거리',f"{exact['minimum_gap_mm']:.3f}mm 이상 /28부품 자세"),('고정 출선 검사','120개 통과'),('연결 루프 검사','246개 표본 통과'),('루프–CAD 여유',f"{loops['minimum_body_gap_mm']:.3f}mm (전선 반경 여유 포함)"),('루프 전선끼리',f"{loops['minimum_wire_wire_gap_mm']:.3f}mm 이상"),('각 가닥 루프 자유 길이',f"{qc['target_length_mm']:.0f}mm"),('밀집 표본 최소 굽힘 반경',f"{qc['minimum_dense_sampled_radius_mm']:.3f}mm / 설계 가정10mm"),('자세간 최대 형상 변화',f"2°당{qc['largest_matched_curve_point_shift_per_2deg_mm']:.3f}mm"),('STEP 재개방',str(roundtrip['roundtrip_solids'])+'솔리드, 유효')]
body='<!doctype html><meta charset="utf-8"><title>V11 CAD 검토</title><style>body{font:16px/1.7 system-ui,Malgun Gothic;margin:36px auto;padding:0 24px;max-width:1150px;background:#f4f7fa;color:#203449}img{width:100%;background:white}table{border-collapse:collapse;width:100%;background:white}td{padding:10px 15px;border-bottom:1px solid #d8e0e9}h1{font-size:28px}.note{padding:18px;background:#fff3d9;border-radius:10px}pre{white-space:pre-wrap;font:14px/1.8 system-ui}</style><h1>V11 · 배선 받침과 한 관절 루프 후보</h1><p>프레임 접촉, 출선 곡선의 간섭, 전선 교차와 자세 사이 형상 튐을 고쳤습니다. 기존 손 비율과 관절 축은 유지했습니다.</p><img alt="실제 CAD 수정 이미지" src="data:image/png;base64,'+img+'"><table>'+''.join('<tr><td>'+html.escape(a)+'</td><td>'+html.escape(b)+'</td></tr>' for a,b in metrics)+'</table><p class="note">현재 통과 범위는 CAD·기하학 표본 검사입니다. 전선의 실제 힘과 처짐, 고정구, 전체 배선과 실시간 서보 성능은 검증되지 않았습니다. 팔 모터의 정격 문제도 남아 있습니다.</p><pre>'+html.escape(readme)+'</pre>'
(D/'reviews'/'v11_review.html').write_text(body,encoding='utf-8')
# Freeze reproducibility inputs and results in this checkpoint; exclude caches and FreeCAD backups.
def included(p):return p.is_file() and '__pycache__' not in p.parts and 'mpl_config' not in p.parts and p.suffix.lower() not in ['.fcbak','.pyc'] and not p.name.endswith('.tmp')
files=sorted([p for folder in [W,D] for p in folder.rglob('*') if included(p) and p.name!='V11_EVIDENCE_INDEX.json'])
index={p.relative_to(P).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
(D/'V11_EVIDENCE_INDEX.json').write_text(json.dumps({'scope':'V10/V11 checkpoint hashes; V9 remains separately frozen.','sha256':index},indent=2),encoding='utf-8');files.append(D/'V11_EVIDENCE_INDEX.json')
archive=P.parent/'robot-hand-v11-review.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 z.writestr('START_HERE_V11.txt',readme)
 for p in files:z.write(p,p.relative_to(P).as_posix())
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 for p,h in index.items():assert hashlib.sha256(z.read(p)).hexdigest()==h,p
print(json.dumps({'archive':str(archive),'bytes':archive.stat().st_size,'files':len(files)+1,'summary':summary},indent=2),flush=True)

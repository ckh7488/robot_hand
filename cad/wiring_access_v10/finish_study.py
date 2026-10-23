import json,hashlib,base64,zipfile,datetime,html
from pathlib import Path
D=Path(__file__).resolve().parent;P=D.parent.parent;rd=lambda p:json.loads(p.read_text(encoding='utf-8'))
a=rd(D/'reviews'/'selected_exact_initial.json');b=rd(D/'reviews'/'selected_exact_delta.json');assert len(b['rows'])==6 and b['all_selected_fixed_exits_passed'];rows=[r for r in a['rows'] if r['motor']!='little_mcp_motor']+b['rows'];selected=rd(D/'selected_routes.json')['selected'];choices={r['motor']:r['template'] for r in selected}
assert len(rows)==120 and all(r['passed'] and r['template']==choices[r['motor']] for r in rows)
combined={k:v for k,v in a.items() if k!='rows'};combined['rows']=rows;combined['provenance']='Initial120exact checks; little_mcp6rows replaced by independently rechecked outboard route after tail-pair conflict.';combined['elapsed_seconds']=a['elapsed_seconds']+b['elapsed_seconds'];combined['BRep_uncached_distance_queries']=a['BRep_uncached_distance_queries']+b['BRep_uncached_distance_queries'];combined['all_selected_fixed_exits_passed']=True
(D/'reviews'/'selected_exact_final.json').write_text(json.dumps(combined,indent=2),encoding='utf-8')
pairs=rd(D/'reviews'/'fixed_tail_pairs.json');assert pairs['all6sampled_fixed_tail_pairs_passed'];integrity=rd(D/'reviews'/'candidate_integrity.json');assert integrity['V9_frozen_hashes_unchanged']==222;thumb=rd(D/'reviews'/'thumb_channel.json');original=rd(D/'reviews'/'lead_access.json');minimum=min(r['minimum']['distance_mm'] for r in rows);near_fastener=min(r['removed_material_distance_mm'] for r in thumb['related_fasteners'])
summary={'revision':10,'scope':'Fixed lead exit packaging study, not complete electrical harness or final CAD release','V9_frozen_hashes_unchanged':222,'original_motors_with_blocked_fixed_leads':{p['pose']:sum(bool(m['wire_collisions']) for m in p['motors']) for p in original['poses']},'templates_compared':14,'pose_motor_template_tests':1680,'final_fixed_exit_BRep_tests':120,'final_fixed_exits_passed':True,'minimum_exact_reserved_envelope_gap_mm':minimum,'mutual_fixed_tails_mesh6poses_passed':True,'thumb_link_removed_volume_mm3':thumb['removed_volume_mm3'],'thumb_link_removed_fraction_percent':100*thumb['removed_volume_mm3']/thumb['old_volume_mm3'],'minimum_removed_material_to_fasteners_mm':near_fastener,'geometry_changes':'One separate thumb-link cable channel; no V9 file changed','pending':['Genuine mated housing and3mmstraight-exit strain relief','Connected inter-joint cables and service slack','Both ports where electrically required','Full swept motion and servo tracking error','Electrical branch/protection and measured current','Print material, layer strength, creep and fatigue'],'manufacturing_release':False,'full_harness_validated':False}
(D/'reviews'/'study_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
readme='''V10 배선 공간 추가 검토 — V9 감사본은 변경하지 않았습니다.

이 압축은 앞서 전달한 robot-hand-v9-audit.zip에 덧붙이는 자료입니다.
두 ZIP을 같은 위치에 풀면 mujoco-direct/cad/wiring_access_v10 폴더가 추가됩니다.
V9 형상·코드·결과222해시는 변경되지 않았습니다. 기존 V9 감사는 그대로 진행할 수 있습니다.

추가 검토 결과
- 기존 한 관절 시험용 전선 경로를20모터×6자세에 넣어 실제 BRep 겹침을 검사했습니다.
  Open10 / Curl9 / Splay6 / Backbend10 / Close6 / ThumbSweep9모터에서 그 경로가 막혔습니다.
  이 결과는 그 시험용 경로가 불가하다는 뜻이며 모든 배선이 불가능하다는 뜻은 아닙니다.
- 포트 방향·꺾임 방향·직선 출선 길이를 바꾼14후보×20모터×6자세를 실제 CAD메시로 비교했습니다.
- 19모터는 경로 선택으로 피하고, 엄지 PIP는 별도 Thumb_metacarpal에 국부 홈을 만든 후보를 썼습니다.
- 선택20출선×6자세120건은 실제 BRep 거리로 통과했습니다. 최저 여유는 약0.35mm이며,
  이는 실제 전선OD1.9mm보다 반경0.3mm큰 여유 외곽 기준입니다. 제조 공차/추종오차 인증은 아닙니다.
- 전선들을 함께 배치하니 새끼MCP/PIP사이에 겹침이 있어 새끼MCP를 바깥쪽 출선으로 바꿨습니다.
  수정한6자세를 BRep로 재검사하고, 서로 다른 모터의 전선·커넥터 외곽끼리도6자세 메시검사를 통과했습니다.
- 엄지 링크는134.14mm3(약0.67%)만 제거한 유효1솔리드 후보입니다.
  제거 부위와 관련 나사·너트 사이 최소거리는11.57mm, 기존 재료 밖에 더한 부피는0입니다.
  STL 닫힘/방향/1연결 검사는 통과했으며 구조 강도 승인은 아닙니다.

열어 볼 파일
wiring_review.html: 수치와 실제 CAD 비교 이미지가 포함된 보고서.
Thumb_metacarpal_channel_candidate.FCStd / parts/동명STEP: 엄지 홈 후보만 들어 있는 독립 CAD.
reviews/study_summary.json: 요약. selected_exact_final.json: 최종120건 실제 거리.
reviews/fixed_tail_pairs.json: 선택된 출선끼리6자세 간섭 검사.
selected_routes.json: 모터별 임시 출선 선택.
parts/: 공칭 커넥터 외곽, 전선 여유 외곽, 엄지 후보 STEP/STL(모두mm).

범위와 남은 일
여기서 검사한 것은 모터마다 커넥터1개와 짧은 고정 출선3가닥입니다.
관절을 건너 두 모터를 연결한 완성 케이블이 아니며, 각 전선의 끝은 아직 연결되지 않았습니다.
중간 모터에 필요한 두 번째 커넥터, 조립 중 케이블 통과, 인출 공구, 움직이는 여유 루프,
전원 분기와 보호, 수명, 전체 연속 운동 범위는 검증되지 않았습니다.
일부 후보는 하우징 뒤3mm에서 굽힘을 시작하며 이는 공간 가정입니다. 정품 케이블의
크림프·직선부·스트레인릴리프를 실측해야 채택할 수 있습니다. 굽힘 반경10~15mm는 설계 여유 가정입니다.
JST하우징은 공칭 외곽이며 자체 모터 케이스와의 체결 적합성을 인증하지 않았습니다.
서보 동역학, 실제 빠르기, 팔 모터 재선정은 이번 검토 범위 밖입니다.

재현 (V9 감사 ZIP과 함께 풀고, 별도 복사본에서 실행)
FreeCAD1.0동봉Pythonw: make_route_templates.py / review_lead_access.py / make_thumb_channel.py
분석Pythonw(기존 requirements-audit.txt 환경): compare_routes.py
선택 순서는 selected_routes_initial.json, selected_routes.json에 보관했습니다.
FreeCAD동봉Pythonw: validate_selected_routes.py (현재 최종20선택을120건 직접 재실행)
분석Pythonw: check_tail_pairs.py / check_candidate_and_baseline.py
모든 스크립트는 현재 폴더의 상대경로로 V9 데이터를 읽습니다. 기존 V9를 수정하지 않습니다.
선택전 검사는 selected_exact_initial.json, 마지막6건 보완은 selected_exact_delta.json에 남겼습니다.
보고서 최종120건은 두 결과를 모터/자세 기준으로 병합하고 선택 파일과 일치함을 확인했습니다.
'''
(D/'README.txt').write_text(readme,encoding='utf-8')
def img(n):return '<img src="data:image/png;base64,'+base64.b64encode((D/'reviews'/n).read_bytes()).decode()+'">'
body='<!doctype html><html lang="ko"><meta charset="utf-8"><title>V10 배선 공간 추가 검토</title><style>body{font:16px/1.7 system-ui,Malgun Gothic;background:#eff4fa;color:#24394c;margin:30px auto;max-width:1100px;padding:0 20px}img{width:100%;margin:18px 0}pre{white-space:pre-wrap;background:white;padding:24px;border-radius:14px}</style><h1>V10 · 배선 공간 추가 검토</h1><p>V9 감사본 보존 / 고정 출선20개×6자세 실제 거리 통과 / 전체 연결 하네스와 실물 체결은 검증 전</p>'+img('wiring_comparison.png')+img('thumb_channel.png')+'<pre>'+html.escape(readme)+'</pre></html>'
(D/'wiring_review.html').write_text(body,encoding='utf-8')
archive=P.parent/'robot-hand-v10-wiring-study.zip';hashes={}
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in sorted(D.rglob('*')):
  if not p.is_file() or any(s in ['__pycache__','mpl_config'] for s in p.relative_to(D).parts) or p.suffix not in ['.py','.json','.txt','.step','.stl','.FCStd','.html','.png']:continue
  name=(Path('mujoco-direct')/p.relative_to(P)).as_posix();data=p.read_bytes();z.writestr(name,data);hashes[name]=hashlib.sha256(data).hexdigest()
 z.writestr('V10_WIRING_README.txt',readme);z.writestr('v10_package_manifest.json',json.dumps({'summary':summary,'sha256':hashes},indent=2))
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 for n,h in hashes.items():assert hashlib.sha256(z.read(n)).hexdigest()==h
print(json.dumps({'archive':str(archive),'bytes':archive.stat().st_size,'files':len(hashes),'summary':summary},indent=2))

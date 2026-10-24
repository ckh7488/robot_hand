"""Create the fabrication-readiness decision from completed measured evidence."""
import base64
import hashlib
import html
import json
from datetime import datetime,timezone
from pathlib import Path

HERE=Path(__file__).resolve().parent
OUT=HERE/'generated/manufacturing_audit'


def main():
    assembly=json.loads((OUT/'assembly.json').read_text())
    interfaces=json.loads((OUT/'interfaces.json').read_text())
    walls=json.loads((OUT/'walls.json').read_text())
    mesh=json.loads((OUT/'mesh_cleanup.json').read_text())
    assert assembly['completed'] and walls['completed']
    for report in (assembly,interfaces,walls):
        for relative,digest in report['input_sha256'].items():
            assert hashlib.sha256((HERE.parents[1]/relative).read_bytes()).hexdigest()==digest,relative
    root_hits=[p for p in assembly['cases']['open']['overlaps'] if '_root_fork_' in p['a'] and p['b'].endswith('joint_pin_0')]
    assert len(root_hits)==10
    import math
    assert all(abs(p['volume_mm3']-6*math.pi)<1e-6 for p in root_hits)
    cases={k:dict(all_pairs=len(v['overlaps']),different_bodies=sum(not p['same_simulator_body'] for p in v['overlaps'])) for k,v in assembly['cases'].items()}
    structures=[p for p in walls['parts'] if 'faceplate' not in p['name']]
    thinnest=min(structures,key=lambda p:p['sampled_min_mm'])
    affected_mesh=[p for p in mesh['parts'] if p['removed_degenerate_count']]
    findings=[
        dict(id='M01',severity='조립 불가',title='뿌리 핀 구멍을 지지살이 막음',
            evidence='5개 손가락 × 양쪽 지지대 = 10곳. 각 핀–지지대 교차 체적 18.8496 mm³. 반지름2mm 핀의 아래 절반 × 지지대 두께3mm = 6π mm³와 일치한다.',
            cause='구멍이 있는 원형 지지대에 직육면체 지지살을 합친 뒤, 최종 관통 구멍을 다시 뚫지 않았다.',
            action='최종 합친 형상에 핀 구멍을 관통 가공하고 축·공차·핀 삽입 및 빠짐 방지까지 같이 정의한다.'),
        dict(id='M02',severity='운동 간섭',title='뿌리 회전 받침이 움직이는 마디와 겹침',
            evidence='편 손에서 네 손가락 고정 받침–마디 교차가 각각 약629~631mm³, 회전 링–마디는 각각 약486.65mm³다. 주먹에도 서로 다른 강체 간 간섭이 남는다.',
            cause='이전13자세 검사는 주요 외피만 대상으로 했고, 회전 링·고정 받침·부싱·핀을 제외했다.',
            action='축 높이·받침·관절 눈의 단면을 함께 다시 설계하고 전 부품을 넣어 운동 간섭을 재검사한다.'),
        dict(id='M03',severity='연결 누락',title='회전 링과 지지대 사이에 하중 전달 경로가 없음',
            evidence='네 손가락에서 링–왼쪽/오른쪽 지지대 최소거리가 모두1.00mm다. 중지 고정 받침–손바닥은0.431mm, 엄지는2.524mm 떨어져 있다. 엄지 링과 지지대는 별도 부품끼리 각각51.64mm³ 겹친다.',
            cause='시뮬레이터의 같은 강체 묶음이 실제 결합을 대신하고 있다. 중앙 벌림축의 축·축방향 유지 구조도 완성돼 있지 않다.',
            action='일체 제작할 부품과 따로 조립할 부품을 결정하고, 실제 연결 살·좌면·볼트·베어링/부싱 지지 구조로 하중 경로를 닫는다.'),
        dict(id='M04',severity='유지·공차 미완',title='핀·부싱·장식판·손끝 캡이 고정되지 않음',
            evidence='핀Ø4 / 부싱 내경Ø4.4: 지름차0.4mm. 부싱 외경Ø6.7 / 관절 구멍Ø6.8: 지름차0.1mm. 핀 반대쪽 잠금, 부싱 유지, 9개 장식판 및5개 손끝 캡의 체결 정의가 없다.',
            cause='상용 부싱 품번·설치 방식·끼워맞춤·축방향 간격·핀 고정 방식이 지정되지 않았다. 6개 덮개 나사는 머리만 있고 몸통 길이가0이다.',
            action='핀/어깨볼트와 너트 또는 유지링, 부싱 품번·끼워맞춤, 와셔/스페이서, 실제 나사 길이·나사산 물림·공구 접근을 정의한다.'),
        dict(id='M05',severity='국소 얇은 형상',title='접힘 홈을 낸 뒤 남은 가장자리가 매우 얇음',
            evidence=f"구조부의 표본 최소 두께는 {thinnest['sampled_min_mm']:.4f}mm ({thinnest['name']}). 유효 솔리드이거나 명목 벽 두께2mm여도 절삭 경계 전체의 두께를 보장하지 않는다.",
            cause='굽힘 간섭을 덜어내는 형상 절삭이 얇은 쐐기·날카로운 끝을 남긴다. 수치는 면별 표본의 국소 법선 방향 두께이며 전역 최소값 인증은 아니다.',
            action='공정별 최소 형상 크기와 하중을 정한 뒤 절삭 경계에 충분한 단면·라운드·보강부를 확보한다. 방향별 강도와 피로는 별도 검증한다.'),
        dict(id='M06',severity='구동 연결 미완',title='수정한 손에 모터·전달계가 연결되지 않음',
            evidence='이 CAD의 설치 모터는0개다. 별도 form_drive_v1의20모터 배치는 이전 외형 기준이며, 새 엄지의21mm 축 높이 변경·65도 축 방향 변경과 새 주먹의PIP122~133도를 반영하지 않는다. 기존 구동 입력의 굽힘 상한은100도다.',
            cause='줄·풀리·외피 끝단·장력 조절·출력축 결합과 수정된 손의 하중·관성이 하나의 조립으로 통합되지 않았다.',
            action='실제 뿌리 관절 모듈을 먼저 완성한 뒤 새 좌표계로 모터/줄 경로, 길이 변화, 장력, 움직이는 질량을 다시 연결한다.'),
        dict(id='M07',severity='가동 범위 미완',title='허용 각도 안에서도 이웃 손가락을 뚫을 수 있음',
            evidence='검지만 벌림−22도, 나머지 편 손 조건에서 검지–중지 가운데 마디 교차 체적은1447.58mm³다. 이 자세는 현재 개별 관절 제한 안에 있다.',
            cause='기준 주먹 경로의 일부 자세 통과를 자유로운20축 조합의 무간섭으로 확대할 수 없다.',
            action='상황별 도달 가능 영역, 기계적 정지부 및 충돌 제한을 정의하고 연속 운동의 최소 여유를 검사한다.'),
        dict(id='M08',severity='출력 파일 정리',title='손끝 STL에 퇴화 삼각형이 있음',
            evidence='손끝 캡5개에 각각 넓이0 수준의 삼각형8개가 있다. 이 삼각형만 진단상 제거하면 열린 경계/비다양체 모서리가0이 된다. 원본 STL은 수정하지 않았다.',
            cause='유효한 STEP의 테셀레이션 출력 문제다. 이것만으로 손끝의 실제 솔리드에 구멍이 있다고 판단하지 않는다.',
            action='수정 형상에서 STL을 다시 내보내고 실제 사용할 슬라이서의 경고·벽/구멍 보존을 확인한다.'),
        dict(id='M09',severity='판정 입력 부족',title='재료·공정·하중·수명 기준이 미정',
            evidence='확정된 재료/제작공정, 손끝 힘·반복 횟수·충격 조건, 허용 처짐/유격, 구매품 공차와 조립 도면이 없다. 기존 밀도1250kg/m³ 등은 가정이다.',
            cause='연결이 없는 부품을 고정한 구조해석이나 스톨토크만으로 제작 성능을 승인할 수 없다.',
            action='한 관절 모듈의 조립·유격·강도 시험 기준부터 정하고, 하중 경로가 완성된 뒤 강도/강성/피로와 구동 성능을 검증한다.')]
    summary=dict(created_at=datetime.now(timezone.utc).isoformat(),audited_geometry_commit='87f6bf17865fb441c407e0651c26f1f19c203a8c',
        verdict='NOT_READY_FOR_FABRICATION',geometry_modified_by_audit=False,
        parts=120,valid_step_solids=sum(p['valid'] and p['solid_count']==1 for p in assembly['parts']),
        assembly_cases=cases,wall_parts=len(walls['parts']),wall_samples=sum(p['samples'] for p in walls['parts']),
        sampled_thinnest_structural_part=thinnest,mesh_parts_with_degenerates=len(affected_mesh),
        root_pin_blocked_interfaces=len(root_hits),findings=findings,
        evidence_files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (OUT/'assembly.json',OUT/'interfaces.json',OUT/'walls.json',OUT/'mesh_cleanup.json')},
        supplementary_source_sha256={str(p.relative_to(HERE.parents[1])):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (HERE/'verify.py',HERE.parent/'form_candidate_v1/build.py',
             HERE.parent/'form_drive_v1/generated/dynamics_contract.json',Path(__file__),
             HERE/'mesh_cleanup_screen.py',HERE/'render_manufacturing_sections.py')},
        manufacturing_process='not specified',strength_approved=False,continuous_motion_approved=False)
    (OUT/'readiness.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    rows=''.join(f"<tr><td><b>{html.escape(f['id'])}</b><br>{html.escape(f['severity'])}</td><td><h3>{html.escape(f['title'])}</h3><p>{html.escape(f['evidence'])}</p><p class='muted'>{html.escape(f['cause'])}</p></td><td>{html.escape(f['action'])}</td></tr>" for f in findings)
    table=''.join(f"<tr><td>{label}</td><td>{cases[key]['all_pairs']}</td><td>{cases[key]['different_bodies']}</td></tr>" for label,key in [('편 손','open'),('주먹','fist'),('검지 벌림−22°','index_spread_minus22_others_open')])
    image=base64.b64encode((OUT/'assembly_blockers.png').read_bytes()).decode()
    document=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>로봇 손 제작성 검토</title>
<style>body{{font:16px/1.7 'Malgun Gothic',sans-serif;color:#202b3c;margin:36px auto;max-width:1180px;padding:0 24px}}h1{{font-size:32px;margin-bottom:8px}}h2{{margin-top:38px}}h3{{font-size:17px;margin:0}}p{{margin:10px 0}}.status{{background:#fff1ee;border-left:5px solid #d44736;padding:20px}}.muted,small{{color:#59687a}}table{{border-collapse:collapse;width:100%;margin:18px 0}}td,th{{border:1px solid #d6dce4;padding:14px;vertical-align:top;text-align:left}}th{{background:#edf2f7}}img{{width:100%;height:auto}}.findings td:first-child{{width:110px}}.findings td:last-child{{width:26%}}@media print{{body{{font-size:10pt;margin:0}}tr{{break-inside:avoid}}h2{{break-after:avoid}}}}</style>
<h1>로봇 손 제작성 검토</h1><p class="muted">2026-10-25 · 대상: webcam_articulation_v1 · CAD 기준87f6bf1 · 원본 형상 변경 없음</p>
<div class="status"><b>판정: 현재 파일 그대로는 제작·조립할 수 없습니다.</b><p>120개 STEP 부품의 유효성과 주먹 모양은 확인되지만, 핀 구멍 막힘·회전 부품 간섭·끊어진 연결·체결 및 구동 누락이 남아 있습니다. 제작용 설계로 진행하기 전에 뿌리 관절 모듈을 실제 조립 구조로 완성해야 합니다.</p></div>
<h2>실제 CAD 단면에서 드러난 문제</h2><img src="data:image/png;base64,{image}" alt="핀 구멍 막힘, 회전 받침 간섭, 지지대 연결 누락의 실제 STEP 단면">
<h2>검사 범위와 수치</h2><p>손120개 부품을 빠짐없이 넣고 서로 다른 파일 사이의 체적 교차를 검사했습니다. 0.02mm³ 초과만 집계했습니다. 같은 시뮬레이터 강체에 배정된 부품도 제외하지 않았습니다.</p>
<table><tr><th>자세</th><th>체적 중복 부품 쌍</th><th>그중 서로 다르게 움직이는 강체 사이</th></tr>{table}</table>
<p>같은 강체 내부 중복은 일체 제작/접착/별도 조립 의도를 정해야 합니다. 서로 다르게 움직여야 하는 부품의 체적 중복은 실제 운동을 막습니다. 이전13자세 검사는 외피 일부만 대상으로 했기 때문에 이번 결과와 검사 범위가 다릅니다.</p>
<p>STEP:120/120 유효한 단일 솔리드. 벽 두께:{summary['wall_parts']}부품·{summary['wall_samples']}개 위치 표본. 가장 얇은 구조부 표본:{thinnest['sampled_min_mm']:.4f}mm. 벽 두께는 면적0.25mm² 이상인 면의5개 UV 위치에서 측정한 국소값이며 전역 최소 또는 강도 판정이 아닙니다.</p>
<h2>제작을 막는 항목과 수정 순서</h2><table class="findings"><tr><th>분류</th><th>확인된 근거와 원인</th><th>필요한 수정</th></tr>{rows}</table>
<h2>다음 설계의 완료 조건</h2><ol><li>뿌리2축 모듈 한 개에 실제 핀/부싱/와셔/유지부와 손바닥 체결까지 포함하고 조립 순서를 증명합니다.</li><li>구멍을 막는 지지살, 움직이는 부품의 중복, 끊어진 연결을 제거하고 공차가 반영된 여유를 확인합니다.</li><li>확정한 공정으로 소형 끼워맞춤 시험편과 관절 모듈을 검사한 뒤 나머지 손가락으로 확장합니다.</li><li>수정된 엄지 축·가동 범위로 실제 전달계와 하중/관성을 연결하고, 전체 손의 연속 운동·강도·구동 성능을 별도로 검증합니다.</li></ol>
<h2>공정·공차 판단의 참고 근거</h2><p>제작 공정은 아직 지정되지 않았습니다. <a href="https://help.prusa3d.com/article/modeling-with-3d-printing-in-mind_164135">Prusa 제조사 지침</a>은 FDM 치수 여유가 형상·방향·재료·장비에 따라 달라지고 시험 출력이 필요하다고 설명합니다. 예시0.4mm 노즐의 한 줄 폭은약0.45mm입니다. 이는 이 손의 공정을 확정하거나 모든 출력 방식에 같은 최소치를 적용한 것이 아닙니다.</p>
<p><a href="https://blog.igus.eu/knowledge-base/how-is-a-iglidur-plain-bearing-fixed-in-place-in-a-bearing-housing/">igus 부싱 설치 설명</a>은 해당 제품의 하우징 끼워맞춤과 압입 유지가 설계의 일부임을 보여 줍니다. 이 CAD의 이름 없는 부싱에 제조사 공차를 자동 적용하지 않았습니다. 실제 품번과 재료·설치 방식을 정한 뒤 공차 체인을 작성해야 합니다.</p>
<h2>이번에 판정하지 않은 것</h2><p>검사 대상은 이 후보의 손120부품입니다. 참고용 팔과 별도 구동 CAD 전체의 제작성은 이번 판정에 포함하지 않습니다. 재료 강도·피로 수명·충격·마찰/마모·장시간 발열·실물 구동 지연, 모든 자유20축 조합 및 연속 경로의 무간섭은 승인하지 않았습니다. 조립이 성립하지 않는 현재 형상에 임의 고정 조건을 주는 강도해석으로 합격 수치를 만들지 않았습니다. 사용자 실시간 웹캠 추종 품질 역시 별도입니다.</p>
<small>계산 근거: assembly.json · interfaces.json · walls.json · mesh_cleanup.json · readiness.json. 저장소 cad/webcam_articulation_v1/generated/manufacturing_audit에 보존합니다.</small></html>'''
    (OUT/'manufacturability_review.html').write_text(document,encoding='utf-8')
    print(json.dumps(dict(verdict=summary['verdict'],cases=cases,wall_samples=summary['wall_samples'],min_mm=thinnest['sampled_min_mm']),indent=2),flush=True)


if __name__=='__main__':main()

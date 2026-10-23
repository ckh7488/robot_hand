from pathlib import Path
D=Path(__file__).resolve().parent
source=(D/'render_candidate.py').read_text(encoding='utf-8').split("sheet('hand_redesign.png'")[0]
source=source.replace("for it in items:\n  if mode", "for it in items:\n  if mode in ['module','explode','mounted_cut'] and not (it['name'].startswith('index_root_') or it['name'] in ['index_spread_carrier','Index_root_cartridge','Index_root_mount_screws','Index_root_mount_nuts'] or (mode=='mounted_cut' and it['name']=='Display_palm_cutaway')):continue\n  if mode")
source=source.replace("p=np.array(it['pos_mm'])/1000;r=np.array(it['R']);", "p=np.array(it['pos_mm'])/1000;r=np.array(it['R']);\n  if it['name']=='Index_root_cartridge':it=it|{'color':[.18,.58,.61]}\n  if mode=='explode':\n   n=it['name']\n   if n=='Index_root_mount_screws':p[2]-=.015\n   elif n in ['index_root_frame','index_root_base_screws']:p[0]-=.015\n   elif n in ['index_root_motor','index_root_base_body_tap']:p[0]-=.040\n   elif n in ['index_root_rotor','index_root_hinge','index_root_output_screws','index_root_output_nuts','index_root_output_hinge_tap','index_spread_carrier']:p[0]-=.040;p[2]+=.030\n  ")
exec(compile(source,str(D/'render_candidate.py'),'exec'))
S['Open'].append({'name':'Display_palm_cutaway','asset':'Display_palm_cutaway','pos_mm':[0,0,0],'R':[[1,0,0],[0,1,0],[0,0,1]],'role':'printed','color':[.76,.80,.85]})
sheet('reviews/index_mount_cutaway.png','Open','mounted_cut','손바닥과 검지 모듈의 체결 단면','손바닥 일부를 잘라 내부를 보인 화면 / 맞춤턱에 앉힌 뒤 밑면 M3×12 두 개로 고정',[('체결 사선',125,-35),('밑면',270,75),('측면',180,0)])
sheet('reviews/index_cartridge_views.png','Open','module','검지 뿌리 · 분리형 고정 모듈 후보','청록색이 새 출력 부품 / 기존 관절축·손가락 길이 유지 · 손바닥 아래에서 M3×12 두 개로 체결',[('조립 상태',125,-35),('고정 발과 너트 입구',180,-10),('밑면 체결',270,75)])
sheet('reviews/index_cartridge_exploded.png','Open','explode','손바닥 밖에서 조립하는 검지 뿌리','출력 고정대 → S101 → 모터 → H101과 가동 브래킷 · 분해 표시 간격은 설명용',[('분해 사선',125,-35),('측면',180,0)])

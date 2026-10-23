import json, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt, numpy as np
from matplotlib import font_manager as fm
for f in fm.findSystemFonts():
    if 'NotoSansCJK-Regular' in f or 'NotoSansCJK-Medium' in f: fm.fontManager.addfont(f)
plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False, 'font.size': 10})
INK, INK2, MUTE, GRID, SURF = '#0b0b0b', '#52514e', '#8a8984', '#e4e3df', '#fcfcfb'
BLUE, ORANGE, GRAY = '#2a78d6', '#eb6834', '#c9c8c2'
fig = plt.figure(figsize=(15, 10.2), facecolor=SURF)
gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1], hspace=0.62, wspace=0.28, left=0.11, right=0.97, top=0.9, bottom=0.11)
fig.suptitle('V9 독립 감사 — 실제 CAD로 다시 계산한 핵심 수치 4개', x=0.02, ha='left', fontsize=16, color=INK, weight='bold')
fig.text(0.02, 0.935, 'Claude 독립 재계산 (FCL 실제 메시 거리 · 직접 작성한 순기구학, scene.json 6자세와 오차 6e-14 mm). 팔 정격은 CAD 에이전트 인용값(미확인).', fontsize=9.5, color=INK2)
# ---------- A: joint ranges
ax = fig.add_subplot(gs[:, 0]); ax.set_facecolor(SURF)
rows = [('검지 벌림', (-2, 9), (-3, 22), (0, 8), False), ('중지 벌림', (-5, 4), (-3, 3), (0, 2), False), ('약지 벌림', (-6, 5), (-4, 3), (-4, 0), False),
        ('소지 벌림', (-12, 6), (-22, 3), (-8, 0), False), ('MCP (4손가락)', (-24, 110), (-20, 85), (-20, 60), True), ('PIP (4손가락)', (-9, 113), (0, 100), (0, 45), False),
        ('DIP (4손가락)', (-9, 100), (0, 80), (0, 30), True), ('엄지 뿌리', (-2, 9), None, (-10, 0), False), ('엄지 대립', (-3, 113), None, (0, 20), False),
        ('엄지 MCP', (-9, 57), None, (5, 25), False), ('엄지 IP', (-9, 100), None, (5, 15), True), ('팔꿈치', (-20, 101), (0, 110), (0, 0), False)]
for k, (name, v9, old, ps, more) in enumerate(rows):
    y = len(rows) - 1 - k
    if old: ax.barh(y + 0.27, old[1] - old[0], left=old[0], height=0.2, color=GRAY, edgecolor=SURF, linewidth=1)
    ax.barh(y, v9[1] - v9[0], left=v9[0], height=0.3, color=BLUE, edgecolor=SURF, linewidth=1)
    ax.barh(y - 0.27, max(ps[1] - ps[0], 0.8), left=ps[0] - (0.4 if ps[1] == ps[0] else 0), height=0.16, color=ORANGE, edgecolor=SURF, linewidth=1)
    lab = f'{v9[0]}~{v9[1]}°' + ('+' if more else '')
    ax.text(v9[1] + 2, y, lab, va='center', fontsize=9, color=INK)
ax.set_yticks(range(len(rows))); ax.set_yticklabels([r[0] for r in rows][::-1], fontsize=10, color=INK)
ax.axvline(0, color=MUTE, lw=0.8); ax.set_xlim(-30, 135); ax.set_xlabel('관절각 (도)', color=INK2)
ax.grid(axis='x', color=GRID, lw=0.6); ax.set_axisbelow(True)
for s in ['top', 'right']: ax.spines[s].set_visible(False)
for s in ['left', 'bottom']: ax.spines[s].set_color(MUTE)
ax.set_title('① 한 축만 움직일 때 1 mm 여유가 유지되는 범위', loc='left', fontsize=12, color=INK, weight='bold')
fig.text(0.02, 0.012, '파랑 = V9 실제 CAD (펼침에서 한 축만, 1° 간격)   주황 = 6자세 검사에 쓰인 범위   회색 = 이전 프로젝트 목표 범위\n"+" = 찾은 끝까지 1 mm 이상.  벌림과 엄지 뿌리는 옆 손가락이 0°일 때 기준 — 동시에 움직이면 더 좁아짐 (②)', fontsize=8.6, color=INK2, va='bottom')
# ---------- B: simultaneous spread errors
b = fig.add_subplot(gs[0, 1]); b.set_facecolor(SURF)
cats = ['Backbend\n벌림 ±3°', 'Splay\n벌림 ±3°', 'Close\n벌림 ±3°', 'Backbend\nMCP ±3°', 'Splay\nMCP ±3°', 'Close\nMCP ±3°']
below = [67, 45, 0, 0, 0, 0]; contact = [26, 18, 0, 0, 0, 0]; x = np.arange(len(cats)); w = 0.36
b1 = b.bar(x - w / 2 - 0.01, below, w, color=BLUE, label='1 mm 미만'); b2 = b.bar(x + w / 2 + 0.01, contact, w, color=ORANGE, label='접촉 (0 mm)')
for xi, v in zip(x, below): b.text(xi - w / 2, v + 1.5, str(v), ha='center', fontsize=9, color=INK)
for xi, v in zip(x, contact): b.text(xi + w / 2, v + 1.5, str(v), ha='center', fontsize=9, color=INK)
b.set_xticks(x); b.set_xticklabels(cats, fontsize=8.6, color=INK); b.set_ylim(0, 81); b.set_ylabel('81개 조합 중 개수', color=INK2)
b.set_title('② 네 손가락에 동시에 −3/0/+3° 오차 (3⁴ = 81조합)', loc='left', fontsize=12, color=INK, weight='bold')
b.legend(frameon=False, fontsize=9, loc='upper right'); b.grid(axis='y', color=GRID, lw=0.6); b.set_axisbelow(True)
for s in ['top', 'right']: b.spines[s].set_visible(False)
# ---------- C: arm torque
c = fig.add_subplot(gs[1, 1]); c.set_facecolor(SURF)
J = ['팔꿈치', '전완 회전', '손목 굽힘', '손목 편위']; grav = [9.77, 2.53, 3.05, 2.79]; tot = [20.9, 4.13, 5.04, 4.63]; rated = [8.3, 3.0, 3.0, 2.5]
x = np.arange(4)
c.bar(x - w / 2 - 0.01, grav, w, color=BLUE, label='중력만 (최악 방향)'); c.bar(x + w / 2 + 0.01, tot, w, color=ORANGE, label='중력 + 90°를 0.6초에 (관성, 상한 추정)')
for xi, r in zip(x, rated):
    c.plot([xi - 0.42, xi + 0.42], [r, r], color=INK, lw=1.6); c.plot([xi - 0.42, xi + 0.42], [0.7 * r] * 2, color=INK, lw=1, ls=(0, (3, 2)))
for xi, v in zip(x, grav): c.text(xi - w / 2, max(v, 0) + (0.4 if v > 4 else 0.9), f'{v:g}', ha='center', fontsize=8.6, color=INK)
for xi, v in zip(x, tot): c.text(xi + w / 2, v + 0.4, f'{v:g}', ha='center', fontsize=8.6, color=INK)
c.set_xticks(x); c.set_xticklabels(J, color=INK); c.set_ylabel('N·m', color=INK2); c.set_ylim(0, 24)
c.set_title('③ 팔 모터: 필요 토크 vs 정격 (검은 실선) · 정격 70% (점선)', loc='left', fontsize=12, color=INK, weight='bold')
c.legend(frameon=False, fontsize=9, loc='upper right'); c.grid(axis='y', color=GRID, lw=0.6); c.set_axisbelow(True)
for s in ['top', 'right']: c.spines[s].set_visible(False)
# ---------- D: middle MCP
d = fig.add_subplot(gs[2, 1]); d.set_facecolor(SURF)
sc = ['그쪽 가정\n(점질량·정지토크 0.215)', '실제 메시 관성\n+ 로터 반사관성', '+ 전류 0.7 A 제한\n(0.102 N·m, 추정)']
ng = [0.210, 0.214, 0.282]; wg = [0.277, 0.282, 0.653]; x = np.arange(3)
d.bar(x - w / 2 - 0.01, ng, w, color=BLUE, label='중력 없음'); d.bar(x + w / 2 + 0.01, wg, w, color=ORANGE, label='최악 손 방향 중력')
for xi, v in zip(x, ng): d.text(xi - w / 2, v + 0.015, f'{v:.2f}', ha='center', fontsize=9, color=INK)
for xi, v in zip(x, wg): d.text(xi + w / 2, v + 0.015, f'{v:.2f}', ha='center', fontsize=9, color=INK)
d.set_xticks(x); d.set_xticklabels(sc, fontsize=8.8, color=INK); d.set_ylabel('초', color=INK2); d.set_ylim(0, 0.78)
d.set_title('④ 중지 MCP 90° 최소 시간 (낙관 근사)', loc='left', fontsize=12, color=INK, weight='bold')
d.legend(frameon=False, fontsize=9, loc='upper left'); d.grid(axis='y', color=GRID, lw=0.6); d.set_axisbelow(True)
for s in ['top', 'right']: d.spines[s].set_visible(False)
fig.savefig('/mnt/user-data/outputs/v9_audit_key_numbers.png', dpi=150, facecolor=SURF)

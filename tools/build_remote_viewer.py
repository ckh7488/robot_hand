"""Build a portable, offline trajectory viewer from saved simulation outputs.

This displays joint-center kinematics, not CAD collision or hardware evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument('--results', type=Path, default=ROOT/'sim/remote_drive_v1/results')
ap.add_argument('--output', type=Path)
a = ap.parse_args()
files = list(sorted(a.results.glob('*_trajectory.json')))
assert files, 'No trajectories found'
input_files = [*files, a.results/'report.json', ROOT/'cad/remote_drive_v1/design.json', Path(__file__).resolve()]
input_hashes = {f: hashlib.sha256(f.read_bytes()).hexdigest() for f in input_files}
all_traces = [json.loads(f.read_text(encoding='utf-8')) for f in files]
unique_traces = {}
for trace in all_traces:
    key = (trace['variant'], trace['case'])
    assert key not in unique_traces or unique_traces[key] == trace, f'Conflicting trajectory aliases: {key}'
    unique_traces[key] = trace
traces = list(unique_traces.values())
for trace in traces:
    n = len(trace['time_s'])
    assert n > 1 and len(trace['joint_names']) == 20
    assert len(trace['command_deg']) == len(trace['actual_deg']) == n
    assert all(len(row) == 20 for key in ['command_deg', 'actual_deg'] for row in trace[key])
    if 'requested_deg' in trace:
        assert len(trace['requested_deg']) == n and all(len(row) == 20 for row in trace['requested_deg'])
    else:
        trace['requested_deg'] = trace['command_deg']
report = json.loads((a.results/'report.json').read_text(encoding='utf-8'))
assert set(report['cases']) == {f"{t['variant']}/{t['case']}" for t in traces}, 'Report and display cases differ'
for relative, expected in report['source_hashes'].items():
    source = (ROOT/relative).resolve()
    assert source.is_relative_to(ROOT), relative
    assert hashlib.sha256(source.read_bytes()).hexdigest() == expected, f'Stale simulation source: {relative}'
    input_hashes[source] = expected
payload = dict(design=json.loads((ROOT/'cad/remote_drive_v1/design.json').read_text(encoding='utf-8')),
               traces=traces, cases=report['cases'], engine=report['engine'],
               repository_source_hashes={**report['source_hashes'], 'tools/build_remote_viewer.py': input_hashes[Path(__file__).resolve()]},
               source_hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in [*files, a.results/'report.json']})
HTML = r'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Hand · 추종 시뮬레이션</title>
<style>
:root{color-scheme:light;--ink:#152b38;--muted:#536a77;--paper:#f4f6f7;--line:#d7e0e5;--actual:#007f89;--target:#bd7621}*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:15px/1.55 system-ui,'Malgun Gothic',sans-serif}main{max-width:1320px;margin:auto;padding:32px}header{border-bottom:1px solid var(--line);padding-bottom:20px;margin-bottom:22px}.eyebrow{font-size:12px;letter-spacing:.12em;color:var(--actual);font-weight:750}h1{font-size:30px;line-height:1.2;margin:8px 0 12px}p{margin:8px 0;color:var(--muted)}.notice{padding:11px 16px;background:#fff3da;border:1px solid #e5ce99;border-radius:8px;color:#745521;font-size:13px}.controls{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin:20px 0}button,select{font:inherit;padding:8px 12px;border:1px solid var(--line);border-radius:7px;background:white;color:var(--ink)}button{cursor:pointer;background:var(--ink);color:white}input[type=range]{flex:1;min-width:160px;accent-color:var(--actual)}.time{font-variant-numeric:tabular-nums;min-width:70px}.grid{display:grid;grid-template-columns:1.15fr 1fr;gap:18px}.card{background:white;border:1px solid var(--line);border-radius:12px;padding:20px;overflow:hidden}h2{font-size:16px;margin:0 0 10px}.legend{display:flex;gap:20px;font-size:13px;margin-bottom:10px}.target{color:var(--target)}.actual{color:var(--actual)}.feasible{color:#765ab2}.legend{flex-wrap:wrap;gap:12px}.detail{background:#f5f7fa;border-radius:8px;padding:12px;font-size:12px;color:var(--muted)}canvas{width:100%;display:block}#hand{height:410px;cursor:grab}#plot{height:260px}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:18px 0}.metric{background:#edf3f5;padding:12px;border-radius:8px}.metric span{display:block;color:var(--muted);font-size:12px}.metric strong{font-size:23px;font-variant-numeric:tabular-nums}.small{font-size:12px;color:var(--muted)}.verdict{font-size:13px;color:#8b3f2a;margin:10px 0}table{border-collapse:collapse;width:100%;font-size:13px;margin-top:16px}th,td{border-bottom:1px solid var(--line);padding:6px;text-align:right;font-variant-numeric:tabular-nums}th:first-child,td:first-child{text-align:left}tbody tr{cursor:pointer}tbody tr.selected{background:#e3f1f1}footer{font-size:12px;color:var(--muted);margin-top:20px}@media(max-width:850px){main{padding:18px}.grid{grid-template-columns:1fr}h1{font-size:24px}#hand{height:340px}.metrics{gap:6px}.metric{padding:9px}.metric strong{font-size:19px}table{font-size:11px;table-layout:fixed}th,td{padding:5px 2px}th:first-child,td:first-child{width:30%;overflow-wrap:anywhere}}
</style><main><header><div class="eyebrow">ROBOT HAND / REMOTE DRIVE</div><h1>목표 움직임과 로봇의 추종 비교</h1><p>같은 요청 자세에 대해 기존 직결 구조와 모터 재배치 구조의 움직임을 비교합니다.</p><div class="notice">실측 전 모델입니다. 그림은 관절 중심을 연결한 표시이며, 실제 CAD 외피·줄 경로·충돌을 재현하지 않습니다. 손목·팔은 고정되어 있습니다.</div></header>
<div class="controls"><label>구조 <select id="variant"></select></label><label>동작 <select id="case"></select></label><button id="play">일시정지</button><label>재생 <select id="speed"><option value="0.25">0.25배</option><option value="0.5">0.5배</option><option value="1" selected>실시간</option></select></label><input id="time" type="range" min="0" step="1" value="0" aria-label="시뮬레이션 시간"><output class="time" id="stamp"></output><button id="reset">처음으로</button></div>
<div class="grid"><section class="card"><h2 id="motion-title">다섯 손가락</h2><div class="legend"><span class="target">┄ 요청 자세</span><span class="feasible">┄ 구조에 맞춘 구동 목표</span><span class="actual">━ 계산된 관절</span></div><canvas id="hand"></canvas><p class="small">드래그하여 시점을 바꿀 수 있습니다. 표시에는 100Hz로 추출한 각도를 사용합니다. 아래 지연은 원래 물리 계산 시간간격으로 평가한 결과입니다.</p><div class="metrics"><div class="metric"><span>도달한 사건 중 최대 지연</span><strong id="lag"></strong></div><div class="metric"><span>시차를 판별하지 못한 사건</span><strong id="missing"></strong></div><div class="metric"><span>요청 대비 최대 각도 오차</span><strong id="error"></strong></div></div><p class="verdict" id="verdict"></p><p class="detail" id="detail"></p><p class="small">시차를 판별하지 못한 사건과 목표를 놓친 사건은 구분합니다. 유효 표본만으로 100ms 이내라고 판정하지 않습니다. 16축은 같은 CAD 질량에서 끝마디를 연동한 가상 비교입니다.</p></section>
<section class="card"><h2>같은 시간축의 요청과 응답</h2><label>관절 <select id="joint"></select></label><canvas id="plot"></canvas><p class="small">시간축을 옮겨 맞추지 않은 원시 각도입니다. 표의 행을 눌러 관절을 선택할 수 있습니다.</p><table><thead><tr><th>관절</th><th>요청 °</th><th>구동 °</th><th>응답 °</th><th>오차 °</th></tr></thead><tbody id="rows"></tbody></table></section></div>
<footer id="source"></footer></main><script id="data" type="application/json">__PAYLOAD__</script><script>
'use strict';
const data=JSON.parse(document.getElementById('data').textContent),$=id=>document.getElementById(id),names={baseline20:'기존 직결 20축',remote20:'모터 재배치 20축',remote16:'16축 가상 연동 비교'};
const caseNames={small5:'작은 5° 왕복',small10:'작은 10° 왕복',large45:'45° 굽힘·펴기',fast90_150ms:'빠른 큰 움직임',upstream_hold:'뿌리 움직임과 끝마디 유지',rapid_reversal:'급반전',independent_distal:'가운데·끝마디 독립 움직임'};
let tr=data.traces.find(t=>t.variant==='remote20'&&t.case==='rapid_reversal')||data.traces[0],frame=0,joint=5,playing=true,elapsed=0,last=performance.now(),yaw=-.5,pitch=.45;
for(const v of new Set(data.traces.map(t=>t.variant)))$('variant').add(new Option(names[v]||v,v));$('variant').value=tr.variant;
function caseOptions(){const c=tr.case;$('case').replaceChildren();for(const t of data.traces.filter(t=>t.variant===tr.variant))$('case').add(new Option(caseNames[t.case]||t.case,t.case));$('case').value=c}caseOptions();
const labels=['벌림','뿌리 굽힘','가운데 굽힘','끝 굽힘'],fn=['검지','중지','약지','소지'];const jl=tr.joint_names.map((n,i)=>i<16?fn[Math.floor(i/4)]+' '+labels[i%4]:['엄지 뿌리 회전','엄지 대립','엄지 뿌리 굽힘','엄지 끝 굽힘'][i-16]);
jl.forEach((n,i)=>$('joint').add(new Option(n,i)));$('joint').value=joint;
function metrics(){
const m=data.cases[tr.variant+'/'+tr.case];
$('motion-title').textContent='다섯 손가락 · '+(caseNames[tr.case]||tr.case);
$('lag').textContent=m.valid_event_max_ms===null?'미확정':m.valid_event_max_ms.toFixed(1)+'ms';
$('missing').textContent=m.missing_events+' / '+m.event_count;
const taskMax=m.task_raw_max_error_deg??m.raw_max_error_deg;
$('error').textContent=taskMax.toFixed(1)+'°';
$('verdict').textContent=m.simulated_100ms_event_budget_met?'이 모델의 사건 지연 기준은 충족했습니다. 정확도·조립·실물 성능 통과를 뜻하지 않습니다.':'이 시험에서는 전체 사건의 100ms 지연 기준을 충족하지 못했습니다.';
const counts={};for(const e of m.events||[]){const c=e.classification||e.status;counts[c]=(counts[c]||0)+1}
const parts=[];
if(Object.hasOwn(counts,'preexisting_level'))parts.push('시작 때 이미 임계각을 지난 사건 '+counts.preexisting_level+'개');
if(Object.hasOwn(counts,'never_reached'))parts.push('구간 내 임계각 미도달 '+counts.never_reached+'개');
if(Object.hasOwn(counts,'early_crossing'))parts.push('일찍 통과한 사건 '+counts.early_crossing+'개');
parts.push('구동 목표 대비 최대 오차 '+m.raw_max_error_deg.toFixed(2)+'°');
if(m.projection_loss_max_deg!==undefined)parts.push('구조의 자세 표현 손실 최대 '+m.projection_loss_max_deg.toFixed(2)+'°');
if(m.unsettled_quality_holds!==undefined)parts.push('진폭 기준으로 정착하지 못한 구간 '+m.unsettled_quality_holds+'개');
if(m.max_endpoint_overshoot_deg!==undefined)parts.push('목표를 지나친 각도 최대 '+m.max_endpoint_overshoot_deg.toFixed(2)+'°');
$('detail').textContent=parts.join(' · ');
$('source').textContent=data.engine+' · '+tr.case+' · 입력장치 지연 제외 · 원본 결과: report.json / *_trajectory.json';
$('time').max=tr.time_s.length-1;
}
function resize(c){const d=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;if(c.width!==Math.round(w*d)||c.height!==Math.round(h*d)){c.width=Math.round(w*d);c.height=Math.round(h*d)}const ctx=c.getContext('2d');ctx.setTransform(d,0,0,d,0,0);ctx.clearRect(0,0,w,h);return[ctx,w,h]}
const mm=(a,b)=>[0,1,2].flatMap(i=>[0,1,2].map(j=>[0,1,2].reduce((s,k)=>s+a[i*3+k]*b[k*3+j],0)));
const mv=(a,b)=>[0,1,2].map(i=>a[i*3]*b[0]+a[i*3+1]*b[1]+a[i*3+2]*b[2]);const add=(a,b)=>a.map((x,i)=>x+b[i]);const rad=x=>x*Math.PI/180;
const rx=d=>{let c=Math.cos(rad(d)),s=Math.sin(rad(d));return[1,0,0,0,c,-s,0,s,c]},ry=d=>{let c=Math.cos(rad(d)),s=Math.sin(rad(d));return[c,0,s,0,1,0,-s,0,c]},rz=d=>{let c=Math.cos(rad(d)),s=Math.sin(rad(d));return[c,-s,0,s,c,0,0,0,1]};
function fk(q){return data.design.hand.fingers.map((f,i)=>{let p=f.root.slice(),R=mm(mm(rz(f.base_rpy[2]),ry(f.base_rpy[1])),rx(f.base_rpy[0])),ps=[p];R=mm(R,rz(-q[i*4]));p=add(p,mv(R,[0,0,data.design.hand.root_axis_offset_z]));ps.push(p);for(let k=0;k<3;k++){if(i===4&&k===1)R=mm(R,rz(f.mcp_mount_yaw));R=mm(R,rx(q[i*4+k+1]));p=add(p,mv(R,[0,f.links[k],0]));ps.push(p)}return ps})}
function hand(){const[c,w,h]=resize($('hand'));const sc=Math.min(w/480,h/440),cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);const project=p=>{let x=p[0],y=p[1]-160,z=p[2],u=cy*x+sy*z,v=-sy*x+cy*z;return[w/2+sc*u,h*.53-sc*(cp*y-sp*v)]};function line(ps,color,dashed,width){c.beginPath();ps.map(project).forEach(([x,y],i)=>i?c.lineTo(x,y):c.moveTo(x,y));c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dashed?[5,5]:[]);c.lineCap='round';c.stroke();c.setLineDash([])}line([[-116,0,-24],[-116,195,-24],[116,195,-24],[116,0,-24],[-116,0,-24]],'#c5d0d7',false,1.5);const target=fk(tr.requested_deg[frame]),feasible=fk(tr.command_deg[frame]),actual=fk(tr.actual_deg[frame]);target.forEach(ps=>line(ps,'#bd7621',true,3));if(tr.variant==='remote16')feasible.forEach(ps=>line(ps,'#765ab2',true,2));actual.forEach(ps=>{line(ps,'#007f89',false,5);ps.slice(1).forEach(p=>{const[x,y]=project(p);c.beginPath();c.arc(x,y,3.2,0,Math.PI*2);c.fillStyle='#164d53';c.fill()})});c.font='12px system-ui';c.fillStyle='#607581';c.fillText('고정 손바닥 / 관절 중심 표시',14,h-14)}
function plot(){const[c,w,h]=resize($('plot')),l=45,r=w-12,t=25,b=h-32,vals=tr.requested_deg.map(x=>x[joint]).concat(tr.command_deg.map(x=>x[joint]),tr.actual_deg.map(x=>x[joint]));let lo=Math.min(...vals)-5,hi=Math.max(...vals)+5;const x=i=>l+(r-l)*tr.time_s[i]/tr.time_s.at(-1),y=v=>b-(b-t)*(v-lo)/(hi-lo);c.font='11px system-ui';c.lineWidth=1;for(let i=0;i<=4;i++){let v=lo+(hi-lo)*i/4,Y=y(v);c.strokeStyle='#e6ecef';c.beginPath();c.moveTo(l,Y);c.lineTo(r,Y);c.stroke();c.fillStyle='#536a77';c.fillText(v.toFixed(0),5,Y+4)}for(let k=0;k<=4;k++){let X=l+(r-l)*k/4;c.fillText((tr.time_s.at(-1)*k/4).toFixed(2)+'s',X-12,h-10)}for(const[key,color,dash]of[['requested_deg','#bd7621',[6,4]],['command_deg','#765ab2',[2,3]],['actual_deg','#007f89',[]]]){c.strokeStyle=color;c.lineWidth=2;c.setLineDash(dash);c.beginPath();tr[key].forEach((v,i)=>i?c.lineTo(x(i),y(v[joint])):c.moveTo(x(i),y(v[joint])));c.stroke()}c.setLineDash([]);c.strokeStyle='#263e4b';c.beginPath();c.moveTo(x(frame),t);c.lineTo(x(frame),b);c.stroke()}
jl.forEach((label,i)=>{let row=document.createElement('tr');row.innerHTML='<td></td><td></td><td></td><td></td><td></td>';row.children[0].textContent=label;row.onclick=()=>{joint=i;$('joint').value=i;draw()};$('rows').appendChild(row)});
function draw(){hand();plot();$('time').value=frame;$('stamp').textContent=tr.time_s[frame].toFixed(2)+' s';[...$('rows').children].forEach((r,i)=>{const q=tr.requested_deg[frame][i],u=tr.command_deg[frame][i],a=tr.actual_deg[frame][i];r.className=i===joint?'selected':'';r.children[1].textContent=q.toFixed(1);r.children[2].textContent=u.toFixed(1);r.children[3].textContent=a.toFixed(1);r.children[4].textContent=(a-q).toFixed(1)})}
$('variant').onchange=()=>{tr=data.traces.find(t=>t.variant===$('variant').value&&t.case===tr.case)||data.traces.find(t=>t.variant===$('variant').value);caseOptions();elapsed=0;frame=0;metrics();draw()};$('case').onchange=()=>{tr=data.traces.find(t=>t.variant===$('variant').value&&t.case===$('case').value);elapsed=0;frame=0;metrics();draw()};$('joint').onchange=()=>{joint=+$('joint').value;draw()};$('play').onclick=()=>{playing=!playing;$('play').textContent=playing?'일시정지':'재생'};$('reset').onclick=()=>{elapsed=0;frame=0;draw()};$('time').oninput=()=>{frame=+$('time').value;elapsed=tr.time_s[frame];playing=false;$('play').textContent='재생';draw()};
let drag=null;$('hand').onpointerdown=e=>{drag=[e.clientX,e.clientY];$('hand').setPointerCapture(e.pointerId)};$('hand').onpointermove=e=>{if(drag){yaw+=(e.clientX-drag[0])*.01;pitch=Math.max(-1.2,Math.min(1.2,pitch+(e.clientY-drag[1])*.01));drag=[e.clientX,e.clientY];draw()}};$('hand').onpointerup=()=>drag=null;
function tick(now){if(playing){elapsed+=Math.max(0,(now-last)/1000)*+$('speed').value;elapsed%=tr.time_s.at(-1);frame=Math.max(0,Math.min(tr.time_s.length-1,Math.round(elapsed/(tr.time_s[1]-tr.time_s[0]))));draw()}last=now;requestAnimationFrame(tick)}window.onresize=draw;metrics();draw();requestAnimationFrame(tick);
</script></html>'''
text = HTML.replace('__PAYLOAD__', json.dumps(payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False).replace('</', '<\\/'))
out = a.output or a.results/'viewer.html'
out.parent.mkdir(parents=True, exist_ok=True)
for source, expected in input_hashes.items():
    assert hashlib.sha256(source.read_bytes()).hexdigest() == expected, f'Input changed while building viewer: {source}'
out.write_text(text, encoding='utf-8')
print(json.dumps({'output':str(out),'trajectories':len(traces),'display':'joint-center FK; no collision or hardware validation'}))

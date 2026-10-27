"""Saved actual hardware views and actual common material, never installed evidence."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm065_render',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
r64=n.m.drive.module('_arm065_renderer',ROOT/'cad/roll_bolted_interface_v1/render_evidence.py')
post,r,engine=r64.post,r64.r,r64.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['input_hash_guard']
    base,delta,reloc,clock,shift,moved=r64.prev.metadata()
    first=n.p.read(ROOT/'cad/recessed_mounting_root_v1/reviews/check.json');sel=n.p.read(ROOT/'cad/recessed_mounting_root_v1/reviews/selected_check.json')
    old064=n.p.read(ROOT/'cad/roll_bolted_interface_v1/reviews/check.json')
    before=[z for z in base if z['name'] not in a.p.a.OLD]+[z for z in n.p.read(ROOT/first['scene'])['parts'] if z['name']!=a.p.a.CARRIER]+n.p.read(ROOT/sel['scene'])['parts']
    before=[z for z in before if z['name'] not in a.p.OLD]+n.p.read(ROOT/old064['scene'])['parts']
    new=n.p.read(ROOT/raw['scene'])['parts'];after=[z for z in before if z['name'] not in a.OLD]+new
    assert len(before)==888 and len(after)==886
    source={}
    def matrix(name):
        M=np.array(delta.get(name,np.eye(4)))
        if name in reloc:M=reloc[name]@M
        if name in post.a.a.CLOCK_NAMES:M=clock@M
        if name in moved:M=shift@M
        return M
    def scene(items):
        parts=[]
        for z in items:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=matrix(z['name']),color=z.get('color',[.3,.5,.75])))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def fit(ss,vec):return r.view_fit(ss,[dict(angles_deg=[0.]*20)],np.array(vec),1.)
    images=HERE/'images';images.mkdir(exist_ok=True)
    ss=[scene(before),scene(after)];v=fit(ss,[.8,-.55,.4])
    r.sheet([(label,render(s,v,1050)) for label,s in zip(['ARM064 separate888','ARM065 shared-root886 /unadopted'],ss)],2,1050,
        '실제 CAD 전후,손/16모터/빔/전기 유지','현재862설치교체0 /8기능부품을6행으로공동화 /모터삭제0 /전체토크·외형승인 아님').save(images/'before_after.png')
    beam=[z for z in before if z['name']=='Forearm_box_beam']
    bolts=[z for z in before if z['name'].startswith(('arm064_clamp_M3x12_','arm064_clamp_nut_'))]
    ss=[scene([z for z in before if z['name'] in a.OLD]+bolts+beam),scene(new+bolts+beam)];v=fit(ss,[.5,-.7,.65])
    r.sheet([(label,render(s,v,1050)) for label,s in zip(['기존뒤collar/rails + 별도출력clamp','공동출력판/뒤root/두나사위치cap'],ss)],2,1050,
        '뒤 고정 전체를 공동 반력 구조로 대체','새rearroot는출력면Y-230기준 /기존앞unit유지 /뒤나사4형상유지X+-31->+-25.5명시 /축뽑힘마찰미검증').save(images/'shared_unit.png')
    e64=n.p.read(ROOT/old064['entry_scene'])['parts'];entries=[z for z in e64 if z['name'].startswith('arm064_clamp_')]+n.p.read(ROOT/raw['entry_scene'])['parts']
    s=scene(new+bolts+beam+entries)
    r.sheet([('공동unit/두나사위치/20mm진입참고8',render(s,fit([s],[.65,-.75,.65]),1100))],1,1100,
        '전체 클램프와 선언한 나사 진입','진입8은선정공구/handle아님 /기존출력나사6진입관계도검사 /실제압착·강도·조립승인 아님').save(images/'entry_and_root.png')
    records=sorted([z for z in raw['native']['new6__retained880']['overlaps'] if z['part']!='AK_forearm_roll'],key=lambda z:z['material_intersection_mm3'],reverse=True)[:2]
    world,membership,loaded=a.context();native=n.c.load(new,source);lookup={z['name']:z for z in world+native}
    queries=[];commons=[]
    for i,q in enumerate(records):
        sh=lookup[q['lead']]['shape'].intersect(lookup[q['part']]['shape'],tol=1e-6);assert sh.isValid() and sh.Solids()
        commons.append(dict(name='arm065_actual_common_'+str(i),shape=sh,body='forearm',color=[.86,.13,.12],role='actual new hardware obstruction evidence,not installed'))
        queries.append(dict(lead=q['lead'],part=q['part'],volume_mm3=sh.Volume(),solids=len(sh.Solids())))
    common_scene=n.a.a.prior.export(commons,HERE/'generated/actual_common') if commons else None
    if commons:
        panels=[]
        for z,q in zip(n.p.read(ROOT/common_scene)['parts'],queries):
            s=scene([z]);panels.append((q['part'],render(s,fit([s],[.65,-.75,.65]),950)))
        r.sheet(panels,2,950,'새 공동 구조의 실제 겹친 재료,각각 확대','전체부품을빨갛게칠한그림아님 /첫실패보존 /작은코너절삭이나각도제한으로성공처리하지않음').save(images/'actual_common.png')
    else:
        ss=[scene([z for z in after if z['name'] not in raw['assembly_initial_absent_names']]),scene(after)];v=fit(ss,[.65,-.75,.65])
        r.sheet([(label,render(s,v,1050)) for label,s in zip(['조건부시작 /cap와체결8없음','최종공동unit /수동유지조건'],ss)],2,1050,
            '실행한 조립 조건의 시작과 끝','유한표본검사범위는check.json /공급공구·연속조립·토크승인 아님').save(images/'assembly_conditions.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        common_queries=queries,common_scene=common_scene,new_common_queries=len(queries),native_groups_repeated=0,opaque=True,
        whole_hardware_display_counts=[888,886],input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

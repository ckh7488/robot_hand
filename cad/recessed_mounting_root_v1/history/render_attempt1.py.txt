"""Opaque actual before/after geometry and preserved first-candidate failures."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm063_render',Path(__file__).with_name('selected_model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
post=n.m.drive.module('_arm063_render_post',ROOT/'cad/integrated_motor_end_v1/postprocess.py')
r,engine=post.r,post.engine


def metadata():
    raw=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    _,_,meta,membership,delta,reloc,clock,shift=post.metadata(raw)
    stage060=n.p.read(ROOT/'cad/rotation_reserved_support_v1/reviews/check.json')
    stage061=n.p.read(ROOT/'cad/central_reaction_frame_v1/reviews/check.json')
    before=[z for z in meta if z['name'] not in a.base.p.p.p.OLD]+n.p.read(ROOT/stage060['scene'])['parts']
    before=[z for z in before if z['name']!=a.base.p.p.OLD]+n.p.read(ROOT/stage061['scene'])['parts']
    assert len(before)==871
    return before,delta,reloc,clock,shift,set(raw['design']['moved_names'])


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    first=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json');assert selected['completed']
    before,delta,reloc,clock,shift,moved=metadata();source={}
    initial=n.p.read(ROOT/first['scene'])['parts'];fresh=n.p.read(ROOT/selected['scene'])['parts']
    after=[z for z in before if z['name'] not in a.OLD]+[z for z in initial if z['name']!=a.CARRIER]+fresh
    assert len(after)==871
    entries=n.p.read(ROOT/first['entry_scene'])['parts']
    def matrix(name):
        M=np.array(delta.get(name,np.eye(4)))
        if name in reloc:M=reloc[name]@M
        if name in post.a.a.CLOCK_NAMES:M=clock@M
        if name in moved:M=shift@M
        return M
    def scene(items):
        parts=[]
        for z in items:
            name=z['name'];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=matrix(name),color=z.get('color',[.3,.5,.75])))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,view,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':view},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def fit(ss,vec):return r.view_fit(ss,[dict(angles_deg=[0.]*20)],np.array(vec),1.)
    images=HERE/'images';images.mkdir(exist_ok=True)
    ss=[scene(before),scene(after)];view=fit(ss,[.8,-.55,.4])
    r.sheet([(label,render(s,view,1050)) for label,s in zip(['이전 ARM061 /871전부','교정 whole-clamp/root /871전부'],ss)],2,1050,
        '실제 CAD 전후 /손·16원격모터·전기 위치 유지',
        '현재862설치교체0 /별도871미채택 /새6기능부품중4체결품은형상유지·축이동명시 /전체구동승인 아님').save(images/'before_after.png')
    beam=[z for z in before if z['name']=='Forearm_box_beam']
    oldunit=[z for z in before if z['name'] in a.OLD]+beam
    newunit=fresh+[z for z in initial if z['name']!=a.CARRIER]+beam
    ss=[scene(oldunit),scene(newunit)];view=fit(ss,[.5,-.7,.6])
    r.sheet([(label,render(s,view,1050)) for label,s in zip(['기존 ears/외측rail/중앙연결살','새 내부체결좌석/내측rail/앞면pickup'],ss)],2,1050,
        '앞쪽 체결 전체와 연결된 반력 경로',
        'M3축X+-31 -> +-25.5 /lower·cap함께교체 /머리·너트공간+앞면연결 /빔절삭0 /강도·예압승인 아님').save(images/'whole_unit_path.png')
    ss=[scene(initial+beam),scene(newunit+entries)];view=fit(ss,[.65,-.75,.7])
    r.sheet([(label,render(s,view,1050)) for label,s in zip(['보존한첫후보 /r44살이모터·cap침범','교정후보 /원래spoke범위·앞면살/진입참고4'],ss)],2,1050,
        '첫 후보의 실패를 보존하고 지지 방식 교정',
        '바뀐carrier관계만새17검사 /기존cap·체결5와진입4동일 /선정공구·조립검사 아님').save(images/'root_correction.png')
    s=scene(newunit+entries+n.p.read(ROOT/first['protection_input_scene'])['parts'][:9])
    r.sheet([('교정unit6+진입4+기존9보수회전공간',render(s,fit([s],[.65,-.75,.7]),1100))],1,1100,
        '전체 체결과 기존 회전 보호공간의 새 관계',
        '보수원통은설치물아님 /대표출력9관련공간만 /다른모터전체명령·긴재료운동승인 아님').save(images/'reserved_spaces.png')
    # Preserve actual first-candidate common material,not whole red parts.
    firstnative=n.c.load(initial,source);lookup={z['name']:z for z in firstnative}
    carrier=lookup[a.CARRIER]['shape'];rotor_record=first['native']['new_whole_unit6__retained865_neutral']['overlaps'][0]
    meta=next(z for z in before if z['name']==rotor_record['part'])
    rotor=n.c.load([meta],source)[0];rotor['shape']=n.g.original.transform(rotor['shape'],matrix(rotor['name']))
    queries=[];common=[]
    for label,right in [('first_rotor',rotor['shape']),('first_cap',lookup[a.CAP]['shape'])]:
        q=carrier.intersect(right,tol=1e-6);assert q.isValid() and len(q.Solids())==1
        queries.append(dict(id=label,volume_mm3=q.Volume(),solids=len(q.Solids())))
        common.append(dict(name='arm063_'+label+'_common',shape=q,body='forearm',color=[.85,.1,.1],role='preserved_initial_failure_evidence_not_installed'))
    common_scene=n.a.a.prior.export(common,HERE/'generated/first_common')
    panels=[]
    for z,label in zip(n.p.read(ROOT/common_scene)['parts'],['첫후보 실제rotor공통,현재살아님','첫후보 실제cap공통,현재살아님']):
        s=scene([z]);panels.append((label,render(s,fit([s],[.65,-.75,.7]),950)))
    r.sheet(panels,2,950,'보존한 첫 후보의 실제 공통재료,각기 다른 확대',
        '실패를삭제/이름변경성공처리하지않음 /설치물0 /같은미세코너정밀반복 아님').save(images/'first_failure_common.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'selected_model.py',HERE/'reviews/check.json',HERE/'reviews/selected_check.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        first_common_scene=common_scene,first_common_queries=queries,common_source_queries=2,
        initial_rotor_record=rotor_record,native_groups_repeated=0,opaque=True,whole_hardware_display_counts=[871,871],
        pixels_matrices_armvisualunits_numerically_replayed=False,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

"""Actual CAD before/after and new mounting interfaces; no hardware regeneration."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm064_render',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
prev=n.m.drive.module('_arm064_renderer_prior',ROOT/'cad/recessed_mounting_root_v1/render_evidence.py')
post,r,engine=prev.post,prev.r,prev.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['input_hash_guard']
    base,delta,reloc,clock,shift,moved=prev.metadata();first=n.p.read(ROOT/'cad/recessed_mounting_root_v1/reviews/check.json')
    selected=n.p.read(ROOT/'cad/recessed_mounting_root_v1/reviews/selected_check.json')
    before=[z for z in base if z['name'] not in a.a.OLD]+[z for z in n.p.read(ROOT/first['scene'])['parts'] if z['name']!=a.a.CARRIER]+n.p.read(ROOT/selected['scene'])['parts']
    new=n.p.read(ROOT/raw['scene'])['parts'];after=[z for z in before if z['name'] not in a.OLD]+new
    assert len(before)==871 and len(after)==888
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
    ss=[scene(before),scene(after)];v=fit(ss,[.8,-.55,.4])
    r.sheet([(label,render(s,v,1050)) for label,s in zip(['ARM063 /871 actual rows','ARM064 /888 separate proposal'],ss)],2,1050,
        '실제 CAD 전후,모터/손/빔 위치 유지',
        '현재862설치교체0 /2부품대체+새19 /국부체결후보 /랙분포·전체구동·외형 미승인').save(images/'before_after.png')
    beam=[z for z in before if z['name']=='Forearm_box_beam']
    vendor=n.p.read(ROOT/'cad/mounting_unit_interface_basis_v1/generated/vendor_reference/scene.json')['parts']
    ss=[scene([z for z in before if z['name'] in a.OLD]+beam+vendor),scene(new+beam+vendor)];v=fit(ss,[.65,-.6,.6])
    r.sheet([(label,render(s,v,1050)) for label,s in zip(['기존 빈 출력판/뒤받침과 공식표면 참고','새6post/E고정 + 출력판/사각clamp/EF'],ss)],2,1050,
        '실제 출력·stator 나사면을 잇는 후보',
        '공식열린표면은기존ARM062의참고등록 /6M3x25·6M3x8·clamp2 /실제장착clock·토크승인 아님').save(images/'mounting_interfaces.png')
    entries=n.p.read(ROOT/raw['entry_scene'])['parts'];s=scene(new+entries+beam)
    r.sheet([('새19하드웨어/진입참고16/유지beam',render(s,fit([s],[.6,-.8,.55]),1100))],1,1100,
        '고정·회전 몸체와 선언한 체결 진입',
        'E:stator6post /EF:출력판·사각beamclamp /20mm진입은공급socket/handle아님 /축방향pullout마찰미검증').save(images/'entry_and_capture.png')
    physical=sorted([z for z in raw['native']['new19__retained869']['overlaps'] if z['part']!='AK_forearm_roll'],key=lambda z:z['material_intersection_mm3'],reverse=True)[:1]
    approach=sorted(raw['native']['entry16__retained869']['overlaps'],key=lambda z:z['material_intersection_mm3'],reverse=True)[:1]
    records=physical+approach
    world,membership,loaded=a.context();lookup={z['name']:z for z in world+n.c.load(new+entries,source)}
    commons=[];queries=[]
    for i,q in enumerate(records):
        sh=lookup[q['lead']]['shape'].intersect(lookup[q['part']]['shape'],tol=1e-6);assert sh.isValid() and sh.Solids()
        commons.append(dict(name='arm064_obstruction_common_'+str(i),shape=sh,body='forearm',color=[.86,.13,.12],role='actual common evidence,not installed'))
        queries.append(dict(lead=q['lead'],part=q['part'],volume_mm3=sh.Volume(),solids=len(sh.Solids()),
            kind='actual_hardware_common' if i==0 else 'declared_entry_reference_common_not_selected_tool'))
    common_scene=n.a.a.prior.export(commons,HERE/'generated/actual_common') if commons else None
    if commons:
        panels=[]
        for z,q in zip(n.p.read(ROOT/common_scene)['parts'],queries):
            s=scene([z]);panels.append((q['kind'],render(s,fit([s],[.6,-.8,.55]),1000)))
        r.sheet(panels,2,1000,'유지 지지부와 새 체결부·진입참고의 공통,각각 확대',
            '왼쪽실제하드웨어 /오른쪽선언진입공간·공급공구아님 /명목모터원통교차와구별 /작은코너절삭반복0').save(images/'actual_obstructions.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        common_queries=queries,common_scene=common_scene,new_common_queries=len(queries),native_groups_repeated=0,opaque=True,
        whole_hardware_display_counts=[871,888],existing_vendor_surface_not_regenerated=True,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

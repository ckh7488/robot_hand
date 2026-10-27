"""Actual saved poses and two new common-material queries,not installed parts."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm066_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm066_renderer',ROOT/'cad/shared_roll_root_v1/render_evidence.py')
r,engine=old.r,old.engine


def metadata():
    base,delta,reloc,clock,shift,moved=old.r64.prev.metadata()
    first=n.p.read(ROOT/'cad/recessed_mounting_root_v1/reviews/check.json');sel=n.p.read(ROOT/'cad/recessed_mounting_root_v1/reviews/selected_check.json')
    q64=n.p.read(ROOT/'cad/roll_bolted_interface_v1/reviews/check.json');q65=n.p.read(ROOT/'cad/shared_roll_root_v1/reviews/check.json')
    rows=[z for z in base if z['name'] not in m.a.p.a.OLD]+[z for z in n.p.read(ROOT/first['scene'])['parts'] if z['name']!=m.a.p.a.CARRIER]+n.p.read(ROOT/sel['scene'])['parts']
    rows=[z for z in rows if z['name'] not in m.a.p.OLD]+n.p.read(ROOT/q64['scene'])['parts']
    rows=[z for z in rows if z['name'] not in m.a.OLD]+n.p.read(ROOT/q65['scene'])['parts'];assert len(rows)==886
    def matrix(name):
        M=np.array(delta.get(name,np.eye(4)))
        if name in reloc:M=reloc[name]@M
        if name in old.post.a.a.CLOCK_NAMES:M=clock@M
        if name in moved:M=shift@M
        return M
    return rows,matrix


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    first=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json')
    assert first['completed'] and selected['completed']
    rows,_,source=m.context();lookup={z['name']:z for z in rows};meta,matrix=metadata();meta={z['name']:z for z in meta}
    queries=[];commons=[]
    for i,raw in enumerate([first,selected]):
        key=raw['first_blocked_sample'];path=next(p for p in raw['paths'] if any(s['id']==key for s in p['samples']))
        sample=next(z for z in path['samples'] if z['id']==key);record=max(raw['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3'])
        M=m.translation(sample['translation_mm']);sh=n.g.original.transform(lookup[record['lead']]['shape'],M).intersect(lookup[record['part']]['shape'],tol=1e-6)
        assert sh.isValid() and sh.Solids()
        name='arm066_failed_entry_common_'+str(i)
        commons.append(dict(name=name,shape=sh,body='forearm',color=[.9,.12,.1],role='failed assembly common evidence,not installed'))
        queries.append(dict(name=name,id=key,lead=record['lead'],part=record['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids())))
    common_scene=n.a.a.prior.export(commons,HERE/'generated/actual_common');commonmeta=n.p.read(ROOT/common_scene)['parts']
    def scene(names,translations=None,extra=None):
        parts=[];translations=translations or {}
        for name in names:
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            M=m.translation(translations.get(name,[0.,0.,0.]))@matrix(name)
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=M,color=z.get('color',[.3,.5,.75])))
        for z in extra or []:
            path=ROOT/z['mesh'];source[z['mesh']]=r.sha(path)
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.eye(4),color=[.9,.12,.1]))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def sheet(panels,cols,size,title,footer,file):
        v=r.view_fit([z[1] for z in panels],[dict(angles_deg=[0.]*20)],np.array([.55,-.7,.65]),1.)
        r.sheet([(label,render(s,v,size)) for label,s in panels],cols,size,title,footer).save(HERE/'images'/file)
    (HERE/'images').mkdir(exist_ok=True)
    root=m.a.NAME;beam='Forearm_box_beam';cap=m.a.CAP;front='arm063_recessed_front_cap'
    sheet([('빔 +Y120 시작',scene([root,beam],{beam:[0,120,0]})),('빔 원래 위치,7표본 통과',scene([root,beam])),
        ('뒤덮개 +Z90 시작',scene([root,beam,cap],{cap:[0,0,90]})),('뒤덮개 착좌,8표본 통과',scene([root,beam,cap]))],2,850,
        '형상 변경 없는 빔·뒤덮개 벤치 선조립','조건부 root 시작 /모터·손·나머지 부속 아직 없음 /유한표본,체결 성공 아님','bench_steps.png')
    nuts=[k for k in selected['bench_names'] if k.startswith('arm063_') and 'nut' in k]
    group=[front]+nuts
    sheet([('첫 수직 진입 Z+50,실패',scene([root,beam,cap,front],{front:[0,0,50]})),
        ('너트 먼저,빈 구간 Y-190으로 내림',scene([root,beam,cap]+group,{k:[0,-27.5,0] for k in group})),
        ('앞으로27.5 이동 후 원래 위치',scene([root,beam,cap]+group))],3,850,
        '앞 덮개 진입 순서 비교','수직 첫 실패 보존 /새 순서의 실제 완료 범위는 selected_check /전체22부품 체결 성공 아님','front_cap_order.png')
    panels=[]
    for raw,q in zip([first,selected],queries):
        p=next(p for p in raw['paths'] if any(s['id']==q['id'] for s in p['samples']));s=next(s for s in p['samples'] if s['id']==q['id'])
        panels.append((q['id'],scene(p['present_names']+p['moving_names'],{k:s['translation_mm'] for k in p['moving_names']})))
    sheet(panels,2,1000,'실제 조립 진입에서 막힌 두 자세','재료를 깎거나 모터를 없애지 않음 /두 조건의 첫 실패 /후속 나사·모터 복구 미실행','failed_entries.png')
    panels=[]
    for z,q in zip(commonmeta,queries):
        s=scene([],extra=[z]);v=r.view_fit([s],[dict(angles_deg=[0.]*20)],np.array([.55,-.7,.65]),1.)
        panels.append((q['id'],render(s,v,900)))
    r.sheet(panels,2,900,'실제 겹친 재료,각각 확대','새 공통 질의2 /원래 경로 재실행0 /증거는 설치 부품 아님').save(HERE/'images/actual_common.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/selected_check.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=2,
        native_path_groups_repeated=0,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},
        opaque=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

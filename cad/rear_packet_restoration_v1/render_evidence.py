"""Actual saved entry poses and bounded new axis/common evidence."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm069_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm069_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();raw=n.p.read(HERE/'reviews/check.json');assert raw['completed']
    source=dict(raw['source_sha256']);meta,matrix=old.metadata();meta={z['name']:z for z in meta};axis_reads=[]
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        sh=n.c.load([z],source)[0]['shape'];return n.g.original.transform(sh,matrix(name))
    for c in raw['module_contracts']:
        for suffix,expected in [('_base_screws',c['radial']),('_rack_screw_1',c['normal'])]:
            name=c['module']+suffix;sh=native(name);cyl=[]
            for face in sh.Faces():
                if face.geomType()=='CYLINDER':
                    cy=BRepAdaptor_Surface(face.wrapped).Cylinder();axis=np.array(cy.Axis().Direction().Coord())
                    cyl.append(dict(radius_mm=cy.Radius(),axis=axis,point=list(cy.Axis().Location().Coord()),axis_parallel=bool(abs(axis@np.array(expected))>1.-1e-10)))
            axis_reads.append(dict(name=name,expected_axis=expected,cylinders=cyl,all_cylinder_axes_parallel=bool(cyl and all(z['axis_parallel'] for z in cyl))))
    queries=[];common_scene=None;commonmeta=[]
    if raw.get('first_blocked_sample'):
        key=raw['first_blocked_sample'];p=next(p for p in raw['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in p['samples'] if s['id']==key)
        if raw['native'][key]['overlaps']:
            q=max(raw['native'][key]['overlaps'],key=lambda q:q['material_intersection_mm3']);M=np.array(s['transform'])
            sh=n.g.original.transform(native(q['lead']),M).intersect(native(q['part']),tol=1e-6)
            assert sh.isValid() and sh.Solids()
            name='arm069_failed_entry_common_0';queries=[dict(name=name,id=key,lead=q['lead'],part=q['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=q['material_intersection_mm3'])]
            n.a.save(HERE/'reviews/common_prepared.json',dict(completed=False,common_queries=queries,new_common_queries=1,common_scene=None))
            common_scene=n.a.a.prior.export([dict(name=name,shape=sh,body='forearm',evidence_only=True,color=[.9,.1,.08])],HERE/'generated/actual_common')
            commonmeta=n.p.read(ROOT/common_scene)['parts']
            n.a.save(HERE/'reviews/common_prepared.json',dict(completed=True,common_queries=queries,new_common_queries=1,common_scene=common_scene))
    def scene(names,poses=None,extras=None,bright=None):
        parts=[];poses=poses or {};bright=set(bright or [])
        for name in sorted(set(names)):
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@matrix(name),
                color=[.95,.55,.1] if name in bright else z.get('color',[.3,.5,.75])))
        for z in extras or []:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.eye(4),color=[.9,.1,.08]))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def sheet(panels,cols,size,title,foot,file,direction=[.55,-.7,.65]):
        v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],np.array(direction),1.)
        r.sheet([(label,render(s,v,size)) for label,s in panels],cols,size,title,foot).save(HERE/'images'/file)
    (HERE/'images').mkdir(exist_ok=True)
    prior=n.p.read(ROOT/'cad/captive_module_assembly_v1/reviews/selected_check.json');bench=prior['bench_names'];packet_names={k for p in raw['original16packets'] for k in p['names']}
    before=bench+sorted(set(raw['initial_world_present_names'])&packet_names);after=bench+sorted(set(raw['packet_final_present_names'])&packet_names)
    sheet([('이번 시작,완료1/16 묶음',scene(before)),('실제 마지막,완료'+str(len(raw['restored_modules']))+'/16 묶음',scene(after))],2,1100,
        '원래 설치위치와 부품을 유지한 뒤쪽 모터 복구','비packet478 조립가정 /다른 부품은 보기 생략 /현재862 교체·방사랙 외형채택 아님','restoration_before_after.png')
    first=raw['module_contracts'][0];paths=[p for p in raw['paths'] if p['module']==first['module'] and p['stage']!='world'];panels=[]
    def vector(v):return '('+','.join(format(float(x),'.1f') for x in v)+')'
    labels={'M2_screws_before_motor':'M2나사 먼저','motor_reference_into_cradle':'모터 묶음 착좌','motor_and_own_leads_into_carrier':'자기 전선과 함께'}
    for p in paths:
        for s in p['samples'][-2:]:
            panels.append((labels[p['id'].split('__')[1]]+' / '+vector(s['translation_mm']),scene(p['present_names']+p['moving_names'],{k:s['transform'] for k in p['moving_names']},bright=p['moving_names'])))
    sheet(panels,2,850,'실제 방위에서의 M2 선조립과 자기 전선 동반 진입','각 모듈의 새 관계 검사 /공급모터·전기·출력 내부는 참고묶음 /지그·실제체결 미검증','bench_order.png')
    body=next((p for p in raw['paths'] if p['stage']=='world' and '__body19_' in p['id']),None)
    if body:
        local=bench+sorted(set(body['present_names'])&packet_names)+body['moving_names']
        sheet([(label,scene(local,{k:s['transform'] for k in body['moving_names']},bright=body['moving_names'])) for label,s in [('바깥 시작',body['samples'][0]),('실제 실행 마지막',body['samples'][-1])]],2,1050,
            '첫 추가 묶음의 전체 바깥 시작과 실제 도달 상태','다른 부품은 보기 생략 /native검사는 해당 시점 전체 장애물 /최종표본만으로 연속운동 승인 아님','body_entry.png')
    if queries:
        q=queries[0];pair=scene([q['lead'],q['part']],{q['lead']:q['lead_transform']},bright=[q['lead']]);red=scene([],extras=commonmeta)
        vp=r.view_fit([pair],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);vr=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.)
        r.sheet([('막힌 실제 두 부품',render(pair,vp,1100)),('같은 공통재료,별도 확대',render(red,vr,1100))],2,1100,
            '뒤쪽 복구에서 처음 막힌 위치','공통 '+format(q['volume_mm3'],'.9g')+' mm3 /정확 부품명은 check/evidence 기록 /작은 모서리 절삭0').save(HERE/'images/first_block.png')
    else:
        last=raw['module_contracts'][-1];wp=[p for p in raw['paths'] if p['module']==last['module'] and p['stage']=='world' and '__rack_M3_' in p['id']]
        panels=[]
        for p in wp:
            for s in [p['samples'][0],p['samples'][-1]]:
                local=bench+sorted(set(p['present_names'])&packet_names)+p['moving_names'];panels.append(('M3 실제진입 '+vector(s['translation_mm']),scene(local,{k:s['transform'] for k in p['moving_names']},bright=p['moving_names'])))
        if panels:sheet(panels,2,950,'마지막 뒤쪽 묶음의 랙 고정나사 진입','원래 M2받침너트는 뒤에 복구 /나사·모터 삭제0 /유한표본 검사','last_M3_entry.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),actual_axis_reads=axis_reads,
        actual_M2_M3_axes_match_declared_bases=all(z['all_cylinder_axes_parallel'] for z in axis_reads),native_path_groups_repeated=0,
        new_hardware_geometry=0,new_evidence_geometry=len(queries),output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},
        opaque=True,all_selected_geometry_inside_frame=True,view_omissions_not_test_exclusions=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('rendered',len(list((HERE/'images').glob('*.png'))),'common',len(queries),'axes',len(axis_reads),flush=True)


if __name__=='__main__':main()

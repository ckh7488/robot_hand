"""Saved original common read;new first failed guide entry and actual poses."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm070_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm070_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/continuation_check.json');first=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json');electrical_final=n.p.read(HERE/'reviews/final_check.json');assert all(z['completed'] for z in [raw,first,selected,electrical_final])
    source=dict(raw['source_sha256']);meta,matrix=old.metadata();meta={z['name']:z for z in meta}
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        sh=n.c.load([z],source)[0]['shape'];return n.g.original.transform(sh,matrix(name))
    prior_common=n.p.read(ROOT/'cad/rear_packet_restoration_v1/reviews/common_prepared.json')
    commonmeta=n.p.read(ROOT/prior_common['common_scene'])['parts'];saved_common=n.c.load(commonmeta,source)[0]['shape'];c=raw['module_contracts'][0]
    local=n.g.original.transform(saved_common,np.linalg.inv(np.array(c['guide_actual_frame'])))
    survey=dict(old_common_scene=prior_common['common_scene'],old_common_queries_repeated=0,stored_common_volume_mm3=saved_common.Volume(),
        stored_common_bbox_world=n.c.m.pack._bbox(saved_common),stored_common_bbox_in_actual_guide_frame=n.c.m.pack._bbox(local),
        original_floor_local_z_mm=c['original_guide']['groove_floor_z_mm'],actual_groove_opening_axis=c['guide_opening_axis'],actual_guide_frame=c['guide_actual_frame'],
        original_wire_paths=c['original_leads'],original_source_sequence='wire_guides_v1/service_check.py:insert guide/M3/spacers,lay wires,fit nuts/lid/M2screws. Old world pass not inherited.',
        axial_guide_attempt=first['first_blocked_sample'],axial_guide_attempt_failure=first['native'][first['first_blocked_sample']]['overlaps'])
    n.a.save(HERE/'reviews/survey.json',dict(completed=True,**survey))
    axes=[]
    for c in raw['module_contracts']:
        name=c['module']+'_lid_screw_1';cyl=[]
        for face in native(name).Faces():
            if face.geomType()=='CYLINDER':
                cy=BRepAdaptor_Surface(face.wrapped).Cylinder();axis=np.array(cy.Axis().Direction().Coord());cyl.append(dict(radius_mm=cy.Radius(),axis=axis,point=np.array(cy.Axis().Location().Coord())))
        axes.append(dict(name=name,expected_axis=c['guide_opening_axis'],cylinders=cyl))
    queries=[];newcommon=[]
    for report in [first,selected,electrical_final,raw]:
        key=report.get('first_blocked_sample')
        if not key or not report['native'][key]['overlaps']:continue
        p=next(p for p in report['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in p['samples'] if s['id']==key);q=max(report['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3']);M=np.array(s['transform'])
        sh=n.g.original.transform(native(q['lead']),M).intersect(native(q['part']),tol=1e-6);assert sh.isValid() and sh.Solids()
        name='arm070_failed_entry_common_'+str(len(queries));queries.append(dict(name=name,id=key,lead=q['lead'],part=q['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=q['material_intersection_mm3']))
        newcommon.append(dict(name=name,shape=sh,body='forearm',evidence_only=True,color=[.9,.1,.08]))
        n.a.save(HERE/'reviews/common_prepared.json',dict(completed=False,common_queries=queries,new_common_queries=len(queries),common_scene=None))
    common_scene=n.a.a.prior.export(newcommon,HERE/'generated/actual_common') if newcommon else None
    newmeta=n.p.read(ROOT/common_scene)['parts'] if common_scene else []
    n.a.save(HERE/'reviews/common_prepared.json',dict(completed=True,common_queries=queries,new_common_queries=len(queries),common_scene=common_scene))
    def scene(names,poses=None,extras=None,bright=None):
        parts=[];poses=poses or {};bright=set(bright or [])
        for name in sorted(set(names)):
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@matrix(name),color=[.95,.55,.1] if name in bright else z.get('color',[.3,.5,.75])))
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
    bench=n.p.read(ROOT/'cad/captive_module_assembly_v1/reviews/selected_check.json')['bench_names'];packet_names={k for p in raw['original16packets'] for k in p['names']}
    before=bench+sorted(set(raw['initial_world_present_names'])&packet_names);after=bench+sorted(set(raw['packet_final_present_names'])&packet_names)
    sheet([('이번 시작,완료5/16 묶음',scene(before)),('실제 마지막,완료'+str(len(raw['restored_modules']))+'/16 묶음',scene(after))],2,1100,'형상변경 없는 원래 뒤쪽 모터 복구','비packet478 조립가정 /다른 부품은 보기 생략 /현재862 교체·방사랙 외형채택 아님','restoration_before_after.png')
    oldq=prior_common['common_queries'][0];name=raw['module_contracts'][0]['module'];lead=name+'_lead1';guide=name+'_wire_guide_base'
    inherited_electrical=[z['path'] for z in raw['inherited_electrical_paths']]
    oldpair=scene([lead,guide],{lead:oldq['lead_transform']},bright=[lead]);lay=next((p for p in inherited_electrical+raw['paths'] if p['id'].endswith('__lay_three_leads_in_open_groove')),None)
    if lay:
        s=lay['samples'][-1];pair=scene([lead,guide],{lead:s['transform']},bright=[lead])
        sheet([('ARM069 옆진입,저장 실패 자세',oldpair),('새 열린 홈 진입,실제 마지막 자세',pair)],2,1100,'같은 전선과 안내대,조립 진입 방향의 차이','옛 공통 재계산0 /뚜껑·모터 등은 보기 생략 /실제검사에는 당시 전체 벤치 장애물 포함','wire_entry_before_after.png')
    paths=[z['path'] for z in raw['inherited_open_paths']]+inherited_electrical+[p for p in raw['paths'] if p['module']==name and p['stage']=='carrier_bench'];panels=[]
    labels=['guide_base_side_before_motor','unit7_with_own_plug_into_open_guide','lay_three_leads_in_open_groove','guide_lid_after_leads']
    for label in labels:
        p=next((p for p in paths if p['id'].endswith('__'+label)),None)
        if p:
            s=p['samples'][-1];panels.append(({'guide_base_side_before_motor':'안내대 옆진입','unit7_with_own_plug_into_open_guide':'모터와 자기 plug 착좌','lay_three_leads_in_open_groove':'열린 홈에 세 전선','guide_lid_after_leads':'전선 뒤 뚜껑'}[label],scene(p['present_names']+p['moving_names'],{k:s['transform'] for k in p['moving_names']},bright=p['moving_names'])))
    if panels:sheet(panels,2,1000,'원래 안내대·모터·전선·뚜껑의 실제 선후관계','느슨한 부품 수동유지 /전선·plug는 고정형상 참고 /공급품 내부·실제 하네스 변형 미검증','open_guide_order.png')
    body=next((p for p in raw['paths'] if p['stage']=='world' and '__body19_' in p['id']),None)
    if body:
        names=bench+sorted(set(body['present_names'])&packet_names)+body['moving_names']
        sheet([(label,scene(names,{k:s['transform'] for k in body['moving_names']},bright=body['moving_names'])) for label,s in [('전체 장애물 밖 시작',body['samples'][0]),('실제 실행 마지막',body['samples'][-1])]],2,1050,'조립한 원래 묶음의 세계좌표 진입','다른 부품은 보기 생략 /native는 해당 시점 전체 장애물 /세계등록·운반 및 연속경로 미검증','body_entry.png')
    for q,z in zip(queries,newmeta):
        pair=scene([q['lead'],q['part']],{q['lead']:q['lead_transform']},bright=[q['lead']]);red=scene([],extras=[z])
        vp=r.view_fit([pair],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);vr=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.)
        r.sheet([('실제로 막힌 두 부품',render(pair,vp,1050)),('같은 공통재료,별도 확대',render(red,vr,1050))],2,1050,'새 조립 시도에서 남긴 실패 증거','공통 '+format(q['volume_mm3'],'.10g')+' mm3 /원래 실패 보존 /작은 모서리 절삭0').save(HERE/'images'/('first_block_'+q['name'][-1]+'.png'))
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/selected_check.json',HERE/'reviews/final_check.json',HERE/'reviews/continuation_check.json',HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries),new_hardware_geometry=0,
        actual_lid_axis_reads=axes,old_failed_common_read_only=True,old_native_path_groups_repeated=0,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},
        opaque=True,all_selected_geometry_inside_frame=True,view_omissions_not_test_exclusions=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('rendered',len(list((HERE/'images').glob('*.png'))),'newcommon',len(queries),'newlidaxes',len(axes),flush=True)


if __name__=='__main__':main()

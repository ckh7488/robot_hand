"""Opaque views of unchanged original solids and actual independent nut entry."""
import argparse,importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm077_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm077_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--selected',action='store_true');args=ap.parse_args();tag='_selected' if args.selected else '';prefix='arm077'+tag+'_';report='selected_check' if args.selected else 'check';out=HERE/('reviews/evidence'+tag+'.json');assert not out.exists();q=n.p.read(HERE/('reviews/'+report+'.json'));assert q['completed'];source=dict(q['source_sha256']);metas,matrix=old.metadata();meta={z['name']:z for z in metas};meta.update({z['name']:z for z in n.p.read(ROOT/n.p.read(ROOT/'cad/branched_terminal_support_v1/reviews/check.json')['scene'])['parts']})
    def pose(name):return np.eye(4) if name==m.NAME else matrix(name)
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        return n.g.original.transform(n.c.load([z],source)[0]['shape'],pose(name))
    parts=m.split_nuts({m.BASE:dict(name=m.BASE,shape=native(m.BASE),body='forearm')});nutlookup={z['name']:z for z in parts}
    if not args.selected:
        assert not (HERE/'generated/original_nut_solids').exists()
        solids_scene=n.a.a.prior.export([dict(name=z['name'],shape=z['shape'],body='forearm',color=[.8,.55,.12],source_compound=m.BASE,source_solid_index=z['source_solid_index'],evidence_only=True,not_additional_hardware=True) for z in parts],HERE/'generated/original_nut_solids')
    else:solids_scene=n.p.read(HERE/'reviews/evidence.json')['original_solids_scene']
    extracted=n.p.read(ROOT/solids_scene)['parts'];meta.update({z['name']:z for z in extracted});queries=[];common_scene=None;extras=[];key=q.get('first_blocked_sample')
    if key and q['native'][key]['overlaps']:
        path=q['paths'][-1];sample=path['samples'][-1];v=max(q['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3']);lead=nutlookup[v['lead']]['shape'];fixed=nutlookup[v['part']]['shape'] if v['part'] in nutlookup else native(v['part']);M=np.array(sample['transform']);sh=n.g.original.transform(lead,M).intersect(fixed,tol=1e-6);assert sh.isValid() and sh.Solids();name=prefix+'first_new_common'
        queries=[dict(name=name,id=key,lead=v['lead'],part=v['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=v['material_intersection_mm3'])]
        n.a.save(HERE/('reviews/common_prepared'+tag+'.json'),dict(completed=False,common_queries=queries,common_scene=None));common_scene=n.a.a.prior.export([dict(name=name,shape=sh,body='forearm',color=[.9,.1,.08],evidence_only=True)],HERE/('generated/actual_common'+tag));extras=n.p.read(ROOT/common_scene)['parts'];meta.update({z['name']:z for z in extras})
    n.a.save(HERE/('reviews/common_prepared'+tag+'.json'),dict(completed=True,common_queries=queries,common_scene=common_scene,new_common_queries=len(queries)));neutral={z['name'] for z in extracted+extras};red={z['name'] for z in extras}
    def scene(names,poses=None):
        result=[];poses=poses or {}
        for name in sorted(set(names)):
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];M=np.eye(4) if name in neutral else pose(name)
            result.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@M,color=[.9,.1,.08] if name in red else [.9,.63,.12] if name in nutlookup else [.93,.48,.1] if name==m.NAME else z.get('color',[.3,.5,.75])))
        return dict(parts=result,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.;return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def sheet(panels,title,footer,name,separate=False):
        target=HERE/'images'/(prefix+name+'.png');assert not target.exists();size=1100;direction=np.array([.65,-.5,.6]);v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],direction,1.);images=[]
        for label,s in panels:
            vv=r.view_fit([s],[dict(angles_deg=[0.]*20)],direction,1.) if separate else v;images.append((label,render(s,vv,size)))
        r.sheet(images,2,size,title,footer).save(target)
    (HERE/'images').mkdir(exist_ok=True);p=q['paths'][0];last=p['samples'][-1];previous=n.p.read(ROOT/'cad/preloaded_rack_nut_module_v1/reviews/selected_check.json');oldM=previous['paths'][-1]['samples'][-1]['transform']
    sheet([('이전,4너트 동시 옆진입 X+29.5,실패',scene([m.BASE,m.NAME],{m.BASE:oldM})),('새,첫 개별 너트의 실제 마지막 자세',scene(p['moving_names']+[m.NAME,'thumb_ip_actuator_adapter'],{k:last['transform'] for k in p['moving_names']}))],'원래 네 너트의 동시 이동과 개별 조립 비교','원래 형상·최종 위치 유지 /기존106 재계산0 /새 경로 실제도달만 /그림은 국부 부품만','order_before_after')
    names=set(q['mapped_special_packet']['names'])-{m.BASE};done=set(q['final_completed_solid_names'])
    sheet([('이전,마지막 너트묶음 없는 모듈47',scene(names)),('새,실제 개별 착좌 '+str(len(done))+'/4',scene(names|done))],'개별 너트 조립의 실제 마지막 구성','native는 뒤8·손·팔 포함717 전체+먼저 놓은 너트 /완료4개는 원래compound 한 행으로 복구 /전체 구동 미승인','module_actual_result')
    aligned=next((s for s in p['samples'] if s['phase']=='original_axis_seating'),last)
    sheet([('실제 전체장애물 밖 시작',scene(p['moving_names']+['thumb_ip_actuator_adapter'],{k:p['samples'][0]['transform'] for k in p['moving_names']})),('같은 경로의 도달한 축맞춤/마지막',scene(p['moving_names']+['thumb_ip_actuator_adapter'],{k:aligned['transform'] for k in p['moving_names']}))],'첫 개별 너트의 전체 진입 경로','표시는 너트·adapter만 /모든717 및 완료너트가 native 장애물 /연속 경로·공구·토크 미검증','individual_entry')
    if queries:
        a=queries[0];sheet([('새 실패 자세,실제 두 부품',scene([a['lead'],a['part']],{a['lead']:a['lead_transform']})),('같은 실제 공통재료 확대',scene([a['name']]))],'개별 너트의 첫 실제 막힘','공통 '+format(a['volume_mm3'],'.10g')+' mm3 /증거는 설치부품 아님 /첫 실패 뒤 미실행','first_new_common',True)
    for f in [Path(__file__),HERE/'model.py',HERE/('reviews/'+report+'.json'),HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    outputs={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob(prefix+'*.png')}
    n.a.save(out,dict(completed=True,source_sha256=source,original_solids_scene=solids_scene,original_solid_extraction_count=4,new_original_solid_exports=0 if args.selected else 4,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries)+(0 if args.selected else 4),new_hardware_geometry=0,output_sha256=outputs,opaque=True,unique_image_names_no_overwrite=True,view_omissions_not_native_exclusions=True,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('render',len(outputs),'original evidence exports',0 if args.selected else 4,'newcommon',len(queries),flush=True)


if __name__=='__main__':main()

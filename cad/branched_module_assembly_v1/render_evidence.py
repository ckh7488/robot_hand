"""Unique opaque images of actual dualport bench and reached world entry."""
import argparse,importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm075_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm075_render_engine',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--selected',action='store_true');args=ap.parse_args();suffix='_selected' if args.selected else '';prefix='arm075_selected_' if args.selected else 'arm075_'
    report='selected_check' if args.selected else 'check';out=HERE/('reviews/evidence'+suffix+'.json');assert not out.exists();q=n.p.read(HERE/('reviews/'+report+'.json'));assert q['completed'];source=dict(q['source_sha256'])
    metas,matrix=old.metadata();meta={z['name']:z for z in metas};fresh=n.p.read(ROOT/n.p.read(ROOT/'cad/branched_terminal_support_v1/reviews/check.json')['scene'])['parts'];meta.update({z['name']:z for z in fresh});queries=[];common_scene=None;extras=[]
    def pose(name):return np.eye(4) if name==m.NAME else matrix(name)
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        return n.g.original.transform(n.c.load([z],source)[0]['shape'],pose(name))
    key=q.get('first_blocked_sample')
    if key and q['native'][key]['overlaps']:
        p=next(p for p in q['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in p['samples'] if s['id']==key);v=max(q['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3']);M=np.array(s['transform'])
        sh=n.g.original.transform(native(v['lead']),M).intersect(native(v['part']),tol=1e-6);assert sh.isValid() and sh.Solids();name=prefix+'first_assembly_common'
        queries=[dict(name=name,id=key,lead=v['lead'],part=v['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=v['material_intersection_mm3'])]
        n.a.save(HERE/('reviews/common_prepared'+suffix+'.json'),dict(completed=False,common_queries=queries,common_scene=None));common_scene=n.a.a.prior.export([dict(name=name,shape=sh,body='forearm',color=[.9,.1,.08],evidence_only=True)],HERE/('generated/actual_common'+suffix));extras=n.p.read(ROOT/common_scene)['parts'];meta.update({z['name']:z for z in extras})
    n.a.save(HERE/('reviews/common_prepared'+suffix+'.json'),dict(completed=True,common_queries=queries,common_scene=common_scene,new_common_queries=len(queries)))
    rednames={z['name'] for z in extras}
    def scene(names,poses=None):
        parts=[];poses=poses or {}
        for name in sorted(set(names)):
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];M=np.eye(4) if name in rednames else pose(name)
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@M,color=[.9,.1,.08] if name in rednames else [.93,.48,.1] if name in {m.NAME,m.OLD} else z.get('color',[.3,.5,.75])))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def sheet(panels,title,footer,file,size=1100):
        target=HERE/'images'/file;assert not target.exists();v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.)
        r.sheet([(label,render(s,v,size)) for label,s in panels],2,size,title,footer).save(target)
    (HERE/'images').mkdir(exist_ok=True)
    previous=n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json')
    if not args.selected:
        sheet([('이전 첫port0 벤치34',scene(previous['final_carrier_bench_present_names'])),('새 실제 마지막 벤치'+str(len(q['final_carrier_bench_present_names'])),scene(q['final_carrier_bench_present_names']))],
            '새 분기 받침에서 양쪽 조절부 조립','원래 모터·전기·죠 위치 유지 /M3포함 보류 /별도 벤치·수동유지 /전체구동 미승인','arm075_dualport_bench_before_after.png')
        sheet([('원래 port1 조절부 시작',scene(q['initial_kit1_present_names'])),('실제 port1 선조립 마지막',scene(q['final_kit1_present_names']))],
            '두 번째 원래7개 부품의 독립 선조립','첫port0 독립42·motor14는 같은입력 증명승계/새0 /새받침과의 진입은 별도 새검사','arm075_port1_kit_before_after.png')
    else:
        initial=n.p.read(HERE/'reviews/check.json');p=initial['paths'][-1];first=q['paths'][0];context={first['moving_names'][0],'Wrist_box_carrier'}
        sheet([('이전 앞쪽+Y20 진입,실패',scene(context,{k:p['samples'][-1]['transform'] for k in p['moving_names']})),('새 최종축위치에서 옆진입 마지막',scene(context,{k:first['samples'][-1]['transform'] for k in first['moving_names']}))],
            '너트의 실제 진입 높이와 손목 받침','전체장애물 밖 시작 유지 /원래부품·최종위치 유지 /이전 실패 삭제0 /native는713전체',prefix+'nut_order.png')
    world=next((p for p in q['paths'] if p['stage']=='world'),None)
    if world and not args.selected:
        bench_names=n.p.read(ROOT/'cad/captive_module_assembly_v1/reviews/selected_check.json')['bench_names'];context=set(bench_names)&set(world['present_names'])
        panels=[(label,scene(set(world['moving_names'])|context,{k:s['transform'] for k in world['moving_names']})) for label,s in [('전체 장애물 밖 시작',world['samples'][0]),('실제 마지막'+(' /통과' if world['completed'] else ' /중단'),world['samples'][-1])]]
        sheet(panels,'특수 몸체43 전체의 원래 장착면 진입','그림은 받침22만 표시 /native는 실제670 전체 /새받침1 명시교체 /힘·연속조립·전체외형 미승인','arm075_world_body_entry.png')
    if queries:
        a=queries[0];pair=scene([a['lead'],a['part']],{a['lead']:a['lead_transform']});red=scene([a['name']]);panels=[]
        for label,s in [('첫 새막힘의 실제 두 부품',pair),('같은 공통재료만 확대',red)]:
            v=r.view_fit([s],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);panels.append((label,render(s,v,1100)))
        target=HERE/('images/'+prefix+'first_new_common.png');assert not target.exists();r.sheet(panels,2,1100,'새 조립 경로의 첫 실제 막힘','공통 '+format(a['volume_mm3'],'.10g')+' mm3 /증거는 설치부품 아님').save(target)
    for f in [Path(__file__),HERE/'model.py',HERE/('reviews/'+report+'.json'),HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    output_files=[f for f in (HERE/'images').glob(prefix+'*.png') if args.selected or not f.name.startswith('arm075_selected_')]
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries),new_hardware_geometry=0,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in output_files},opaque=True,all_selected_geometry_inside_frame=True,unique_image_names_no_overwrite=True,view_omissions_not_test_exclusions=True,old_inclusion_Boolean_or_points_repeated=0,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('rendered',len(output_files),'newcommon',len(queries),flush=True)


if __name__=='__main__':main()

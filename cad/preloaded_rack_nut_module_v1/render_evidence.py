"""Unique opaque views of the nut-first order and the first new block."""
import argparse,importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm076_render',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm076_renderer',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');r,engine=old.r,old.engine


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--selected',action='store_true');args=ap.parse_args();suffix='_selected' if args.selected else '';prefix='arm076_selected_' if args.selected else 'arm076_';report='selected_check' if args.selected else 'check'
    out=HERE/('reviews/evidence'+suffix+'.json');assert not out.exists();q=n.p.read(HERE/('reviews/'+report+'.json'));assert q['completed'];source=dict(q['source_sha256'])
    metas,matrix=old.metadata();meta={z['name']:z for z in metas};fresh=n.p.read(ROOT/n.p.read(ROOT/'cad/branched_terminal_support_v1/reviews/check.json')['scene'])['parts'];meta.update({z['name']:z for z in fresh});queries=[];common_scene=None;extras=[]
    def pose(name):return np.eye(4) if name==m.NAME else matrix(name)
    def native(name):
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=source[rel])
        return n.g.original.transform(n.c.load([z],source)[0]['shape'],pose(name))
    key=q.get('first_blocked_sample')
    if key and q['native'][key]['overlaps']:
        p=next(p for p in q['paths'] if any(s['id']==key for s in p['samples']));s=next(s for s in p['samples'] if s['id']==key);v=max(q['native'][key]['overlaps'],key=lambda z:z['material_intersection_mm3']);M=np.array(s['transform']);sh=n.g.original.transform(native(v['lead']),M).intersect(native(v['part']),tol=1e-6);assert sh.isValid() and sh.Solids()
        name=prefix+'first_new_assembly_common';queries=[dict(name=name,id=key,lead=v['lead'],part=v['part'],lead_transform=M,volume_mm3=sh.Volume(),solids=len(sh.Solids()),source_native_volume_mm3=v['material_intersection_mm3'])]
        n.a.save(HERE/('reviews/common_prepared'+suffix+'.json'),dict(completed=False,common_queries=queries,common_scene=None));common_scene=n.a.a.prior.export([dict(name=name,shape=sh,body='forearm',color=[.9,.1,.08],evidence_only=True)],HERE/('generated/actual_common'+suffix));extras=n.p.read(ROOT/common_scene)['parts'];meta.update({z['name']:z for z in extras})
    n.a.save(HERE/('reviews/common_prepared'+suffix+'.json'),dict(completed=True,common_queries=queries,common_scene=common_scene,new_common_queries=len(queries)));rednames={z['name'] for z in extras}
    def scene(names,poses=None):
        parts=[];poses=poses or {}
        for name in sorted(set(names)):
            z=meta[name];path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];M=np.eye(4) if name in rednames else pose(name)
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=np.array(poses.get(name,np.eye(4)))@M,color=[.9,.1,.08] if name in rednames else [.95,.68,.15] if 'rack_nut' in name else [.93,.48,.1] if name in {m.NAME,m.OLD} else z.get('color',[.3,.5,.75])))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,v,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':v},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':v},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def sheet(panels,title,footer,file,separate=False):
        target=HERE/'images'/file.replace('arm076_',prefix,1);assert not target.exists();size=1100;v=r.view_fit([s for _,s in panels],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.);images=[]
        for label,s in panels:
            vv=r.view_fit([s],[dict(angles_deg=[0.]*20)],np.array([.65,-.5,.6]),1.) if separate else v;images.append((label,render(s,vv,size)))
        r.sheet(images,2,size,title,footer).save(target)
    (HERE/'images').mkdir(exist_ok=True)
    before=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/check.json');nutpaths=[p for p in q['paths'] if p['id'].startswith('preload_rack_nut_')];placed=set(q['initial_carrier_bench_present_names'])
    for p in nutpaths:
        if p['completed']:placed.update(p['moving_names'])
    sheet([('이전,양 조절부 뒤 너트 진입',scene(before['final_carrier_bench_present_names'])),('새 순서,양 조절부보다 너트 먼저',scene(placed))],'원래 랙 너트 두 개의 선조립 순서','같은 부품·최종축 유지 /누락은 단계별 조립 조건 /수동유지·운반·실제 공구 미검증','arm076_nut_first_before_after.png')
    sheet([('같은 전기 몸체25에서 시작',scene(q['initial_carrier_bench_present_names'])),('새 순서 실제 마지막 벤치'+str(len(q['final_carrier_bench_present_names'])),scene(q['final_carrier_bench_present_names']))],'너트 뒤 양쪽 조절부의 실제 벤치 결과','기존 독립98·전기64는 동일입력 승계 /너트가 있는 조절부 진입은 새검사 /전체 구동 미승인','arm076_bench_actual_result.png')
    world=next((p for p in q['paths'] if p['stage']=='world' and len(p['moving_names'])==45),None)
    if world:
        bench=n.p.read(ROOT/'cad/captive_module_assembly_v1/reviews/selected_check.json')['bench_names'];context=set(bench)&set(world['present_names'])
        sheet([(label,scene(set(world['moving_names'])|context,{k:s['transform'] for k in world['moving_names']})) for label,s in [('전체 장애물 밖 시작',world['samples'][0]),('실제 마지막'+(' /표본 통과' if world['completed'] else ' /중단'),world['samples'][-1])]],'너트를 포함한 몸체45의 원래 장착면 진입','그림은 받침22만 표시 /native는 뒤8 포함670전체 /원래714부분 결과는 별도 보존','arm076_body45_world_entry.png')
    if key:
        p=next(p for p in q['paths'] if any(s['id']==key for s in p['samples']));names=set(p['moving_names'])|{z['part'] for z in q['native'][key]['overlaps']};last=p['samples'][-1]
        sheet([('새 경로 전체밖 시작',scene(names,{k:p['samples'][0]['transform'] for k in p['moving_names']})),('첫 실제 막힘 자세',scene(names,{k:last['transform'] for k in p['moving_names']}))],'새 순서에서 도달한 첫 막힘','표시는 해당 moving·실제 겹친 상대만 /native는 저장된 모든 present /실패 뒤 경로 미실행','arm076_first_block_path.png')
    if queries:
        a=queries[0];sheet([('실제 두 부품의 실패 자세',scene([a['lead'],a['part']],{a['lead']:a['lead_transform']})),('같은 실제 공통재료 확대',scene([a['name']]))],'새 조립 경로의 대표 공통재료','공통 '+format(a['volume_mm3'],'.10g')+' mm3 /설치부품 아님 /원시 실패 유지','arm076_first_new_common.png',True)
    for f in [Path(__file__),HERE/'model.py',HERE/('reviews/'+report+'.json'),HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    images={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob(prefix+'*.png') if args.selected or not f.name.startswith('arm076_selected_')}
    n.a.save(out,dict(completed=True,source_sha256=source,common_scene=common_scene,common_queries=queries,new_common_queries=len(queries),new_evidence_geometry=len(queries),new_hardware_geometry=0,output_sha256=images,opaque=True,unique_image_names_no_overwrite=True,view_omissions_not_test_exclusions=True,old_inclusion_Boolean_or_points_repeated=0,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))
    print('rendered',len(images),'newcommon',len(queries),flush=True)


if __name__=='__main__':main()

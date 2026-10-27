"""Save actual common material and show the completed geometric connection."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_link as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
helper=m.drive.module('_guided030_renderer',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine
from PIL import ImageDraw

def main():
    raw=p.read(HERE/'reviews/check.json');assert raw['completed'];case=raw['cases'][0];rows=c.load(p.read(ROOT/case['scene'])['parts'])
    data,pack,guide,hardware=b.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);lookup={z['name']:z for z in rows}
    obstacle={'hand':hand,'packaging':pack,'electrical':data[6],'arm':data[7],'wrist_hardware':hardware};commons=[];pairs=[]
    for k,test in case['native'].items():
        if not test['overlaps']:continue
        hit=max(test['overlaps'],key=lambda z:z['material_intersection_mm3']);ob=next(z for z in obstacle[k] if z['name']==hit['part']);sh=lookup[hit['lead']]['shape'].intersect(ob['shape'],tol=1e-6)
        commons.append(dict(name='actual_'+k+'_common',shape=sh,body='palm',color=[.98,.02,.05],installed_part=False))
        pairs.append(dict(group=k,**hit,saved_STEP_common_default_volume_mm3=sh.Volume(),common_name=commons[-1]['name']))
    evidence=b.export(commons,HERE/'generated/actual_common') if commons else None
    meta=lambda xs:[{k:v for k,v in z.items() if k!='shape'} for z in xs]
    context=meta(hand+pack+hardware+data[6])
    for z in data[7]:
        path=ROOT/'cad/remote_bus_v1/generated/arm_visual_meshes'/(z['name']+'.stl')
        assert path.exists();context.append(dict(name=z['name'],mesh=path.relative_to(ROOT).as_posix(),sha256_stl=m.drive.sha(path),color=[.5,.6,.7]))
    p.write(HERE/'reviews/details.json',dict(completed=True,source_sha256=data[0],actual_pairs=pairs,common_scene=evidence,
            whole_current_rows=dict(hand=len(hand),packaging=len(pack),electrical=len(data[6]),neutral_arm=len(data[7]),wrist_hardware=len(hardware)),
            endpoint_link_only_not_threading_or_tension=True))
    folder=HERE/'images';folder.mkdir(exist_ok=False);source={};fresh=p.read(ROOT/case['scene'])['parts'];commonmeta=p.read(ROOT/evidence)['parts'] if evidence else []
    local=p.read(ROOT/'cad/wrist_outlet_route_v1/inboard/generated/assembly_scene.json')['new_parts'];local=[z for z in local if not z['installed_part_candidate']]
    def scene(extra,ctx):
        parts=[]
        for z,alpha in [(z,.16) for z in ctx]+[(z,.82) for z in extra]:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.57,.63,.69]),alpha=alpha,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    before=scene(local,context);after=scene(fresh+commonmeta,context)
    # The inset shows only the true hand/wrist obstacle solids and distal span.
    distal=[z for z in fresh if z['segment']>=4];near=[z for z in context if z['name'] in {x['part'] for x in pairs} or z['name'].startswith('wrist_')]
    inset=scene(distal+commonmeta,near);scenes=[before,after,inset]
    view=r.view_fit([before,after],[dict(angles_deg=[0.]*20)]*2,np.array([1.,-.65,.45]),1.)
    close=r.view_fit([inset],[dict(angles_deg=[0.]*20)],np.array([.6,-1.,.7]),1.);ims=[]
    try:
        for index,s in enumerate(scenes):
            v=close if index==2 else view;s['_render_state']=r.make_renderer(s,{'v':v},1100,engine)
            for j,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'v',{'v':v},1100,engine);ims.append(im)
        r.sheet([('이전: 손목 국부 참고만 연결',ims[0]),('이번: 실제 양끝과 안내대를 이은 전체 후보',ims[1])],2,1100,
                '모터 → 손목 안내대 → 엄지 종단 · 같은 손/모터 배치',
                '보라: 피복2개 · 주황: 구동줄2개 · 빨강: 실제 공통 재료 · 설명용 정적 CAD,실제 구동/장력 승인이 아님').save(folder/'link_before_after.png')
        r.sheet([('새 전체 후보에서 남은 실제 관통',ims[2])],1,1100,'국부 통과와 전체 연결의 차이',
                '피복 외경/손·모터는 유지 · 빨강만 실제 공통 재료 · 손 전체가 절삭 대상이라는 뜻이 아님').save(folder/'actual_block.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    p.write(folder/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in folder.iterdir() if z.is_file()},actual_saved_CAD=True,whole_hand_drive_approved=False))
if __name__=='__main__':main()

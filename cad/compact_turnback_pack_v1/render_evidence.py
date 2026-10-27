"""Opaque hardware comparison and actual first blocking material evidence."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import pack_model as a
n,k,np,ROOT,HERE=a.n,a.k,a.np,a.ROOT,a.HERE
v=n.m.drive.module('_pack049_visual_basis',ROOT/'cad/drive_packaging_review_v1/inspect_render.py')
r,engine=v.r,v.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json');assert raw['completed']
    _,base,_=v.metadata();new=n.p.read(ROOT/raw['scene'])['parts']
    removed=set(raw['replaced_hardware']);after=[z for z in base if z['name'] not in removed]+new
    assert len(base)==len(after)==862
    source=dict(raw['source_sha256']);source['cad/compact_turnback_pack_v1/reviews/check.json']=r.sha(HERE/'reviews/check.json')
    source['cad/compact_turnback_pack_v1/render_evidence.py']=r.sha(Path(__file__))
    def make(meta,transforms=None):
        transforms=transforms or {};parts=[]
        for z in meta:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            color=[.62,.68,.73] if z in base[:397] else [.36,.43,.49]
            if z['name'] in removed or z['name'].startswith('compact_'):color=[.93,.48,.1]
            if z.get('evidence_only'):color=[.97,.04,.03]
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,
                neutral_inverse=transforms.get(z['name'],np.eye(4))))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(assembly,view,size):
        try:
            assembly['_render_state']=r.make_renderer(assembly,{'v':view},size,engine)
            assembly['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(assembly,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in assembly:
                assembly['_render_state']['renderer'].close();del assembly['_render_state']
    before,now=make(base),make(after);tiles=[]
    for label,normal in [('Y-Z 옆면',[1.,0.,0.]),('사선',[.8,-.9,.5])]:
        view=r.view_fit([before,now],[dict(angles_deg=[0.]*20)],np.array(normal),1.)
        tiles.extend([('이전 고정부862 / '+label,render(before,view,800)),('새 고정부 후보862 / '+label,render(now,view,800))])
    images=HERE/'images';images.mkdir(exist_ok=True)
    r.sheet(tiles,2,800,'출구 방향을 함께 바꾼 별도 고정부 후보: 실제 CAD 불투명 비교',
        '양쪽 모두 긴 피복·줄 미표시, 고정부 비교만 / 모터·손·배선·팔은 모두 표시 / 연결·외형 개선·구동 승인 아님').save(images/'hardware_before_after.png')
    state=raw['states'][-1];records=[dict(group=key,pair=pair) for key,test in state['native'].items() for pair in test['overlaps']]
    evidence=[]
    if records:
        chosen=max(records,key=lambda z:z['pair']['material_intersection_mm3']);pair=chosen['pair']
        lookup={z['name']:z for z in after};selected=[lookup[pair['lead']],lookup[pair['part']]]
        rotor={z['name'] for z in n.p.read(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json')['parts'] if z['moving']}
        transforms=v.fr.transforms(base,state['case'],rotor)
        for z in new:transforms[z['name']]=np.array(state['case']['H']) if z['body']=='palm' else np.eye(4)
        loaded=n.c.load(selected,source)
        shapes=[n.g.original.transform(z['shape'],transforms.get(z['name'],np.eye(4))) for z in loaded]
        common=shapes[0].intersect(shapes[1],tol=1e-6)
        commonrow=dict(name='pack049_actual_block',shape=common,body='palm',evidence_only=True,installed_part_candidate=False,color=[.97,.04,.03])
        scene=a.export([commonrow],'actual_common');meta=n.p.read(ROOT/scene)['parts']
        item=dict(state_id=state['id'],**chosen,common_scene=scene,common_bbox_mm=n.c.m.pack._bbox(common),
            default_common_volume_mm3=common.Volume())
        evidence.append(item)
        local=make(selected+meta,transforms);view=r.view_fit([local],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
        common_only=make(meta);zoom=r.view_fit([common_only],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
        r.sheet([('실제 두 부품 전체 / 빨강은 공통재료',render(local,view,850)),('같은 공통재료만 확대 / 별도 배율',render(common_only,zoom,850))],2,850,
            '새 고정부 후보의 실제 막힘',str(pair['material_intersection_mm3'])+' mm³ / 손 결함·모든 밀착 배치 불가능의 증거 아님').save(images/'actual_block.png')
    n.a.save(out,dict(completed=True,source_sha256=source,hardware_before_rows=862,hardware_candidate_rows=862,
        whole_housings_and_cords_displayed_on_either_hardware_panel=False,unbuilt_cover_or_hidden_hardware=False,
        new_native_groups=0,actual_block_common_pairs=evidence,
        native_static_reruns=0,outputs={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        exterior_improvement_or_acceptance_claimed=False,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

"""Actual complete saved candidate, opaque comparison and blocking evidence."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import compact_model as a
n,k,np,ROOT,HERE=a.n,a.k,a.np,a.ROOT,a.HERE
v=n.m.drive.module('_link050_visual',ROOT/'cad/drive_packaging_review_v1/inspect_render.py')
r,engine=v.r,v.engine
measure=n.m.drive.module('_link050_measure',ROOT/'cad/long_housing_route_v1/review_core.py')


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();raw=n.p.read(HERE/'reviews/check.json');assert raw['completed']
    source=dict(raw['source_sha256']);source['cad/compact_material_link_v1/reviews/check.json']=r.sha(HERE/'reviews/check.json')
    source['cad/compact_material_link_v1/render_link.py']=r.sha(Path(__file__))
    _,base,oldmaterial=v.metadata();new=n.p.read(ROOT/'cad/compact_turnback_pack_v1/generated/hardware/scene.json')['parts']
    removed={z['replaces'] for z in new};hardware=[z for z in base if z['name'] not in removed]+new
    rotor={z['name'] for z in n.p.read(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json')['parts'] if z['moving']}
    def transforms(case,material):
        result=v.fr.transforms(base,case,rotor)
        for z in new:result[z['name']]=np.array(case['H']) if z['body']=='palm' else np.eye(4)
        for z in material:result[z['name']]=np.eye(4)
        return result
    def make(meta,T):
        parts=[]
        for z in meta:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            color=[.62,.68,.73] if z['name'] in {q['name'] for q in base[:397]} else [.36,.43,.49]
            if z['name'] in removed or z['name'].startswith('compact_motor_turnback') or z['name']=='compact_saddle_nested_turns':color=[.93,.48,.1]
            if z.get('kind') in ['housing','cord']:color=[.62,.21,.78] if z['physical_line']==0 else [.1,.48,.78]
            if z.get('evidence_only'):color=[.97,.04,.03]
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,neutral_inverse=T.get(z['name'],np.eye(4))))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(ss,view,size):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':view},size,engine);ss['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(ss,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close();del ss['_render_state']
    images=HERE/'images';images.mkdir(exist_ok=True);evidence=[]
    if raw['states']:
        neutral=raw['states'][0];material=n.p.read(ROOT/neutral['scene'])['parts'];before=make(base+oldmaterial,{})
        after=make(hardware+material,transforms(neutral['case'],material));tiles=[]
        for name,normal in [('Y-Z 옆면',[1.,0.,0.]),('사선',[.8,-.9,.5])]:
            view=r.view_fit([before,after],[dict(angles_deg=[0.]*20)],np.array(normal),1.)
            tiles += [('이전 전체 연결980행 / '+name,render(before,view,850)),('새 길이 후보 전체 연결'+str(862+len(material))+'행 / '+name,render(after,view,850))]
        r.sheet(tiles,2,850,'실제 전체 피복·줄까지 연결한 별도 후보: 불투명 CAD',
            '고정부·손·모터·배선·팔 참고 전부 표시 / 임시 덮개·부품 숨김 없음 / 외형 수용·힘·전체 구동 승인 아님').save(images/'connected_before_after.png')
        blocked=next((z for z in raw['states'] if z['new_blocking_records']),None)
        if blocked:
            chosen=max(blocked['new_blocking_records'],key=lambda z:z['pair']['material_intersection_mm3']);pair=chosen['pair']
            material=n.p.read(ROOT/blocked['scene'])['parts'];meta={z['name']:z for z in hardware+material};T=transforms(blocked['case'],material)
            weights=[[np.array(z) for z in row] for row in raw['arc_length_weights']];budgets=np.array(raw['free_contour_budgets_mm']);x=np.array(blocked['parameters'])
            shapes=[]
            for name in [pair['lead'],pair['part']]:
                z=meta[name]
                if z.get('kind') in ['housing','cord']:
                    e=a.paths(x,blocked['case'],weights,budgets,z['physical_line'],z['kind'])[z['path_segment']]
                    sh=n.m.swept(e,1.5).cut(n.m.swept(e,.65)).clean() if z['kind']=='housing' else n.m.swept(e,.3)
                else:sh=n.g.original.transform(n.c.load([z],source)[0]['shape'],T.get(name,np.eye(4)))
                shapes.append(sh)
            common=shapes[0].intersect(shapes[1],tol=1e-6)
            scene=a.export([dict(name='link050_actual_block',shape=common,body='palm',evidence_only=True,installed_part_candidate=False,color=[.97,.04,.03])],'actual_common')
            commonmeta=n.p.read(ROOT/scene)['parts'];adaptive=measure.metrics(common)['global_values']
            evidence.append(dict(state_id=blocked['id'],**chosen,source_common_adaptive_values=adaptive,common_bbox_mm=n.c.m.pack._bbox(common),scene=scene))
            local=make([meta[pair['lead']],meta[pair['part']]]+commonmeta,T);v1=r.view_fit([local],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
            only=make(commonmeta,{});v2=r.view_fit([only],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
            r.sheet([('두 실제 부품 전체와 빨간 공통재료',render(local,v1,850)),('같은 공통재료 확대 / 별도 배율',render(only,v2,850))],2,850,
                '새 연결 후보의 실제 막힘',str(pair['material_intersection_mm3'])+' mm³ / 한 경로 후보의 실패, 손 절삭·모든 밀착배치 불가능의 증거 아님').save(images/'actual_block.png')
    n.a.save(out,dict(completed=True,source_sha256=source,actual_common_pairs=evidence,new_native_groups=0,
        old_seam_precision_or_checks_reexecuted=0,opaque_full_candidate=True,rendered_saved_material_states=[z['id'] for z in raw['states'][:1]],
        outputs={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        exterior_improved_or_accepted=False,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()

"""Actual saved fastener start/end geometry; no native checks repeated."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import basis as b
r,engine,old=b.e.r,b.e.engine,b.e.old


def main():
    raw=b.p.read(b.HERE/'reviews/check.json');assert raw['completed']
    base=old.current_metadata();meta={z['name']:z for z in base};source={}
    alt=b.p.read(b.ROOT/'cad/motor_full_cord_thread_v1/reviews/anchor_check.json')
    for i in [0,1]:
        rec=next(z for z in reversed(alt['cases']) if z['line']==i)
        for z in b.p.read(b.ROOT/rec['scene'])['parts']:meta[z['name']]=z
    views=[];paths={z['id']:z for z in raw['paths']}
    motor=['thumb_ip_two_tail_spool','thumb_ip_tail_clamp_1','thumb_ip_tail_clamp_2','anchorfeed_cord_0_0','anchorfeed_cord_1_0']
    child=['thumb_segment3_tip_core','thumb_ip_tendon_clamp']+['thumb_ip_tendon_clamp_'+kind+'_'+str(j) for kind in ['screw','nut'] for j in [1,2]]+['anchorfeed_cord_'+str(i)+'_'+str(23 if i==0 else 27) for i in [0,1]]
    def scene(names,changed):
        parts=[]
        for name in names:
            z=changed if name==changed['name'] else meta[name];file=b.ROOT/z['mesh']
            assert r.sha(file)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            iscord=name.startswith('anchorfeed');ismoving=name==changed['name']
            parts.append(dict(name=name,body='palm',mesh_path=file,points=r.read_stl_points(file),
                color=[.97,.45,.12] if iscord else ([.45,.2,.8] if ismoving else [.56,.64,.7]),
                alpha=1. if iscord or ismoving else .25,neutral_inverse=b.np.eye(4)))
        return dict(parts=parts,design=b.g.DESIGN)
    panels=[]
    for key,names,label in [('motor_0',motor,'모터 핀치나사1'),('child_screw_1',child,'자식 클램프 나사1')]:
        path=paths[key];saved=[z for z in path['samples'] if 'scene' in z];scenes=[]
        for sample in [saved[0],saved[-1]]:
            z=b.p.read(b.ROOT/sample['scene'])['parts'][0];scenes.append(scene(names,z))
        view=r.view_fit(scenes,[dict(angles_deg=[0.]*20)],b.np.array([.8,-.9,.5]),1.)
        for j,ss in enumerate(scenes):
            try:
                ss['_render_state']=r.make_renderer(ss,{'v':view},850,engine)
                for q,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[q,3]=z['alpha']
                img=r.render_one(ss,[0.]*20,'v',{'v':view},850,engine)[0]
            finally:
                if '_render_state' in ss:ss['_render_state']['renderer'].close()
            sample=[saved[0],saved[-1]][j]
            panels.append((label+' / 축 바깥 '+str(sample['outward_distance_mm'])+' mm',img))
    images=b.HERE/'images';images.mkdir(exist_ok=False)
    r.sheet(panels,2,850,'실제 꼬리 고정부: 나사 접근 시작과 명목 복구',
        '각 나사는 별도 조건부 경로 / 전체 줄은 검사에 포함, 그림은 고정부의 실제 줄 조각만 표시 / 접촉은 고정력·예압 승인이 아님').save(images/'fixation_before_after.png')
    b.a.save(images/'report.json',dict(completed=True,source_sha256=source,
        native_recomputed=0,actual_saved_CAD=True,arms_not_displayed=True,
        outputs={z.name:r.sha(z) for z in images.iterdir() if z.is_file()}))


if __name__=='__main__':main()

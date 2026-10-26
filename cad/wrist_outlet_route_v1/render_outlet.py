"""Saved actual CAD before/after; no generated illustrative geometry."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
from outlet_survey import m,g,c,p,np,cq,ROOT,HERE
helper=m.drive.module('_outlet029_render_helper',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine
from PIL import ImageDraw

def main():
    folder=HERE/'images';assert not (folder/'outlet_before_after.png').exists();source={};catalog=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];context=[z for z in catalog if z['name'] in ['Palm_tapered_chassis','thumb_root_fixed_housing']];ev=p.read(ROOT/'cad/wrist_housing_guide_v1/generated/parent_block/scene.json')['parts'];context+=[next(z for z in ev if z['name']=='quarter_posed_thumb_carrier')];common=next(z for z in ev if z['name']=='actual_parent_quarter_housing_common');wrist=p.read(ROOT/'cad/wrist_housing_guide_v1/generated/wrist_reference/scene.json')['parts'];old=p.read(ROOT/'cad/wrist_housing_guide_v1/selected2/generated/parts/scene.json')['parts'];newscene=p.read(HERE/'inboard/generated/assembly_scene.json');new=newscene['new_parts']+newscene['retained_parts']
    def scene(rows,commons=()):
        parts=[]
        for z,alpha in [(z,.27) for z in context]+[(z,.35) for z in wrist]+[(z,.95) for z in rows]+[(z,1.) for z in commons]:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.4,.6,.65]),alpha=alpha,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    a,b=scene(old,[common]),scene(new);scenes=[a,b];view=r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array([1.,-1.8,.6]),1.);ims=[]
    try:
        for s in scenes:
            s['_render_state']=r.make_renderer(s,{'v':view},1100,engine)
            for j,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'v',{'v':view},1100,engine);ims.append(im)
        sh=c.load([common])[0]['shape'];P=np.array(sh.Center().toTuple())-view['focus'];scale=1100/(2*view['parallel_scale_mm']);x=550+P@view['right']*scale;y=550-P@view['up']*scale;ImageDraw.Draw(ims[0]).ellipse((x-22,y-22,x+22,y+22),outline=(220,10,30),width=4)
        r.sheet([('이전 45도 통로: 부모25%에서 두 피복이 걸림',ims[0]),('창 안쪽 X26 / +Y 통로: 안내대 본체1개만 변경',ims[1])],2,1100,'실제 손목 창의 다른 방향으로 국부 걸림 교정','청록: 실제 지지부 후보 · 보라: 국부 피복2개 · 손/관절/모터/전기선 유지 · 긴 모터~엄지 연결·장력/손목운동은 미완').save(folder/'outlet_before_after.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    p.write(folder/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in folder.iterdir() if z.is_file()},before_actual_saved_common=True,whole_hand_drive_approved=False))
if __name__=='__main__':main()

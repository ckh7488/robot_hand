"""Actual wrist/root material sections, not AABB keep-out approximations."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
from outlet_survey import m,g,c,p,np,cq,ROOT,HERE
from PIL import Image,ImageDraw,ImageFont

def main():
    out=HERE/'reviews/sections.json';assert not out.exists();source={};meta=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];parts=c.load([z for z in meta if z['name'] in ['Palm_tapered_chassis','thumb_root_fixed_housing','thumb_root_spread_carrier']],source)
    wrist=c.load(p.read(ROOT/'cad/wrist_housing_guide_v1/generated/wrist_reference/scene.json')['parts'],source);cases=p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'];N=g.g.v.frames(g.DESIGN,np.zeros(20));panels=[];reports=[];font=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',18)
    for case in [cases[4],cases[-1]]:
        F=g.g.v.frames(g.DESIGN,case['angles_deg']);posed=[dict(z,shape=g.g.v.relocate(z['shape'],F[z['body']],N[z['body']])) for z in parts]+wrist
        for zheight in [-3.,8.]:
            im=Image.new('RGB',(620,690),'white');d=ImageDraw.Draw(im);d.text((15,8),case['id']+' / Z='+str(zheight)+'mm',fill='black',font=font)
            def pixel(x,y):return (40+(x-15)*8,640-(y+12)*8)
            for x in range(20,86,10):d.line([pixel(x,-12),pixel(x,60)],fill=(225,225,225));d.text(pixel(x,-15),str(x),fill='black',font=font)
            for y in range(-10,61,10):d.line([pixel(15,y),pixel(85,y)],fill=(225,225,225));d.text((3,pixel(15,y)[1]-8),str(y),fill='black',font=font)
            rr=[]
            for item in posed:
                color=(30,60,120) if item['name']=='thumb_root_spread_carrier' else (150,150,150) if item['name']=='Palm_tapered_chassis' else (85,145,110);section=cq.Workplane('XY').newObject([item['shape']]).section(zheight).val();curves=[]
                for edge in section.Edges():
                    points=[v.toTuple() for v in edge.sample(.08)[0]];curves.append(points)
                    if len(points)>1:d.line([pixel(v[0],v[1]) for v in points],fill=color,width=2)
                rr.append(dict(name=item['name'],section_polylines_mm=curves))
            d.line([pixel(19,-18),pixel(55,18)],fill=(160,45,190),width=3);d.ellipse((pixel(41.868,2.934)[0]-7,pixel(41.868,2.934)[1]-7,pixel(41.868,2.934)[0]+7,pixel(41.868,2.934)[1]+7),outline='red',width=2);panels.append(im);reports.append(dict(id=case['id'],angles_deg=case['angles_deg'],z_mm=zheight,parts=rr))
    sheet=Image.new('RGB',(1240,1430),(245,248,252));d=ImageDraw.Draw(sheet);d.text((12,10),'실제 재료 단면: 남색 엄지 회전부 / 회색 손바닥 / 녹색 손목 / 보라 기존 직선',fill='black',font=font)
    for i,im in enumerate(panels):sheet.paste(im,((i%2)*620,45+(i//2)*690))
    folder=HERE/'images';folder.mkdir(exist_ok=True);sheet.save(folder/'material_sections.png');p.write(out,dict(completed=True,source_sha256=source,cases=reports,actual_NATIVE_sections=True,all_hand_parts_included=False,material_boundary_not_freepass_approval=True));print('sections done',flush=True)
if __name__=='__main__':main()

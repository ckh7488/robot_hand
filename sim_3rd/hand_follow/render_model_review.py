"""Render the current CAD's open/fist geometry without webcam input."""
import argparse
from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from hand_follow.cad_hand import CadHand


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output',type=Path,default=Path('D:/Codex-work/robot-hand-workspace/outputs/hand_articulation'))
    options=parser.parse_args()
    options.output.mkdir(parents=True,exist_ok=True)
    robot=CadHand(options.repo,cad='webcam_articulation_v1')
    fist=np.array(robot.design['fist_deg'])
    font=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',28)
    small=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',22)
    sheet=Image.new('RGB',(1760,1660),'#141a24')
    draw=ImageDraw.Draw(sheet)
    panels=[('편 손 · 정면',np.zeros(20),0),('주먹 · 정면',fist,0),
            ('주먹 · 사선 — 엄지 두 마디 굽힘',fist,1),('주먹 · 측면 — 손끝과 손바닥 간격',fist,2)]
    try:
        for index,(title,pose,view) in enumerate(panels):
            robot.set(pose)
            closed=index>0
            rendered=robot.render(view,880,760,focus=[.012,.078,.04] if closed else None,
                                  distance=.36 if closed else None)
            x=(index%2)*880;y=(index//2)*810
            sheet.paste(Image.fromarray(rendered),(x,y+50))
            draw.text((x+24,y+12),title,font=font,fill='#edf5ff')
        draw.text((24,1625),'실제 CAD 렌더 · 자세마다 카메라 확대율 조정 · 웹캠 추종/실물 구동 결과 아님',font=small,fill='#a8c4d5')
        sheet.save(options.output/'open_and_fist.png')
        robot.set(fist)
        axes=Image.fromarray(robot.render(1,880,760,True,focus=[.026,.071,.04],distance=.30))
        board=Image.new('RGB',(880,870),'#141a24');board.paste(axes,(0,70))
        painter=ImageDraw.Draw(board)
        painter.text((20,12),'엄지 관절 축 · 주먹 자세',font=font,fill='#edf5ff')
        painter.text((20,818),'빨강/초록: 직각인 뿌리 2축   파랑/노랑: 바깥 굽힘 2축',font=small,fill='#c2d8e6')
        board.save(options.output/'thumb_axes.png')
        (options.output/'render_inputs.json').write_text(json.dumps(dict(cad_hashes=robot.hashes,
            fist_deg=fist.tolist(),panels=[p[0] for p in panels]),indent=2))
    finally:
        robot.close()
    print(f'Rendered {options.output}',flush=True)


if __name__=='__main__':main()

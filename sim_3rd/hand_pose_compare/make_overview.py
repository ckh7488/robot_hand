"""Assemble a local review figure from previously computed predictions."""
from pathlib import Path
import argparse
import json
import numpy as np
from PIL import Image, ImageDraw
from render_comparison import BG, TEXT, MUTED, font, skeleton


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--cases',type=Path,required=True)
    parser.add_argument('--results',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    predictions=sorted(args.results.glob('case_*/prediction.json'))
    image=Image.new('RGB',(1500,130+340*len(predictions)),BG)
    d=ImageDraw.Draw(image)
    d.text((22,15),'같은 사진으로 비교 · 로봇 변환 / 추가 보정 없음',font=font(29),fill=TEXT)
    for x,title in [(20,'입력 사진'),(510,'MediaPipe 원본 3D'),(1000,'WiLoR 원본 3D')]:
        d.text((x,68),title,font=font(25),fill=TEXT)
    for row,path in enumerate(predictions):
        result=json.loads(path.read_text(encoding='utf-8'))
        source=args.cases/path.parent.name
        case=json.loads((source/'case.json').read_text(encoding='utf-8'))
        y=120+340*row
        photo=Image.open(source/'camera.png').convert('RGB').resize((420,315))
        image.paste(photo,(70,y))
        d.text((15,y+10),str(row+1),font=font(28),fill=TEXT)
        wi=np.asarray(result['result']['wilor_preds']['pred_keypoints_3d'])[0] if result['status']=='tracked' else None
        image.paste(skeleton(case['detection']['world'],size=(470,315)),(510,y))
        image.paste(skeleton(wi,size=(470,315)),(1000,y))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    image.save(args.output)


if __name__=='__main__':
    main()

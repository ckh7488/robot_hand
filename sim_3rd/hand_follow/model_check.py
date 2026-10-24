"""Inspect the CAD's actual joints without webcam tracking or calibration."""
import argparse
from pathlib import Path
import time
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from hand_follow.cad_hand import CadHand

TITLE = 'Robot Hand - Joint and Fist Check'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cad', default='webcam_articulation_v1')
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    options = parser.parse_args()
    robot = CadHand(options.repo, cad=options.cad)
    fist = np.array(robot.design['fist_deg'])
    current = fist.copy()
    font = ImageFont.truetype('C:/Windows/Fonts/malgun.ttf', 19)
    small = ImageFont.truetype('C:/Windows/Fonts/malgun.ttf', 15)
    cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(TITLE, 1360, 900)
    sliders = ['Fingers open-close', 'Thumb root side', 'Thumb root fold', 'Thumb joint 2', 'Thumb joint 3']
    limits = robot.ranges[16:]
    def change(_):
        pass
    cv2.createTrackbar(sliders[0], TITLE, 100, 100, change)
    for index, name in enumerate(sliders[1:]):
        lo, hi = limits[index]
        cv2.createTrackbar(name, TITLE, round(fist[16+index]-lo), round(hi-lo), change)
    axes = True
    animate = False
    start = time.perf_counter()
    try:
        while True:
            if animate:
                fraction = .5-.5*np.cos((time.perf_counter()-start)*np.pi/2)
                path=robot.design['fist_path']
                current=np.array([np.interp(fraction,[p['fraction'] for p in path],
                                           [p['angles_deg'][i] for p in path]) for i in range(20)])
            else:
                current[:16] = fist[:16]*cv2.getTrackbarPos(sliders[0],TITLE)/100
                for index,name in enumerate(sliders[1:]):
                    current[16+index] = cv2.getTrackbarPos(name,TITLE)+limits[index,0]
            robot.set(current)
            canvas = Image.new('RGB',(1440,820),'#141a24')
            draw = ImageDraw.Draw(canvas)
            draw.text((22,14),'관절·주먹 모델 검사  /  카메라 입력 없음',font=font,fill='#f0f4fa')
            draw.text((22,46),'빨강: 뿌리 좌우축  초록: 뿌리 접기축  파랑·노랑: 두 바깥 굽힘축',font=small,fill='#b9ccd9')
            canvas.paste(Image.fromarray(robot.render(1,720,660,axes)),(0,78))
            canvas.paste(Image.fromarray(robot.render(2,720,660,axes)),(720,78))
            draw.text((22,739),'F 주먹   O 펴기   A 자동 왕복   G 축 표시   Q 종료',font=font,fill='#d4e6f1')
            draw.text((22,773),'아래 막대로 각 관절을 따로 움직일 수 있습니다.  별도 형상 후보 · 제조/구동 승인 전',font=small,fill='#a1b7c9')
            cv2.imshow(TITLE,cv2.cvtColor(np.asarray(canvas),cv2.COLOR_RGB2BGR))
            key=cv2.waitKey(20)&255
            if key in (27,ord('q')) or cv2.getWindowProperty(TITLE,cv2.WND_PROP_VISIBLE)<1:
                break
            if key==ord('g'):
                axes=not axes
            if key==ord('a'):
                animate=not animate;start=time.perf_counter()
            if key in (ord('o'),ord('f')):
                animate=False;scale=0 if key==ord('o') else 1
                cv2.setTrackbarPos(sliders[0],TITLE,100*scale)
                for i,name in enumerate(sliders[1:]):
                    cv2.setTrackbarPos(name,TITLE,round(fist[16+i]*scale-limits[i,0]))
    finally:
        robot.close()
        cv2.destroyWindow(TITLE)


if __name__ == '__main__':
    main()

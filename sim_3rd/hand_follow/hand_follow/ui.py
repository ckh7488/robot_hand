"""One window for webcam, CAD hand, calibration and diagnostics."""
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]
TITLE = 'Robot Hand - Webcam + CAD'

class PreviewUI:

    def __init__(self, headless=False):
        font = Path('C:/Windows/Fonts/malgun.ttf')
        self.fonts = {s: ImageFont.truetype(str(font), s) if font.exists() else ImageFont.load_default() for s in (14, 16, 19, 25)}
        self.action = None
        self.text_cache = {}
        self.buttons = [(24, 680, 187, 724, 'C', '편 손 보정'), (199, 680, 362, 724, 'V', '주먹 보정'), (374, 680, 537, 724, 'T', '모델 테스트')]
        if not headless:
            cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(TITLE, 1368, 798)
            cv2.setMouseCallback(TITLE, self._mouse)

    def _mouse(self, event, x, y, flags, parameter):
        if event == cv2.EVENT_LBUTTONUP:
            for x1, y1, x2, y2, key, _ in self.buttons:
                if x1 <= x <= x2 and y1 <= y <= y2:
                    self.action = ord(key.lower())

    def compose(self, frame, detection, rendered, pose, pipeline, timing, mode, view, debug=False):
        canvas = np.full((840, 1440, 3), [20, 26, 36], np.uint8)
        canvas[76:836, 560:1440] = rendered
        if frame is not None:
            camera = frame.copy()
            if detection is not None and detection.get('image') is not None:
                h, w = camera.shape[:2]
                pts = np.rint(detection['image'] * [w, h]).astype(int)
                for a, b in CONNECTIONS:
                    cv2.line(camera, tuple(pts[a]), tuple(pts[b]), (120, 226, 100), 2, cv2.LINE_AA)
                for p in pts:
                    cv2.circle(camera, tuple(p), 3, (60, 180, 250), -1, cv2.LINE_AA)
            scale = min(512 / camera.shape[1], 384 / camera.shape[0])
            camera = cv2.resize(camera, (round(camera.shape[1] * scale), round(camera.shape[0] * scale)))
            y, x = (111 + (384 - camera.shape[0]) // 2, 24 + (512 - camera.shape[1]) // 2)
            canvas[y:y + camera.shape[0], x:x + camera.shape[1]] = cv2.cvtColor(camera, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(canvas)
        draw = ImageDraw.Draw(image)

        def text(x, y, content, size=16, color='#c4ceda'):
            key = (content, size, color)
            if key not in self.text_cache:
                left, top, right, bottom = self.fonts[size].getbbox(content)
                label = Image.new('RGBA', (max(1, right-left), max(1, bottom-top)))
                ImageDraw.Draw(label).text((-left, -top), content, font=self.fonts[size], fill=color)
                if len(self.text_cache) >= 512:
                    del self.text_cache[next(iter(self.text_cache))]
                self.text_cache[key] = label, left, top
            label, left, top = self.text_cache[key]
            image.paste(label, (x+left, y+top), label)
        text(24, 19, '로봇 손 따라 하기', 25, '#f2f6fc')
        text(254, 28, '웹캠 → 손 관절 → CAD', 16, '#8ea3b9')
        text(580, 25, 'FORM V1  ·  실제 CAD 120부품 / 20관절', 19, '#e4edf7')
        label = '모델 주먹 테스트 · 카메라 추종 일시정지' if mode == 'test' else '손 인식 중' if pose['tracked'] else '손이 보이지 않음 · 마지막 자세 유지'
        text(24, 78, label, 16, '#62dbc2' if pose['tracked'] and mode != 'test' else '#e8bc70')
        text(24, 510, f"카메라 {timing['camera_fps']:.1f} fps   처리 {timing['processing_fps']:.1f} fps", 19)
        text(24, 540, f"손 인식 {timing['track_ms']:.0f} ms   입력→각도 {timing['age_ms']:.0f} ms", 16, '#8ea3b9')
        text(24, 576, '손가락 굽힘', 14, '#8ea3b9')
        for i, label in enumerate(['검지', '중지', '약지', '소지', '엄지']):
            x = 24 + i * 104
            text(x, 601, label, 14)
            value = np.clip(np.mean(pose['command_deg'][i * 4 + 1:i * 4 + 4]) / 85, 0, 1)
            draw.rounded_rectangle((x, 629, x + 88, 638), radius=4, fill='#344154')
            if value > 0:
                draw.rounded_rectangle((x, 629, x + max(5, int(88 * value)), 638), radius=4, fill='#55c9b1')
        for x1, y1, x2, y2, key, label in self.buttons:
            draw.rounded_rectangle((x1, y1, x2, y2), radius=7, fill='#263a4e', outline='#41576c')
            text(x1 + 10, y1 + 11, f'{key}  {label}', 16, '#eaf0f7')
        message = pose['warning'] or pipeline.message or 'C 편 손 → V 주먹 보정은 선택 사항입니다'
        if pose.get('estimated_joints') and not pose['warning'] and not pipeline.capture_kind:
            names = '·'.join(['검지', '중지', '약지', '소지'][i // 4] for i in pose['estimated_joints'])
            message = f'{names} 끝마디는 가림으로 추정 중 · X 초기화'
        text(24, 650, message[:43], 14, '#e8bc70' if pose['warning'] else '#92b6c9')
        text(24, 743, '1 정면  2 사선  3 측면   H 손 재선택', 14)
        text(24, 770, 'X 보정 초기화   F 필터   D 진단   Q 종료', 14)
        text(24, 801, '수치 로그 자동 기록 · 카메라 영상은 저장하지 않습니다', 14, '#788ea4')
        text(584, 788, '화면은 관절 자세 미리보기입니다', 14, '#8ea3b9')
        text(1210, 788, ['정면', '사선', '측면'][view % 3], 14, '#8ea3b9')
        if debug and pose['raw_deg'] is not None:
            draw.rounded_rectangle((580, 92, 1418, 212), radius=8, fill='#1c2b3b')
            text(594, 102, '검지: 인식각 → 보정각 → 적용각 (벌림 / 뿌리 / 가운데 / 끝)', 16)
            for y, key in [(131, 'raw_deg'), (155, 'mapped_deg'), (179, 'command_deg')]:
                text(594, y, key + '   ' + '  '.join((f'{v:6.1f}' for v in pose[key][:4])), 16)
            text(1100, 151, f"관절 제한 {len(pose['clipped_joints'])}개", 16)
            if pose.get('thumb_fit'):
                text(1100, 179, f"엄지 위치 오차 {pose['thumb_fit']['tip_error_mm']:.0f} mm", 16)
        return np.asarray(image)

    def show(self, *args, **kwargs):
        image = self.compose(*args, **kwargs)
        cv2.imshow(TITLE, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1) & 255
        if self.action is not None:
            key, self.action = (self.action, None)
        try:
            if cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) < 1:
                return ord('q')
        except cv2.error:
            return ord('q')
        return key

    def close(self):
        cv2.destroyAllWindows()

"""One window for webcam, CAD hand, calibration and diagnostics."""
from pathlib import Path
import time
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from .raw_preview import render_raw, FINGERS
CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]
TITLE = 'Robot Hand - Webcam + CAD'

class PreviewUI:

    def __init__(self, headless=False):
        cv2.setNumThreads(1)
        font = Path('C:/Windows/Fonts/malgun.ttf')
        self.fonts = {s: ImageFont.truetype(str(font), s) if font.exists() else ImageFont.load_default() for s in (14, 16, 19, 25)}
        self.action = None
        self.text_cache = {}
        self.background = Image.new('RGB', (1680, 900), (20, 26, 36))
        self.static_ready = False
        self.timing_label_time = 0.
        self.display_timing = None
        self.capture_count = 0
        self.notice, self.notice_until = '', 0.
        self.last_canvas = None
        self.buttons = [(24, 680, 187, 724, 'C', '편 손 보정'), (199, 680, 362, 724, 'V', '주먹 보정'),
                        (374, 680, 537, 724, 'T', '모델 테스트'), (24, 736, 537, 780, 'S', '오류 자세 저장'),
                        (574, 800, 730, 840, '1', '정면'), (744, 800, 900, 840, '2', '사선'),
                        (914, 800, 1070, 840, '3', '측면')]
        if not headless:
            cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(TITLE, 1596, 855)
            cv2.setMouseCallback(TITLE, self._mouse)

    def _mouse(self, event, x, y, flags, parameter):
        if event == cv2.EVENT_LBUTTONUP:
            for x1, y1, x2, y2, key, _ in self.buttons:
                if x1 <= x <= x2 and y1 <= y <= y2:
                    self.action = ord(key.lower())

    def notify(self, message, duration=4):
        self.notice, self.notice_until = message, time.perf_counter()+duration

    def compose(self, frame, detection, rendered, pose, pipeline, timing, mode, view, debug=False):
        now = time.perf_counter()
        if self.display_timing is None or now-self.timing_label_time >= .25:
            self.display_timing = dict(timing)
            self.timing_label_time = now
        timing = self.display_timing
        image = self.background.copy()
        # Fit without stretching: CAD and raw points have independent physical
        # scales, but neither preview changes the hand's proportions.
        scale = min(532 / rendered.shape[1], 620 / rendered.shape[0])
        rendered = cv2.resize(rendered, (round(rendered.shape[1]*scale), round(rendered.shape[0]*scale)), interpolation=cv2.INTER_LINEAR)
        image.paste(Image.fromarray(rendered), (1124+(532-rendered.shape[1])//2, 111+(620-rendered.shape[0])//2))
        raw, raw_state = render_raw(detection, view, width=532, height=620)
        image.paste(Image.fromarray(raw), (566, 111))
        if frame is not None:
            camera = frame.copy()
            if detection is not None and detection.get('image') is not None:
                h, w = camera.shape[:2]
                pts = np.rint(np.asarray(detection['image']) * [w, h]).astype(int)
                for a, b in CONNECTIONS:
                    cv2.line(camera, tuple(pts[a]), tuple(pts[b]), (120, 226, 100), 2, cv2.LINE_AA)
                for p in pts:
                    cv2.circle(camera, tuple(p), 3, (60, 180, 250), -1, cv2.LINE_AA)
            scale = min(512 / camera.shape[1], 384 / camera.shape[0])
            camera = cv2.resize(camera, (round(camera.shape[1] * scale), round(camera.shape[0] * scale)))
            y, x = (111 + (384 - camera.shape[0]) // 2, 24 + (512 - camera.shape[1]) // 2)
            image.paste(Image.fromarray(cv2.cvtColor(camera, cv2.COLOR_BGR2RGB)), (x, y))
        draw = ImageDraw.Draw(image)

        def text(x, y, content, size=16, color='#c4ceda', static=False):
            if static and self.static_ready:
                return
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
            if static:
                self.background.paste(label, (x+left, y+top), label)
        text(24, 19, '로봇 손 따라 하기', 25, '#f2f6fc', static=True)
        text(254, 28, '사진 · 인식 · 로봇 비교', 16, '#8ea3b9', static=True)
        text(584, 25, 'MediaPipe 원본 3D', 25, '#e4edf7', static=True)
        text(1142, 25, '로봇 손', 25, '#e4edf7', static=True)
        text(584, 78, '인식된 뼈대 · 로봇용 보정 전', 16, '#62dbc2', static=True)
        robot_label = '모델 주먹 테스트 · 웹캠 추종 일시정지' if mode == 'test' else '마디 자세 맞춤 · 웹캠 추종' if pipeline.retarget_mode == 'shape' else '웹캠 추종'
        text(1142, 78, robot_label, 16, '#e8bc70' if mode == 'test' else '#8ea3b9')
        if raw_state != 'tracked':
            text(658, 377, '손 인식 없음' if raw_state == 'missing' else '원본 3D를 표시할 수 없음', 19, '#e8bc70')
            text(658, 411, '새 관측을 기다리는 중', 16, '#8ea3b9')
        label = '모델 주먹 테스트 · 카메라 추종 일시정지' if mode == 'test' else '손 인식 중' if pose['tracked'] else '손이 보이지 않음 · 마지막 자세 유지'
        if now < self.notice_until:
            label = self.notice
        text(24, 78, label, 16, '#62dbc2' if pose['tracked'] and mode != 'test' else '#e8bc70')
        text(24, 510, '카메라', 19, static=True)
        text(277, 510, '화면', 19, static=True)
        text(24, 540, '손 인식', 16, '#8ea3b9', static=True)
        text(277, 540, '입력→각도', 16, '#8ea3b9', static=True)
        text(24, 576, '손가락 굽힘', 14, '#8ea3b9', static=True)
        for i, label in enumerate(['검지', '중지', '약지', '소지', '엄지']):
            x = 24 + i * 104
            text(x, 601, label, 14, static=True)
            value = np.clip(np.mean(pose['command_deg'][i * 4 + 1:i * 4 + 4]) / 85, 0, 1)
            draw.rounded_rectangle((x, 629, x + 88, 638), radius=4, fill='#344154')
            if value > 0:
                draw.rounded_rectangle((x, 629, x + max(5, int(88 * value)), 638), radius=4, fill='#55c9b1')
        for x1, y1, x2, y2, key, label in self.buttons:
            if not self.static_ready:
                for painter in (draw, ImageDraw.Draw(self.background)):
                    painter.rounded_rectangle((x1, y1, x2, y2), radius=7, fill='#263a4e', outline='#41576c')
            text(x1 + 10, y1 + 11, f'{key}  {label}', 16, '#eaf0f7', static=True)
        message = pose['warning'] or pipeline.message or 'C 편 손 → V 주먹 보정은 선택 사항입니다'
        if pose.get('estimated_joints') and not pose['warning'] and not pipeline.capture_kind:
            names = '·'.join(['검지', '중지', '약지', '소지'][i // 4] for i in pose['estimated_joints'])
            message = (f'{names} 끝마디 위치로 보완 중' if pose.get('finger_fit') else
                       f'{names} 끝마디는 가림으로 추정 중 · X 초기화')
        text(24, 650, message[:43], 14, '#e8bc70' if pose['warning'] else '#92b6c9')
        text(400, 748, f'{self.capture_count}건 저장', 16, '#62dbc2')
        text(24, 800, '1 정면  2 사선  3 측면   H 손 재선택', 14, static=True)
        text(24, 825, 'X 보정 초기화   F 필터   D 진단   Q 종료', 14, static=True)
        text(24, 856, '평소 수치만 기록 · S를 누를 때만 사진 저장', 14, '#788ea4', static=True)
        for i, (label, (_, color)) in enumerate(zip(['엄지', '검지', '중지', '약지', '소지'], FINGERS)):
            text(584+i*98, 737, label, 16, color, static=True)
        text(584, 769, '손 방향만 정렬 · 길이·굽힘·떨림 보정 없음', 14, '#8ea3b9', static=True)
        text(1142, 737, f"변환된 관절 자세 · 필터 {'켬' if pipeline.filter_on else '끔'}", 16, '#8ea3b9')
        text(1142, 769, '화면은 관절 자세 미리보기입니다', 14, '#8ea3b9', static=True)
        text(1142, 810, f"두 3D 보기: {['정면', '사선', '측면'][view % 3]}", 16, '#62dbc2')
        text(584, 856, 'S 저장에 사진·원본 3D·로봇 비교 화면이 함께 남습니다', 14, '#788ea4', static=True)
        if debug and pose['raw_deg'] is not None:
            draw.rounded_rectangle((580, 92, 1418, 212), radius=8, fill='#1c2b3b')
            text(594, 102, '검지: 인식각 → 보정각 → 적용각 (벌림 / 뿌리 / 가운데 / 끝)', 16)
            for y, key in [(131, 'raw_deg'), (155, 'mapped_deg'), (179, 'command_deg')]:
                text(594, y, key + '   ' + '  '.join((f'{v:6.1f}' for v in pose[key][:4])), 16)
            limited=(set(pose['clipped_joints']) | set(pose.get('finger_fit',{}).get('active_limits',[])) |
                     set(pose.get('shape_fit',{}).get('active_limits',[])) | set(pose.get('thumb_fit',{}).get('active_limits',[])))
            text(1100, 151, f"관절 제한 {len(limited)}개", 16)
            if 'tip_error_mm' in pose.get('thumb_fit',{}):
                text(1100, 179, f"엄지 위치 오차 {pose['thumb_fit']['tip_error_mm']:.0f} mm", 16)
            elif pose.get('thumb_fit',{}).get('estimated_root_posture'):
                text(1100,179,'엄지 뿌리 자세 보완',16)
        self.static_ready = True
        canvas = np.array(image)
        # Changing ASCII counters do not need expensive Korean glyph layout
        # on every timing update. Cache Korean labels and draw digits directly.
        for value, position, scale, color in (
                (f"{timing['camera_fps']:.1f} fps", (102, 532), .55, (196,206,218)),
                (f"{timing['processing_fps']:.1f} fps", (335, 532), .55, (196,206,218)),
                (f"{timing['track_ms']:.0f} ms", (96, 559), .48, (142,163,185)),
                (f"{timing['age_ms']:.0f} ms", (380, 559), .48, (142,163,185))):
            cv2.putText(canvas,value,position,cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
        return canvas

    def show(self, *args, **kwargs):
        image = self.compose(*args, **kwargs)
        self.last_canvas = image
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

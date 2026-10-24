"""Landmark-to-pose processing and robust, optional two-pose calibration."""
from pathlib import Path
import time
import numpy as np
from .retarget import Calibration, raw_angles
from .filters import OneEuro
from .thumb import ThumbRetargeter

FLEX = np.array([i for i in range(16) if i % 4 != 0])
FIST = np.array([0, 80, 95, 70] * 4 + [-55, 50, 0, 0], float)


class PosePipeline:
    def __init__(self, design, ranges, calibration_path, beta=.025, min_cutoff=2.0):
        self.design = design
        self.ranges = ranges
        self.calibration_path = Path(calibration_path)
        self.calibration = Calibration.load(self.calibration_path) if self.calibration_path.exists() else Calibration()
        self.open_calibrated = self.calibration_path.exists()
        self.fist_calibrated = bool(np.any(np.abs(self.calibration.gain - 1) > .01))
        self.filter = OneEuro(20, min_cutoff, beta)
        self.filter_on = True
        self.command = np.zeros(20)
        self.last_seen = None
        self.last_hand = None
        self.capture_kind = None
        self.capture_started = None
        self.samples = []
        self.message = ''
        self.thumb = ThumbRetargeter(design, ranges) if 'fingers' in design else None

    def start_calibration(self, kind):
        if kind == 'fist' and not self.open_calibrated:
            self.capture_kind = None
            self.message = '먼저 C로 편 손을 보정하세요'
            return
        self.capture_kind, self.capture_started, self.samples = kind, time.perf_counter(), []
        self.message = '손가락을 붙여 편 손을 유지하세요' if kind == 'open' else '주먹을 쥔 상태를 유지하세요'

    def clear_calibration(self):
        self.calibration = Calibration()
        self.open_calibrated = self.fist_calibrated = False
        if self.calibration_path.exists():
            self.calibration_path.unlink()
        self.filter.reset()
        if self.thumb:
            self.thumb.reset()
        self.message = '보정을 초기화했습니다'

    def _capture(self, raw, now):
        if self.capture_kind is None:
            return
        elapsed = now - self.capture_started
        if elapsed > 5:
            self.capture_kind = None
            self.message = '보정 실패: 손을 잘 보이게 한 뒤 다시 시도하세요'
            return
        # Give the user half a second to release the key before collecting.
        if elapsed < .5:
            return
        self.samples.append((now, raw.copy()))
        self.samples = [(stamp, value) for stamp, value in self.samples if now - stamp <= 1.4]
        if elapsed < 1.8 or len(self.samples) < 8:
            return
        samples = np.asarray([value for stamp, value in self.samples])
        med = np.median(samples, axis=0)
        # Median absolute deviation rejects transition outliers without allowing
        # continuously moving hands to define an endpoint.
        mad = np.median(np.abs(samples[:, FLEX] - med[FLEX]), axis=0)
        if np.median(mad) > 7 or np.count_nonzero(mad > 12) > 2:
            self.message = '보정 중: 손을 잠시 멈춰 주세요'
            return
        if self.capture_kind == 'open':
            if np.median(np.abs(med[FLEX])) > 25:
                self.message = '편 손이 아닙니다. 손가락을 펴 주세요'
                return
            self.calibration = Calibration()
            self.calibration.capture_open_hand(med)
            self.open_calibrated, self.fist_calibrated = True, False
            self.message = '편 손 보정 완료 · 필요하면 V로 주먹 보정'
        else:
            movement = med - self.calibration.offset
            independent = np.array([i for i in FLEX if i % 4 != 3])
            if np.any(movement[independent] < 15):
                self.message = '주먹 각도가 충분히 잡히지 않습니다. 손을 조금 돌려 주세요'
                return
            # An occluded DIP can have nearly identical open/fist depth. Do
            # not reject all useful joints or amplify that noise without bound.
            # Only these unobservable DIPs use a declared PIP-based estimate.
            self.calibration.coupled_dips = [i for i in (3, 7, 11, 15) if movement[i] < FIST[i] / 4]
            observed = np.array([i for i in FLEX if i not in self.calibration.coupled_dips])
            self.calibration.gain[observed] = np.clip(FIST[observed] / movement[observed], .5, 4)
            self.fist_calibrated = True
            self.message = '주먹 보정 완료'
            if self.calibration.coupled_dips:
                labels = ['검지', '중지', '약지', '소지']
                names = '·'.join(labels[i // 4] for i in self.calibration.coupled_dips)
                self.message += f' · {names} 끝마디 추정 사용'
        self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
        self.calibration.save(self.calibration_path)
        self.filter.reset()
        self.capture_kind = None

    def update(self, detection, capture_time, now=None):
        now = time.perf_counter() if now is None else now
        result = dict(tracked=False, command_deg=self.command.copy(), raw_deg=None,
                      mapped_deg=None, filtered_deg=None, clipped_joints=[], warning='',
                      estimated_joints=self.calibration.coupled_dips.copy())
        if detection is None:
            if self.last_seen is None or now - self.last_seen > .25:
                self.filter.reset()
                if self.thumb:
                    self.thumb.reset()
            if self.capture_kind and now - self.capture_started > 5:
                self.capture_kind = None
                self.message = '보정 실패: 손이 보이지 않습니다'
            return result
        try:
            thumb = self.design['thumb']
            raw = raw_angles(detection['world'], detection['handedness'],
                             thumb['base_yaw_deg'], thumb['second_yaw_deg'])
        except ValueError as exc:
            result['warning'] = str(exc)
            return result
        if detection['handedness'] != self.last_hand:
            self.filter.reset()
            if self.thumb:
                self.thumb.reset()
        self.last_hand, self.last_seen = detection['handedness'], now
        self._capture(raw, now)
        result['estimated_joints'] = self.calibration.coupled_dips.copy()
        mapped = self.calibration.apply(raw)
        # Spread is poorly observed when the proximal bone faces the camera.
        # Progressively relax it toward neutral in a closed fist, without
        # coupling the four fingers' flexion or the PIP/DIP commands.
        for offset in range(0, 16, 4):
            closure = max(np.clip((raw[offset + 1] - 45) / 35, 0, 1),
                          np.clip((mapped[offset + 2] - 45) / 40, 0, 1))
            mapped[offset] *= 1 - closure
        if self.thumb:
            try:
                mapped[16:], result['thumb_fit'] = self.thumb.solve(
                    detection['world'], detection['handedness'],
                    np.clip(mapped, *self.ranges.T), raw[16:])
            except ValueError as exc:
                result['warning'] = str(exc)
                return result
        filtered = self.filter(mapped, capture_time) if self.filter_on else mapped.copy()
        command = np.clip(filtered, *self.ranges.T)
        clipped = np.flatnonzero(np.abs(command - filtered) > .1).tolist()
        warning = ''
        if result.get('thumb_fit', {}).get('tip_error_mm', 0) > 25:
            warning = '엄지 목표가 CAD 가동 범위를 벗어남'
        if np.sum(raw[[2, 6, 10, 14]] < -25) >= 3:
            warning = '굽힘 방향 이상 · H로 손 재선택 / 진단 로그 확인'
        self.command = command
        result.update(tracked=True, raw_deg=raw, mapped_deg=mapped, filtered_deg=filtered,
                      command_deg=command.copy(), clipped_joints=clipped, warning=warning)
        return result

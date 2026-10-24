"""Landmark-to-pose processing and robust, optional two-pose calibration."""
from pathlib import Path
import time
import numpy as np
from .retarget import Calibration, raw_angles
from .filters import OneEuro
from .thumb import ThumbRetargeter
from .fingers import FingerRetargeter, observations
from .motion import MotionStabilizer
from .observation import ObservationGuard
from .continuity import ContinuousPoseFilter

FLEX = np.array([i for i in range(16) if i % 4 != 0])
FIST = np.array([0, 80, 95, 70] * 4 + [-55, 50, 0, 0], float)


class PosePipeline:
    def __init__(self, design, ranges, calibration_path, beta=None, min_cutoff=None, retarget_mode='angles', stabilize=False, robust=None):
        self.design = design
        self.ranges = ranges
        self.fist = np.asarray(design.get('fist_deg', FIST), float)
        self.calibration_path = Path(calibration_path)
        self.calibration = Calibration.load(self.calibration_path) if self.calibration_path.exists() else Calibration()
        self.open_calibrated = self.calibration_path.exists()
        self.fist_calibrated = bool(np.any(np.abs(self.calibration.gain - 1) > .01))
        self.robust = retarget_mode == 'position' if robust is None else robust
        beta = (.012 if self.robust else .025) if beta is None else beta
        min_cutoff = (2.5 if self.robust else 2.) if min_cutoff is None else min_cutoff
        self.filter = OneEuro(20, min_cutoff, beta)
        self.filter_on = True
        self.motion = MotionStabilizer(ranges) if stabilize else None
        self.guard = ObservationGuard() if self.robust else None
        self.continuity = ContinuousPoseFilter(ranges,min_cutoff,beta) if self.robust else None
        self.last_mapped = np.zeros(20)
        self.previous_capture = None
        self.command = np.zeros(20)
        self.last_seen = None
        self.last_hand = None
        self.capture_kind = None
        self.capture_started = None
        self.samples = []
        self.message = ''
        self.thumb = ThumbRetargeter(design, ranges) if 'fingers' in design else None
        self.fingers = FingerRetargeter(design, ranges) if retarget_mode == 'position' and 'fingers' in design else None
        self.retarget_mode = retarget_mode
        if self.fingers and self.calibration.position_profile.get('cad_signature') != self.fingers.signature:
            self.calibration = Calibration()
            self.open_calibrated = self.fist_calibrated = False
            self.message = 'C 편 손 → V 주먹으로 손끝 위치를 보정하세요'
        elif self.fingers:
            self.fist_calibrated = 'fist_tips' in self.calibration.position_profile

    def start_calibration(self, kind):
        if kind == 'fist' and not self.open_calibrated:
            self.capture_kind = None
            self.message = '먼저 C로 편 손을 보정하세요'
            return
        self.capture_kind, self.capture_started, self.samples = kind, time.perf_counter(), []
        self.message = '손가락을 붙여 편 손을 유지하세요' if kind == 'open' else '주먹을 쥔 상태를 유지하세요'

    def reselect_hand(self):
        """Forget observation identity, preserving the displayed pose on return."""
        if self.guard:
            self.guard.reset()
        if self.continuity:
            self.continuity.hold(self.previous_capture)
        self.last_hand = None
        self.filter.reset()

    def clear_calibration(self):
        self.calibration = Calibration()
        self.open_calibrated = self.fist_calibrated = False
        if self.calibration_path.exists():
            self.calibration_path.unlink()
        self.filter.reset()
        if self.guard:
            self.guard.reset()
        if self.thumb:
            self.thumb.reset()
        if self.fingers:
            self.fingers.reset()
        self.message = '보정을 초기화했습니다'

    def _capture(self, raw, now, observed=None):
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
        self.samples.append((now, raw.copy(), observed))
        self.samples = [sample for sample in self.samples if now - sample[0] <= 1.4]
        if elapsed < 1.8 or len(self.samples) < 8:
            return
        samples = np.asarray([sample[1] for sample in self.samples])
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
            if self.fingers and all(sample[2] is not None for sample in self.samples):
                self.calibration.position_profile = dict(schema=1, cad_signature=self.fingers.signature,
                    open_tips=np.median([sample[2]['tips'] for sample in self.samples], axis=0).tolist())
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
            self.calibration.coupled_dips = [i for i in (3, 7, 11, 15) if movement[i] < self.fist[i] / 4]
            self.calibration.dip_ratios = {i: float(self.fist[i] / self.fist[i - 1]) for i in self.calibration.coupled_dips}
            observed = np.array([i for i in FLEX if i not in self.calibration.coupled_dips])
            self.calibration.gain[observed] = np.clip(self.fist[observed] / movement[observed], .5, 4)
            if self.fingers and self.calibration.position_profile and all(sample[2] is not None for sample in self.samples):
                self.calibration.position_profile['fist_tips'] = np.median(
                    [sample[2]['tips'] for sample in self.samples], axis=0).tolist()
                self.calibration.position_profile['fist_flex_deg'] = med[:16].reshape(4,4)[:,1:].tolist()
            self.fist_calibrated = True
            self.message = '주먹 보정 완료'
            if self.calibration.coupled_dips:
                labels = ['검지', '중지', '약지', '소지']
                names = '·'.join(labels[i // 4] for i in self.calibration.coupled_dips)
                self.message += f' · {names} 끝마디 추정 사용'
        self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
        self.calibration.save(self.calibration_path)
        self.filter.reset()
        if self.fingers:
            self.fingers.reset()
        self.capture_kind = None

    def update(self, detection, capture_time, now=None):
        now = time.perf_counter() if now is None else now
        result = dict(tracked=False, command_deg=self.command.copy(), raw_deg=None,
                      mapped_deg=None, filtered_deg=None, clipped_joints=[], warning='',
                      estimated_joints=self.calibration.coupled_dips.copy())
        if detection is None:
            if self.continuity:
                self.continuity.hold(capture_time if capture_time is not None else now)
            if self.motion:
                self.motion.hold(capture_time if capture_time is not None else now)
            if not self.robust and (self.last_seen is None or now - self.last_seen > .25):
                self.filter.reset()
                if self.thumb:
                    self.thumb.reset()
                if self.fingers:
                    self.fingers.reset()
            if self.capture_kind and now - self.capture_started > 5:
                self.capture_kind = None
                self.message = '보정 실패: 손이 보이지 않습니다'
            return result
        try:
            if self.robust and self.previous_capture is not None and capture_time <= self.previous_capture:
                result['warning'] = '오래된 입력 건너뜀'
                return result
            thumb = self.design['thumb']
            original_raw = raw_angles(detection['world'], detection['handedness'],
                             thumb['base_yaw_deg'], thumb['second_yaw_deg'])
            world = detection['world']
            quality = dict(held_fingers=[])
            if self.guard:
                world, quality = self.guard.update(world, detection['handedness'], capture_time)
                result['observation_quality'] = quality
            raw = raw_angles(world, detection['handedness'],
                             thumb['base_yaw_deg'], thumb['second_yaw_deg'])
        except ValueError as exc:
            if self.continuity:
                self.continuity.hold(capture_time)
            result['warning'] = str(exc)
            return result
        if detection['handedness'] != self.last_hand:
            self.filter.reset()
            if self.continuity:
                self.continuity.hold(capture_time)
            if self.thumb:
                self.thumb.reset()
            if self.fingers:
                self.fingers.reset()
        self.last_hand, self.last_seen = detection['handedness'], now
        dt = None if self.previous_capture is None else max(.001, min(.1, capture_time-self.previous_capture))
        self.previous_capture = capture_time
        try:
            observed = observations(world, detection['handedness']) if self.fingers else None
        except ValueError as exc:
            if self.continuity:
                self.continuity.hold(capture_time)
            result['warning'] = str(exc)
            return result
        # Unreliable fingers must not redefine the user's calibration endpoints.
        if not quality['held_fingers']:
            self._capture(raw, now, observed)
        elif self.capture_kind and now-self.capture_started > 5:
            self.capture_kind = None
            self.message = '보정 실패: 손 좌표가 불안정합니다'
        result['estimated_joints'] = self.calibration.coupled_dips.copy()
        mapped = self.calibration.apply(raw)
        # Spread is poorly observed when the proximal bone faces the camera.
        # Progressively relax it toward neutral in a closed fist, without
        # coupling the four fingers' flexion or the PIP/DIP commands.
        if self.fingers:
            mapped[:16], result['finger_fit'] = self.fingers.solve(observed, raw, self.calibration, continuous=self.robust)
            result['estimated_joints'] = result['finger_fit']['estimated_joints']
        else:
            for offset in range(0, 16, 4):
                closure = max(np.clip((raw[offset + 1] - 45) / 35, 0, 1),
                              np.clip((mapped[offset + 2] - 45) / 40, 0, 1))
                mapped[offset] *= 1 - closure
        for finger in quality['held_fingers']:
            if finger < 4:
                mapped[4*finger:4*finger+4] = self.last_mapped[4*finger:4*finger+4]
        if self.thumb and 4 not in quality['held_fingers']:
            try:
                mapped[16:], result['thumb_fit'] = self.thumb.solve(
                    world, detection['handedness'],
                    np.clip(mapped, *self.ranges.T), raw[16:], dt=(dt or 1/30) if self.robust or self.motion else None)
            except ValueError as exc:
                if self.continuity:
                    self.continuity.hold(capture_time)
                result['warning'] = str(exc)
                return result
        for finger in quality['held_fingers']:
            mapped[4*finger:4*finger+4] = self.last_mapped[4*finger:4*finger+4]
        self.last_mapped = mapped.copy()
        if self.fingers:
            self.fingers.previous = mapped[:16].copy()
        if self.thumb:
            self.thumb.previous = mapped[16:].copy()
        if self.motion:
            filtered = self.motion(mapped, capture_time, smooth=self.filter_on)
            result['motion'] = dict(limited_joints=self.motion.limited_joints,
                                   velocity_deg_s=self.motion.velocity.copy())
        elif self.continuity:
            filtered = self.continuity(mapped, capture_time, smooth=self.filter_on)
            result['confirmed_jump_joints'] = self.continuity.held_joints
        else:
            filtered = self.filter(mapped, capture_time) if self.filter_on else mapped.copy()
        command = np.clip(filtered, *self.ranges.T)
        clipped = np.flatnonzero(np.abs(command - filtered) > .1).tolist()
        warning = ''
        if result.get('thumb_fit', {}).get('tip_error_mm', 0) > 25:
            warning = '엄지 목표가 CAD 가동 범위를 벗어남'
        if np.sum(raw[[2, 6, 10, 14]] < -25) >= 3:
            warning = '굽힘 방향 이상 · H로 손 재선택 / 진단 로그 확인'
        if quality['held_fingers']:
            names = '·'.join(['검지','중지','약지','소지','엄지'][i] for i in quality['held_fingers'])
            warning = f'{names} 좌표 불안정 · 해당 손가락 유지'
        self.command = command
        result.update(tracked=True, raw_deg=original_raw, accepted_raw_deg=raw, mapped_deg=mapped, filtered_deg=filtered,
                      command_deg=command.copy(), clipped_joints=clipped, warning=warning)
        return result

"""Session diagnostics: numeric inputs and commands, never webcam pixels."""
import csv
import datetime
import json
import os
from pathlib import Path
import time
import numpy as np
from .retarget import NAMES


def plain(value):
    if isinstance(value, np.ndarray):
        return np.round(value, 6).tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


class SessionLog:
    def __init__(self, root, metadata):
        root = Path(root)
        self.directory = root / (datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{os.getpid()}')
        self.directory.mkdir(parents=True, exist_ok=True)
        self.started = time.perf_counter()
        self.frames = self.detected = 0
        self.last_detail = self.last_flush = 0.
        self.ages = []
        self.csv_file = (self.directory / 'angles.csv').open('w', newline='', encoding='utf-8')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['t_capture_s', 't_command_s', 'handedness', 'score', 'mode', 'tracked'] +
                          [f'{kind}_{n}' for kind in ('raw', 'mapped', 'cmd', 'actual') for n in NAMES])
        self.details = (self.directory / 'landmarks.jsonl').open('w', encoding='utf-8')
        self.events = (self.directory / 'events.jsonl').open('w', encoding='utf-8')
        metadata = dict(metadata, pid=os.getpid(), started=datetime.datetime.now().isoformat(),
                        image_recording=False, joint_names=NAMES, log_schema=3)
        (self.directory / 'session.json').write_text(json.dumps(metadata, indent=2, default=plain), encoding='utf-8')
        (root / 'latest_session.json').write_text(json.dumps({'directory': str(self.directory), 'pid': os.getpid()}), encoding='utf-8')
        self.event('started')

    def event(self, name, **values):
        self.events.write(json.dumps(dict(event=name, elapsed_s=time.perf_counter() - self.started, **values), default=plain, ensure_ascii=False) + '\n')
        self.events.flush()

    def frame(self, seq, captured, command_time, detection, pose, actual, timings, mode='live'):
        self.frames += 1
        self.detected += int(pose['tracked'])
        self.ages.append(timings['age_ms']); self.ages = self.ages[-300:]
        row = [captured - self.started, command_time - self.started,
               detection['handedness'] if detection else '', detection['score'] if detection else 0,
               mode, pose['tracked']]
        for angles in (pose['raw_deg'], pose['mapped_deg'], pose['command_deg'], actual):
            row.extend(np.round(angles, 3) if angles is not None else [''] * 20)
        self.csv.writerow(row)
        # Every processed sample is needed to distinguish a one-frame input
        # outlier from mapping/filter behavior. Never record webcam pixels.
        self.details.write(json.dumps(dict(seq=seq, elapsed_s=command_time - self.started,
                                          capture_elapsed_s=captured-self.started, mode=mode,
                                          detection=detection, pose=pose, actual_deg=actual, **timings), default=plain) + '\n')
        if command_time - self.last_flush >= 1:
            self.csv_file.flush(); self.details.flush()
            self.status('running', mode=mode, tracked=pose['tracked'], **timings)
            self.last_flush = command_time

    def status(self, state, **values):
        status = dict(state=state, pid=os.getpid(), frames=self.frames, hand_frames=self.detected,
                      updated_at=time.time(), **values)
        temp = self.directory / 'status.tmp'
        temp.write_text(json.dumps(status, indent=2, default=plain), encoding='utf-8')
        temp.replace(self.directory / 'status.json')

    def close(self, error=None):
        self.event('failed' if error else 'stopped', error=error)
        self.status('failed' if error else 'stopped', error=error,
                    median_frame_age_ms=float(np.median(self.ages)) if self.ages else None)
        self.csv_file.close(); self.details.close(); self.events.close()

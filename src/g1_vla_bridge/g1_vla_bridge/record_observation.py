"""Causal observation sampling using the record export time and FK contract."""

from collections import deque
from dataclasses import dataclass
from functools import lru_cache
import importlib.util
from pathlib import Path
import sys
import threading
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory


@lru_cache(maxsize=None)
def record_tool(name):
    path = Path(get_package_share_directory('record')) / 'tools' / f'{name}.py'
    module_name = f'_vla_record_{name}'
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class ObservationTiming:
    reference_ns: int
    acquired_monotonic: float
    ages_s: dict[str, float]
    skew_s: float


@dataclass(frozen=True)
class Sample:
    stamp: float
    value: object
    received: float


@dataclass(frozen=True)
class AlignedSamples:
    reference: float
    images: dict
    joints: Sample


class ObservationBuffer:
    def __init__(self, slots, rate=30.0, max_age=0.1, capacity=32):
        if not np.isfinite(rate) or rate <= 0 or not np.isfinite(max_age) or max_age <= 0:
            raise ValueError('rate and max_age must be finite and positive')
        self.rate, self.max_age = rate, max_age
        self.images = {slot: deque(maxlen=capacity) for slot in slots}
        self.joints = deque(maxlen=2048)
        self.origin = None
        self.lock = threading.Lock()

    def add(self, slot, stamp, value, received):
        if not np.isfinite(stamp) or stamp <= 0:
            return
        with self.lock:
            queue = self.joints if slot == 'joints' else self.images[slot]
            if queue and stamp < queue[-1].stamp:
                return
            queue.append(Sample(stamp, value, received))

    def reset_camera(self, slot):
        with self.lock:
            self.images[slot].clear()

    def replace_camera(self, slot, samples):
        with self.lock:
            self.images[slot].clear()
            self.images[slot].extend(samples)

    def select(self, now, max_delay, available_until=None):
        with self.lock:
            queues = {**self.images, 'joints': self.joints}
            missing = [key for key, queue in queues.items() if not queue]
            if missing:
                raise RuntimeError(f'observation missing {missing}')
            latest = min(queue[-1].stamp for queue in queues.values())
            if available_until is not None:
                latest = min(latest, available_until)
            if self.origin is None:
                self.origin = latest
            reference = self.origin + np.floor((latest - self.origin) * self.rate + 1e-6) / self.rate
            if reference > now or now - reference > max_delay:
                raise RuntimeError('observation reference stale or future')
            selected = {}
            for key, queue in queues.items():
                stamps = np.maximum.accumulate([sample.stamp for sample in queue])
                index = int(np.searchsorted(stamps, reference, side='right')) - 1
                if index < 0 or reference - stamps[index] > self.max_age:
                    raise RuntimeError(f'observation sample age: {key}')
                selected[key] = queue[index]
            return AlignedSamples(reference, {key: selected[key] for key in self.images},
                                  selected['joints'])


class WristTimeline:
    def __init__(self, slot, capacity=32, fit_frames=900):
        source = {'left_wrist': 'wrist_left', 'right_wrist': 'wrist_right'}[slot]
        self.delay = record_tool('session_reader').CAMERA_DELAY_S[source]
        self.raw = deque(maxlen=fit_frames)
        self.frames = deque(maxlen=capacity)

    def append(self, stamp, frame, received):
        if not np.isfinite(stamp) or stamp <= 0:
            raise ValueError('invalid wrist packet timestamp')
        self.raw.append(stamp)
        self.frames.append((frame, received))
        fitted = record_tool('session_reader').fitted_pts(np.asarray(self.raw)) - self.delay
        fitted = np.maximum.accumulate(fitted)[-len(self.frames):]
        return [Sample(float(stamp), frame, received)
                for stamp, (frame, received) in zip(fitted, self.frames)]


def measured_state(model, base, tips, cameras, joints):
    fk = record_tool('urdf_fk')
    if not joints or not all(np.isfinite(value) for value in joints.values()):
        raise RuntimeError('invalid measured joints')
    poses = {side: fk.matrix_to_pose(model.poses(base, link, joints)[0])
             for side, link in tips.items()}
    camera_poses = {slot: model.poses(base, link, joints)[0]
                    for slot, link in cameras.items()}
    grippers = {side: float(joints[f'{side}_eccentric_joint']) for side in tips}
    return poses, camera_poses, grippers


def model_from_description(text):
    source = ET.fromstring(text)
    root = ET.Element('robot', name=source.get('name', 'robot'))
    root.extend(source.findall('joint'))
    return record_tool('urdf_fk').RobotModel.from_urdf(ET.tostring(root, encoding='unicode'))


class WristReader:
    def __init__(self, slot, url, buffer):
        import av

        self._av = av
        self.slot, self.url, self.buffer = slot, url, buffer
        self.error = ''
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        import time

        while not self.stop_event.is_set():
            self.buffer.reset_camera(self.slot)
            timeline = WristTimeline(self.slot)
            try:
                with self._av.open(self.url, timeout=(5.0, 3.0), options={
                        'rtsp_transport': 'tcp', 'flags': 'low_delay',
                        'use_wallclock_as_timestamps': '1'}) as container:
                    stream = container.streams.video[0]
                    stream.codec_context.thread_count = 2
                    for packet in container.demux(stream):
                        if self.stop_event.is_set():
                            break
                        for frame in packet.decode():
                            if frame.pts is None:
                                continue
                            stamp = float(frame.pts * frame.time_base)
                            if abs(time.time() - stamp) > 3.0:
                                raise RuntimeError('wrist packet clock stale or invalid')
                            samples = timeline.append(stamp, frame, time.monotonic())
                            self.buffer.replace_camera(self.slot, samples)
                            self.error = ''
                    if not self.stop_event.is_set():
                        raise RuntimeError('wrist stream ended')
            except Exception as error:
                self.error = type(error).__name__
                self.buffer.reset_camera(self.slot)
                self.stop_event.wait(1.0)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=9.0)
        if self.thread.is_alive():
            raise RuntimeError('wrist reader did not stop')

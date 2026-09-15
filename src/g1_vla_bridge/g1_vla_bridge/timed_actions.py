"""Request-relative predictions on a shared monotonic execution grid."""

import math

import numpy as np

from g1_vla_bridge.transforms import quat_slerp
from g1_vla_bridge.vla_backend import ActionChunk, SIDES


class TimedActions:

    def __init__(self, rate: float, alpha: float, origin: float) -> None:
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('action_rate_hz must be positive and finite')
        if not math.isfinite(alpha) or not 0 < alpha <= 1:
            raise ValueError('async_ema_alpha must be in (0, 1]')
        self.rate = rate
        self.alpha = alpha
        self.origin = origin
        self.end = origin
        self.consumed = -1
        self.samples: dict[int, tuple[dict, dict]] = {}
        self.last_merge: dict = {}
        self.merges = 0
        self.overlap_total = 0
        self.new_total = 0

    def merge(self, chunk: ActionChunk, requested: float, now: float) -> int:
        end = requested + chunk.horizon / self.rate
        first = max(self.consumed + 1,
                    math.ceil((now - self.origin) * self.rate - 1e-8))
        stop = math.ceil((end - self.origin) * self.rate - 1e-8)
        self.samples = {tick: value for tick, value in self.samples.items()
                        if tick >= first}
        overlap = 0
        for tick in range(first, stop):
            offset = max(0.0, (self.origin + tick / self.rate - requested) * self.rate)
            lower = min(int(math.floor(offset + 1e-8)), chunk.horizon - 1)
            upper = min(lower + 1, chunk.horizon - 1)
            fraction = min(1.0, max(0.0, offset - lower))
            poses, grippers = {}, {}
            previous = self.samples.get(tick)
            overlap += int(previous is not None)
            for side in SIDES:
                start, finish = chunk.poses[side][lower], chunk.poses[side][upper]
                pose = np.concatenate((
                    (1 - fraction) * start[:3] + fraction * finish[:3],
                    quat_slerp(start[3:], finish[3:], fraction)))
                if previous is not None:
                    old = previous[0][side]
                    pose[:3] = (1 - self.alpha) * old[:3] + self.alpha * pose[:3]
                    pose[3:] = quat_slerp(old[3:], pose[3:], self.alpha)
                poses[side] = pose
                grippers[side] = float(chunk.grippers[side][lower])
            self.samples[tick] = poses, grippers
        if stop > first:
            self.end = max(self.end, end)
        accepted = max(0, stop - first)
        self.merges += 1
        self.overlap_total += overlap
        self.new_total += accepted - overlap
        self.last_merge = {
            'accepted': accepted, 'overlap': overlap, 'new': accepted - overlap,
            'observation_to_response_s': now - requested,
            'prediction_remaining_s': max(0., end - now),
            'first_action_offset_s': (self.origin + first / self.rate - requested)
            if accepted else None,
            'responses': self.merges, 'overlap_total': self.overlap_total,
            'new_total': self.new_total,
        }
        return accepted

    def take(self, now: float):
        tick = math.floor((now - self.origin) * self.rate + 1e-8)
        if tick <= self.consumed:
            return None
        self.consumed = tick
        value = self.samples.get(tick)
        self.samples = {key: sample for key, sample in self.samples.items() if key > tick}
        return value

"""Request-relative predictions on a shared monotonic execution grid."""

import math

import numpy as np

from g1_vla_bridge.transforms import quat_slerp
from g1_vla_bridge.vla_backend import ActionChunk, SIDES


class TimedActions:

    def __init__(self, rate: float, alpha: float, origin: float,
                 minimum_overlap_actions: int = 0, first_offset_steps: int = 0,
                 execution_rate: float | None = None) -> None:
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('action_rate_hz must be positive and finite')
        if not math.isfinite(alpha) or not 0 < alpha <= 1:
            raise ValueError('async_ema_alpha must be in (0, 1]')
        if minimum_overlap_actions < 0:
            raise ValueError('async_min_overlap_actions must be non-negative')
        self.rate = rate
        self.execution_rate = rate if execution_rate is None else execution_rate
        if not math.isfinite(self.execution_rate) or self.execution_rate <= 0:
            raise ValueError('execution_rate_hz must be positive and finite')
        self.alpha = alpha
        self.minimum_overlap_actions = minimum_overlap_actions
        self.origin = origin
        if first_offset_steps not in (0, 1):
            raise ValueError('first_offset_steps must be 0 or 1')
        self.first_offset_steps = first_offset_steps
        self.end = origin
        self.consumed = -1
        self.samples: dict[int, tuple[dict, dict]] = {}
        self.last_merge: dict = {}
        self.merges = 0
        self.overlap_total = 0
        self.new_total = 0

    def merge(self, chunk: ActionChunk, requested: float, now: float,
              fallback_poses: dict[str, np.ndarray] | None = None) -> int:
        end = requested + chunk.horizon / self.rate
        first = max(self.consumed + 1,
                    math.ceil((max(now, requested + self.first_offset_steps / self.rate)
                               - self.origin) * self.execution_rate - 1e-8))
        stop = math.ceil((end - self.origin) * self.execution_rate - 1e-8)
        if self.first_offset_steps:
            stop = math.floor((end - self.origin) * self.execution_rate + 1e-8) + 1
        self.samples = {tick: value for tick, value in self.samples.items()
                        if tick >= first}
        accepted = max(0, stop - first)
        old_poses = [self.samples[tick][0] for tick in range(first, stop)
                 if tick in self.samples]
        overlap = len(old_poses)
        blend_count = min(accepted, max(overlap, self.minimum_overlap_actions))
        fallback = old_poses[-1] if old_poses else fallback_poses
        if blend_count and fallback is None:
            raise ValueError('async merge needs fallback poses when overlap is empty')
        for tick in range(first, stop):
            offset = max(0.0, (self.origin + tick / self.execution_rate - requested) * self.rate
                         - self.first_offset_steps)
            lower = min(int(math.floor(offset + 1e-8)), chunk.horizon - 1)
            upper = min(lower + 1, chunk.horizon - 1)
            fraction = min(1.0, max(0.0, offset - lower))
            poses, grippers = {}, {}
            previous = self.samples.get(tick)
            for side in SIDES:
                start, finish = chunk.poses[side][lower], chunk.poses[side][upper]
                pose = np.concatenate((
                    (1 - fraction) * start[:3] + fraction * finish[:3],
                    quat_slerp(start[3:], finish[3:], fraction)))
                if tick - first < blend_count:
                    old = (previous[0] if previous is not None else fallback)[side]
                    pose[:3] = (1 - self.alpha) * old[:3] + self.alpha * pose[:3]
                    pose[3:] = quat_slerp(old[3:], pose[3:], self.alpha)
                poses[side] = pose
                grippers[side] = float((1 - fraction) * chunk.grippers[side][lower]
                                       + fraction * chunk.grippers[side][upper])
            self.samples[tick] = poses, grippers
        if stop > first:
            self.end = max(self.end, end)
        self.merges += 1
        self.overlap_total += overlap
        self.new_total += accepted - overlap
        self.last_merge = {
            'accepted': accepted, 'overlap': overlap, 'blended': blend_count,
            'new': accepted - overlap,
            'observation_to_response_s': now - requested,
            'prediction_remaining_s': max(0., end - now),
            'first_action_offset_s': (self.origin + first / self.execution_rate - requested)
            if accepted else None,
            'responses': self.merges, 'overlap_total': self.overlap_total,
            'new_total': self.new_total,
        }
        return accepted

    def take(self, now: float):
        if self.first_offset_steps and now > self.end + 1e-8:
            self.samples.clear()
            return None
        tick = math.floor((now - self.origin) * self.execution_rate + 1e-8)
        if tick <= self.consumed:
            return None
        self.consumed = tick
        value = self.samples.get(tick)
        self.samples = {key: sample for key, sample in self.samples.items() if key > tick}
        return value

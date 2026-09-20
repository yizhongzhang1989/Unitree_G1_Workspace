"""Blend on the model grid, then interpolate only when executing."""

import math

import numpy as np

from g1_vla_bridge.transforms import quat_slerp
from g1_vla_bridge.vla_backend import ActionChunk, SIDES


def _blend_pose(start, finish, weight):
    retained = 1 - weight
    pose = np.empty(7)
    pose[0] = retained * start[0] + weight * finish[0]
    pose[1] = retained * start[1] + weight * finish[1]
    pose[2] = retained * start[2] + weight * finish[2]
    pose[3:] = quat_slerp(start[3:], finish[3:], weight)
    return pose


class TimedActions:

    def __init__(self, rate: float, origin: float,
                 minimum_overlap_actions: int = 0, first_offset_steps: int = 0,
                 execution_rate: float | None = None) -> None:
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('action_rate_hz must be positive and finite')
        if minimum_overlap_actions < 0:
            raise ValueError('async_min_overlap_actions must be non-negative')
        self.rate = rate
        self.execution_rate = rate if execution_rate is None else execution_rate
        if not math.isfinite(self.execution_rate) or self.execution_rate <= 0:
            raise ValueError('execution_rate_hz must be positive and finite')
        self.minimum_overlap_actions = minimum_overlap_actions
        self.origin = origin
        if first_offset_steps not in (0, 1):
            raise ValueError('first_offset_steps must be 0 or 1')
        self.first_offset_steps = first_offset_steps
        self.end = origin
        self._execution = ExecutionInterpolator(self.execution_rate, origin)
        self.samples: dict[int, tuple[dict, dict]] = {}
        self.last_merge: dict = {}
        self.merges = 0
        self.overlap_total = 0
        self.new_total = 0

    def merge(self, chunk: ActionChunk, requested: float, now: float,
              fallback_poses: dict[str, np.ndarray] | None = None) -> int:
        end = requested + chunk.horizon / self.rate
        consumed = -1
        if self._execution.last_time is not None:
            consumed = math.floor((self._execution.last_time - self.origin) * self.rate + 1e-8)
        first = max(consumed + 1,
                    math.ceil((max(now, requested + self.first_offset_steps / self.rate)
                               - self.origin) * self.rate - 1e-8))
        stop = math.ceil((end - self.origin) * self.rate - 1e-8)
        if self.first_offset_steps:
            stop = math.floor((end - self.origin) * self.rate + 1e-8) + 1
        anchor = math.floor((now - self.origin) * self.rate + 1e-8)
        self.samples = {tick: value for tick, value in self.samples.items()
                if tick >= first or tick == anchor}
        accepted = max(0, stop - first)
        old_poses = [self.samples[tick][0] for tick in range(first, stop)
                 if tick in self.samples]
        overlap_count = len(old_poses)
        blend_count = min(accepted, max(overlap_count, self.minimum_overlap_actions))
        fallback = old_poses[-1] if old_poses else fallback_poses
        if blend_count and fallback is None:
            raise ValueError('async merge needs fallback poses when overlap is empty')
        overlap = 0
        for tick in range(first, stop):
            offset = max(0.0, (self.origin + tick / self.rate - requested) * self.rate
                         - self.first_offset_steps)
            lower = min(int(math.floor(offset + 1e-8)), chunk.horizon - 1)
            upper = min(lower + 1, chunk.horizon - 1)
            fraction = min(1.0, max(0.0, offset - lower))
            poses, grippers = {}, {}
            previous = self.samples.get(tick)
            overlap += int(previous is not None)
            weight = (tick - first + 1) / (blend_count + 1)
            for side in SIDES:
                pose = _blend_pose(chunk.poses[side][lower], chunk.poses[side][upper], fraction)
                if tick - first < blend_count:
                    old = (previous[0] if previous is not None else fallback)[side]
                    pose = _blend_pose(old, pose, weight)
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
            'first_action_offset_s': (self.origin + first / self.rate - requested)
            if accepted else None,
            'responses': self.merges, 'overlap_total': self.overlap_total,
            'new_total': self.new_total,
        }
        return accepted

    def take(self, now: float):
        return self._execution.take(self, now)


class ExecutionInterpolator:
    def __init__(self, rate: float, origin: float):
        self.rate = rate
        self.origin = origin
        self.consumed = -1
        self.last_time = None

    def take(self, actions: TimedActions, now: float):
        if actions.first_offset_steps and now > actions.end + 1e-8:
            actions.samples.clear()
            return None
        tick = math.floor((now - self.origin) * self.rate + 1e-8)
        if tick <= self.consumed:
            return None
        self.consumed = tick
        self.last_time = self.origin + tick / self.rate
        offset = tick * actions.rate / self.rate
        lower = math.floor(offset + 1e-8)
        fraction = max(0., offset - lower)
        start = actions.samples.get(lower)
        finish = actions.samples.get(lower + 1)
        actions.samples = {key: sample for key, sample in actions.samples.items()
                           if key >= lower and (actions.origin + key / actions.rate >= now
                                                or now < actions.end)}
        if start is None or now > actions.end + 1e-8:
            return None
        if fraction < 1e-8:
            return start
        if finish is None:
            return start if not actions.first_offset_steps and now < actions.end else None
        poses, grippers = {}, {}
        for side in SIDES:
            poses[side] = _blend_pose(start[0][side], finish[0][side], fraction)
            grippers[side] = (1 - fraction) * start[1][side] + fraction * finish[1][side]
        return poses, grippers

"""World-frame footstep geometry and training-compatible queue execution."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def relative(pose, anchor):
    delta = np.asarray(pose)[:2] - np.asarray(anchor)[:2]
    cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])
    return np.array((cosine * delta[0] + sine * delta[1],
                     -sine * delta[0] + cosine * delta[1],
                     math.remainder(pose[2] - anchor[2], math.tau)))


def world(pose, anchor):
    cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])
    return np.array((anchor[0] + cosine * pose[0] - sine * pose[1],
                     anchor[1] + sine * pose[0] + cosine * pose[1],
                     math.remainder(anchor[2] + pose[2], math.tau)))


def mean_yaw(feet):
    cosine = sum(math.cos(pose[2]) for pose in feet)
    sine = sum(math.sin(pose[2]) for pose in feet)
    return float(feet[0][2]) if math.hypot(cosine, sine) <= 1e-6 else math.atan2(sine, cosine)


@dataclass
class Footstep:
    side: int
    pose: np.ndarray
    closing: bool = False

    def json(self):
        return dict(side=self.side, pose=self.pose.tolist(), closing=self.closing)


def validate_steps(steps, feet, *, single=False, initial_yaw=None, reference_yaw=None):
    previous = None
    current = np.array(feet, dtype=float, copy=True)
    if reference_yaw is not None and not math.isfinite(reference_yaw):
        raise ValueError('Reference heading must be finite')
    placed = [False, False]
    yaw = mean_yaw(current) if initial_yaw is None else initial_yaw
    for index, step in enumerate(steps):
        if step.side not in (0, 1) or step.pose.shape != (3,) or not np.all(np.isfinite(step.pose)):
            raise ValueError('Each footstep needs left=0/right=1 and finite x, y, yaw')
        if previous == step.side:
            raise ValueError('Footsteps must alternate left and right')
        offset = relative(step.pose, current[1 - step.side])
        if reference_yaw is not None and not placed[1 - step.side]:
            spacing = relative(step.pose, (*current[1 - step.side, :2], reference_yaw))
        else:
            spacing = offset
        width = spacing[1] * (1 if step.side == 0 else -1)
        maximum_width, maximum_distance = (.30, .35) if single else (.36, .48)
        if width > maximum_width + 1e-9:
            side_name = 'left' if step.side == 0 else 'right'
            raise ValueError(f'Step {index + 1} ({side_name}): Foot spacing {width:.3f} m exceeds '
                             f'{maximum_width:.2f} m in the planning support frame')
        if np.linalg.norm(offset[:2]) > maximum_distance + 1e-9:
            raise ValueError(f'Footstep planar displacement exceeds {maximum_distance:.2f} m')
        maximum_yaw = math.pi / 6 if single else math.pi / 2
        if abs(offset[2]) > maximum_yaw + 1e-9:
            raise ValueError(f'Adjacent foot yaw exceeds {math.degrees(maximum_yaw):.0f} degrees')
        if single and abs(math.remainder(step.pose[2] - yaw, math.tau)) > math.pi / 6 + 1e-9:
            raise ValueError('Foot yaw exceeds training coverage of +/-30 degrees from initial heading')
        current[step.side] = step.pose
        placed[step.side] = True
        previous = step.side
    return current


def simple_steps(feet, side, direction, distance, yaw=0., align=True, *, single=True, heading=None,
                 preserve_yaw=False):
    if direction not in ('forward', 'backward', 'left', 'right', 'rotate'):
        raise ValueError('Unknown step direction')
    if not math.isfinite(distance) or distance < 0. or not math.isfinite(yaw):
        raise ValueError('Distance must be finite and nonnegative; yaw must be finite')
    feet = np.asarray(feet, dtype=float)
    if heading is None:
        heading = mean_yaw(feet)
    if not math.isfinite(heading):
        raise ValueError('Reference heading must be finite')
    if direction == 'rotate':
        center = feet[:, :2].mean(axis=0)
        target = heading + yaw
        steps = [Footstep(current, world((0., .12 if current == 0 else -.12, 0.),
                                         (*center, target))) for current in (side, 1 - side)]
    else:
        delta = {'forward': (distance, 0.), 'backward': (-distance, 0.),
                 'left': (0., distance), 'right': (0., -distance)}[direction]
        shift = world((*delta, 0.), (0., 0., heading))[:2]
        goal = np.array((* (feet[side, :2] + shift), feet[side, 2] if preserve_yaw else heading))
        steps = [Footstep(side, goal)]
        if align:
            forward = np.array((math.cos(heading), math.sin(heading)))
            projection = np.dot(goal[:2] - feet[1 - side, :2], forward)
            closing = feet[1 - side, :2] + projection * forward
            closing_yaw = feet[1 - side, 2] if preserve_yaw else heading
            steps.append(Footstep(1 - side, np.array((*closing, closing_yaw)), True))
    validate_steps(steps, feet, single=single)
    return steps


def direction_steps(feet, side, direction, distance, yaw=0., *, heading=None, preserve_yaw=False,
                    width=.24):
    if direction == 'rotate':
        return simple_steps(feet, side, direction, distance, yaw, single=False, heading=heading)
    if direction not in ('forward', 'backward', 'left', 'right'):
        raise ValueError('Unknown step direction')
    if not math.isfinite(distance) or distance < 0. or not math.isfinite(yaw):
        raise ValueError('Distance must be finite and nonnegative; yaw must be finite')
    if not math.isfinite(width) or not 0. <= width <= .36:
        raise ValueError('Foot spacing must be 0.00..0.36 m')
    feet = np.asarray(feet, dtype=float)
    if heading is None:
        heading = mean_yaw(feet)
    if not math.isfinite(heading):
        raise ValueError('Reference heading must be finite')
    delta = {'forward': (distance, 0.), 'backward': (-distance, 0.),
             'left': (0., distance), 'right': (0., -distance)}[direction]
    shift = world((*delta, 0.), (0., 0., heading))[:2]
    count = 1
    while True:
        steps = []
        current = feet.copy()
        try:
            for _ in range(count):
                pair = []
                support = current[1 - side]
                for swing in (side, 1 - side):
                    pose = world((0., width if swing == 0 else -width, 0.), support)
                    if swing == side:
                        pose[:2] += shift / count
                    pose[2] = current[swing, 2] if preserve_yaw else heading
                    pair.append(Footstep(swing, pose))
                    support = pose
                current = validate_steps(pair, current)
                steps.extend(pair)
        except ValueError:
            if count == 500:
                raise
            count = min(2 * count, 500)
        else:
            return steps


def standing_step(feet, side, width=.24):
    if not math.isfinite(width) or not 0. <= width <= .36:
        raise ValueError('Standing foot spacing must be 0.00..0.36 m')
    pose = world((0., width if side == 0 else -width, 0.), feet[1 - side])
    step = Footstep(side, pose, True)
    validate_steps([step], feet)
    return step


def route_steps(points, feet, side, spacing=.18, width=.24, follow_heading=True):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.all(np.isfinite(points)):
        raise ValueError('Route needs at least two finite XY points in drawing order')
    if not .02 <= spacing <= .35 or not 0. <= width <= .36:
        raise ValueError('Invalid route spacing or stance width')
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.r_[True, lengths > 1e-6]
    points = points[keep]
    if len(points) < 2:
        raise ValueError('Route has zero length')
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]
    if cumulative[-1] > 50.:
        raise ValueError('Route exceeds 50 m')
    distances = np.arange(spacing, cumulative[-1], spacing).tolist() + [float(cumulative[-1])]
    initial_heading = mean_yaw(feet)
    steps = []
    current = np.array(feet, dtype=float, copy=True)
    for distance in distances:
        index = min(int(np.searchsorted(cumulative, distance, side='right') - 1), len(lengths) - 1)
        tangent = (points[index + 1] - points[index]) / lengths[index]
        center = points[index] + tangent * (distance - cumulative[index])
        support_heading = current[1 - side, 2]
        desired_heading = math.atan2(tangent[1], tangent[0]) if follow_heading else initial_heading
        turn = math.remainder(desired_heading - support_heading, math.tau)
        changes = np.linspace(np.clip(turn, -math.pi / 2, math.pi / 2), 0., 91) if follow_heading else [turn]
        for change in changes:
            heading = math.remainder(support_heading + change, math.tau)
            normal = np.array((-math.sin(heading), math.cos(heading)))
            pose = np.array((*(center + normal * width * (.5 if side == 0 else -.5)), heading))
            step = Footstep(side, pose)
            try:
                updated = validate_steps([step], current)
            except ValueError:
                if change == changes[-1]:
                    raise
            else:
                break
        steps.append(step)
        current = updated
        side = 1 - side
    validate_steps(steps, feet)
    return steps


class WorldFootstepQueue:
    def __init__(self, scheduler, *, assumed_landing=False, reference_yaw=None):
        self.scheduler = scheduler
        self.assumed_landing = assumed_landing
        if reference_yaw is not None and not math.isfinite(reference_yaw):
            raise ValueError('Reference heading must be finite')
        self.reference_yaw = reference_yaw
        self.pending = deque()
        self.committed = deque()
        self.state = 'standing'
        self.kind = 'continuous'
        self.target_frequency = 1.2
        self.phase = scheduler.phase
        self.elapsed = 0.
        self.hold_elapsed = 0.
        self.stop_phase = None
        self.stop_requested = False
        self.initial_yaw = mean_yaw(scheduler.slots[:2])
        self._profile_duration = None
        self._profile_frequency = 0.
        self._profile_ramp = .25
        self.has_walked = False

    def _planning_start_feet(self, start_feet=None, *, planning=True):
        feet = np.array(self.scheduler.slots[:2] if start_feet is None else start_feet,
                        dtype=float, copy=True)
        if feet.shape != (2, 3) or not np.all(np.isfinite(feet)):
            raise ValueError('Starting feet must contain two finite x, y, yaw poses')
        if planning and start_feet is None and not self.has_walked and self.reference_yaw is not None:
            feet[:, 2] = self.reference_yaw
        return feet

    def tail_feet(self, *, planning=True, start_feet=None):
        feet = self.tail_committed_feet(planning=planning, start_feet=start_feet)
        for step in self.pending:
            feet[step.side] = step.pose
        return feet

    def _validate(self, steps, *, include_pending=False, single=False, start_feet=None):
        feet = self.tail_committed_feet(planning=False, start_feet=start_feet)
        if include_pending:
            for step in self.pending:
                feet[step.side] = step.pose
        initial_support = not self.has_walked and not self.committed and not (include_pending and self.pending)
        return validate_steps(steps, feet, single=single, initial_yaw=self.initial_yaw,
                              reference_yaw=self.reference_yaw if initial_support else None)

    def next_side(self):
        if self.pending:
            return 1 - self.pending[-1].side
        if self.committed:
            return 1 - self.committed[-1][1].side
        return self._next_landing()[1]

    def append(self, steps, *, kind='continuous', start_feet=None):
        if not steps:
            raise ValueError('Footstep list is empty')
        if len(self.pending) + len(steps) > 1000:
            raise ValueError('Queue is limited to 1000 pending footsteps')
        if kind == 'single' and (self.state != 'standing' or self.pending):
            raise ValueError('Single-step action requires standing with an empty queue')
        if self.kind == 'single' and self.state != 'standing':
            raise ValueError('Wait for single-step completion before appending')
        if self.pending and kind != self.kind:
            raise ValueError('Do not mix single-step and continuous plans')
        can_choose_first = self.state == 'standing' and not self.pending and not self.committed
        if steps[0].side != self.next_side() and kind != 'single' and not can_choose_first:
            raise ValueError('First new footstep does not match the next alternating side')
        self._validate(steps, include_pending=True, single=kind == 'single', start_feet=start_feet)
        self.pending.extend(Footstep(step.side, step.pose.copy(), step.closing) for step in steps)
        if self.state == 'standing':
            self.kind = kind

    def replace_pending(self, steps, *, start_feet=None):
        if self.committed:
            expected = 1 - self.committed[-1][1].side
        elif self.pending:
            expected = self.pending[0].side
        else:
            expected = self._next_landing()[1]
        if steps and steps[0].side != expected:
            raise ValueError('Edited queue must retain alternating foot order')
        if self.kind == 'single' and len(steps) not in (0, 1, 2):
            raise ValueError('Single-step action contains one or two landings')
        self._validate(steps, single=self.kind == 'single', start_feet=start_feet)
        self.pending = deque(Footstep(step.side, step.pose.copy(), step.closing) for step in steps)

    def _next_landing(self):
        choices = []
        for side, center in enumerate(self.scheduler.touchdown):
            distance = (center - self.phase) % math.tau
            if distance < 1e-9:
                distance = math.tau
            choices.append((self.phase + distance, side))
        return min(choices)

    def _commit(self):
        while self.pending and len(self.committed) < 4:
            if self.committed:
                landing = self.committed[-1][0] + math.pi
                side = 1 - self.committed[-1][1].side
            else:
                landing, side = self._next_landing()
            if self.pending[0].side != side:
                raise ValueError('Queued side does not match phase schedule')
            self.committed.append((landing, self.pending.popleft()))

    def _preview(self):
        self.scheduler.slots[2:] = self.scheduler.slots[:2]
        for side in (0, 1):
            for _, step in self.committed:
                if step.side == side:
                    self.scheduler.slots[side + 2] = step.pose
                    break

    def start(self, frequency=1.2, *, start_feet=None):
        if self.state != 'standing' or not self.pending:
            raise ValueError('Start requires standing and a nonempty queue')
        if not math.isfinite(frequency) or not .8 <= frequency <= 1.8:
            raise ValueError('Cruise frequency must be 0.8..1.8 Hz')
        if self.has_walked:
            landing, side = self._next_landing()
            liftoff = self.phase + (self.scheduler.liftoff[side] - self.phase) % math.tau
            if self.pending[0].side != side or liftoff > landing:
                raise ValueError('First footstep must use the next swing side at the frozen phase')
        self._validate(list(self.pending), single=self.kind == 'single', start_feet=start_feet)
        if start_feet is not None:
            self.scheduler.slots[:2] = self._planning_start_feet(start_feet, planning=False)
            if self.has_walked:
                self.scheduler.anchor[:] = self.scheduler.slots[self.scheduler.anchor_side]
                self.scheduler.anchor_id += 1
        self.target_frequency = frequency
        self.stop_requested = False
        self.stop_phase = None
        self.elapsed = 0.
        self._profile_duration = None
        self.state = 'starting'
        if not self.has_walked:
            first_side = self.pending[0].side
            self.phase = self.scheduler.touchdown[1 - first_side]
            self.scheduler.phase = self.phase
            self.scheduler.anchor_side = 1 - first_side
            self.scheduler.anchor[:] = self.scheduler.slots[1 - first_side]
            self.scheduler.anchor_id += 1
        self._commit()
        if self.kind == 'single' or not self.pending:
            self.stop_phase = self.committed[-1][0]
            cycles = (self.stop_phase - self.phase) / math.tau
            self._profile_ramp = .25 if self.kind == 'single' else .5
            # Fit both ramps inside the planned phase span instead of adding a landing.
            self._profile_frequency = min(frequency, cycles / self._profile_ramp)
            self._profile_duration = cycles / self._profile_frequency + self._profile_ramp
        self._preview()
        self.has_walked = True

    def request_pause(self):
        if self.state in ('starting', 'walking'):
            self.stop_requested = True

    def _begin_stop(self):
        if not self.committed:
            raise ValueError('No committed landing available for stopping')
        landing = self.committed[-1][0]
        frequency = self.scheduler.frequency
        ramp = .5
        equivalent = (landing - self.phase) / (math.tau * frequency)
        duration = equivalent + 1.5 * ramp
        if duration < 2. * ramp:
            raise ValueError('Insufficient phase span for continuous stop ramp')
        self.stop_phase = landing
        self._profile_frequency = frequency
        self._profile_ramp = ramp
        self._profile_duration = duration
        # Enter the shared profile after its acceleration ramp, already at cruise speed.
        self.elapsed = ramp
        self.state = 'stopping'

    def tail_committed_feet(self, *, planning=True, start_feet=None):
        feet = self._planning_start_feet(start_feet, planning=planning)
        for _, step in self.committed:
            feet[step.side] = step.pose
        return feet

    @staticmethod
    def _profile_area(time, frequency, ramp, end):
        time = min(max(time, 0.), end)
        if time <= ramp:
            return frequency * time * time / (2. * ramp)
        if time <= end - ramp:
            return frequency * (time - ramp / 2.)
        tail = time - (end - ramp)
        return frequency * (end - 1.5 * ramp + tail - tail * tail / (2. * ramp))

    def tick(self, dt, feet):
        scheduler = self.scheduler
        if self.state == 'standing':
            self.hold_elapsed += dt
            scheduler.frequency = 0.
            return
        if (self.kind == 'continuous' and self._profile_duration is not None
                and self.state == 'starting' and self.pending and not self.stop_requested):
            self._profile_duration = None
            self.stop_phase = None
            self.state = 'walking'
        if self._profile_duration is not None:
            before = self._profile_area(
                self.elapsed, self._profile_frequency, self._profile_ramp, self._profile_duration)
            after = self._profile_area(
                self.elapsed + dt, self._profile_frequency, self._profile_ramp, self._profile_duration)
            frequency = (after - before) / dt
            self.elapsed += dt
            if self.kind == 'continuous' and self.elapsed >= self._profile_ramp - 1e-9:
                self.state = 'stopping'
        elif self.state == 'starting':
            self.elapsed += dt
            frequency = self.target_frequency * min(self.elapsed / .5, 1.)
            if self.elapsed >= .5 - 1e-9:
                self.state = 'walking'
        else:
            change = np.clip(self.target_frequency - scheduler.frequency, -.2 * dt, .2 * dt)
            frequency = scheduler.frequency + change
        advance = math.tau * frequency * dt
        if self.stop_phase is not None:
            advance = min(advance, max(0., self.stop_phase - self.phase))
            frequency = advance / (math.tau * dt)
        self.phase += advance
        scheduler.phase = self.phase % math.tau
        scheduler.frequency = frequency
        while self.committed and self.committed[0][0] <= self.phase + 1e-9:
            _, step = self.committed.popleft()
            scheduler.slots[step.side] = step.pose
            scheduler.anchor[:] = step.pose if self.assumed_landing else feet[step.side]
            scheduler.anchor_side = step.side
            scheduler.anchor_id += 1
            if self.kind == 'continuous' and self.state != 'stopping' and not self.stop_requested:
                self._commit()
        self._preview()
        if self.stop_phase is not None and self.phase >= self.stop_phase - 1e-9:
            self.phase = self.stop_phase
            scheduler.phase = self.phase % math.tau
            self.state = 'standing'
            self.hold_elapsed = 0.
            scheduler.frequency = 0.
        elif self.kind == 'continuous' and self.state == 'walking' and (self.stop_requested or not self.pending):
            self._begin_stop()

    def status(self):
        return dict(state=self.state, kind=self.kind, phase_unwrapped=self.phase,
                    target_frequency=self.target_frequency, hold_elapsed=self.hold_elapsed,
                    assumed_landing=self.assumed_landing,
                    committed=[dict(step.json(), landing=phase) for phase, step in self.committed],
                    pending=[step.json() for step in self.pending])

"""Manual RL bridge, reusing the ordinary observation and execution pipeline."""

import json
import math
import queue
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from g1_vla_bridge.online_rl.session import Journal
from g1_vla_bridge.online_rl.transport import ApiError, RetryRequest, Transport
from g1_vla_bridge.vla_backend import ActionChunk, SIDES
from g1_vla_bridge.vla_node import VlaBridgeNode


class OnlineRLBridge(VlaBridgeNode):
    def __init__(self):
        self._rl_ready = threading.Event()
        self._rl_guard = threading.RLock()
        self._rl_commands = queue.Queue(maxsize=16)
        self._rl_endings = queue.SimpleQueue()
        self._rl_active = None
        self._rl_steps = 0
        self._rl_tick = 0
        self._rl_owner = None
        self._rl_phase = 'starting'
        self._rl_pending = True
        self._rl_fault = ''
        self._rl_message = ''
        self._rl_last = None
        self._rl_status = {}
        self._rl_session = None
        self._rl_transport = None
        super().__init__()
        try:
            if self._spec.name != 'cogact_unitree':
                raise ValueError('Online RL requires the CogACT Unitree backend')
            directory = self.declare_parameter('rl_directory', '').value
            if not directory:
                raise ValueError('Set rl_directory to a persistent experiment directory')
            self._rl_max_age = float(self.declare_parameter('rl_max_observation_age_s', 3.0).value)
            if not math.isfinite(self._rl_max_age) or self._rl_max_age <= 0:
                raise ValueError('rl_max_observation_age_s must be positive')
            self._rl_session = Journal(directory)
            self._rl_transport = Transport(self._backend, self._rl_session)
            self._rl_publisher = self.create_publisher(String, '~/rl_status', 10)
            self.create_subscription(String, '~/feedback', self._on_feedback, 10)
            self.create_timer(.2, self._publish_rl_status)
            self._rl_ready.set()
        except Exception:
            self.shutdown()
            self.destroy_node()
            raise

    def _mode_error(self, mode):
        if mode != 'manual' or self._skip_intermediate:
            return 'Online RL requires manual, sequential chunk execution'
        return super()._mode_error(mode)

    def _on_set_skip_intermediate(self, request, response):
        if request.data:
            response.success, response.message = False, 'Online RL requires sequential execution'
            return response
        return super()._on_set_skip_intermediate(request, response)

    def _request_inference(self, generation=None):
        with self._rl_guard:
            if self._rl_fault:
                return self._rl_fault
            if self._rl_pending or self._rl_active:
                return 'Resolve the current request, execution or feedback first'
            if self._rl_phase != 'collecting':
                return f'RL server phase: {self._rl_phase}'
            return super()._request_inference(generation)

    def _on_feedback(self, message):
        try:
            command = json.loads(message.data)
            if not isinstance(command, dict):
                raise ValueError('Feedback command must be an object')
            self._rl_commands.put_nowait(command)
        except (ValueError, queue.Full):
            self._rl_message = 'Invalid feedback command or command queue full'

    def _publish_rl_status(self):
        self._rl_publisher.publish(String(data=json.dumps(self._rl_status)))

    def _snapshot(self):
        pending = self._rl_session.pending()
        record = pending[0] if pending else (
            self._rl_session.get(self._rl_last) if self._rl_last else None)
        summary = None
        if record:
            summary = {key: record.get(key) for key in (
                'request_id', 'id', 'state', 'executed_steps', 'planned_steps', 'reason',
                'feedback', 'feedback_confirmed')}
            summary.update({key: record.get('response', {}).get(key)
                            for key in ('policy_version', 'batch_id', 'execution_chunk_size')})
        with self._rl_guard:
            self._rl_pending = bool(pending)
            self._rl_status = dict(phase=self._rl_phase, fault=self._rl_fault,
                                   message=self._rl_message, record=summary,
                                   pending=bool(pending), directory=str(self._rl_session.directory))

    def _commands(self):
        while not self._rl_commands.empty():
            command = self._rl_commands.get_nowait()
            try:
                request_id = command['request_id']
                record = self._rl_session.get(request_id)
                if command['id'] != record['id']:
                    raise ValueError('Chunk ID mismatch')
                if command.get('command') == 'score':
                    self._rl_session.queue_feedback(request_id, command['reward'])
                elif command.get('command') == 'resolve':
                    self._rl_session.resolve(request_id, command['executed_steps'])
                else:
                    raise ValueError('Unknown feedback command')
                self._rl_last = request_id
                self._rl_message = ''
            except (KeyError, ValueError, TypeError) as error:
                self._rl_message = str(error)

    def _drain_endings(self):
        while not self._rl_endings.empty():
            request_id, steps, completed, reason, uncertain = self._rl_endings.get_nowait()
            if uncertain:
                self._rl_session.update(request_id, state='uncertain', executed_steps=steps,
                                        reason=reason)
            else:
                self._rl_session.finish(request_id, steps, completed, reason)
            self._rl_last = request_id

    def _infer_loop(self):
        while self._alive and not self._rl_ready.wait(.1):
            pass
        next_status = 0.
        next_attempt = 0.
        while self._alive:
            try:
                self._drain_endings()
                self._commands()
                now = time.monotonic()
                if now >= next_status and not self._rl_fault:
                    next_status = now + 1.
                    try:
                        self._rl_phase = self._rl_transport.status()['phase']
                    except RetryRequest as error:
                        self._rl_phase = 'unreachable'
                        self._rl_message = str(error)
                    if self._rl_phase == 'error':
                        self._halt('Server adaptation phase=error; maintainer intervention required')
                records = self._rl_session.pending()
                record = records[0] if records else None
                if record and record['state'] == 'skipped':
                    self._rl_session.queue_feedback(record['request_id'], None)
                    record = self._rl_session.get(record['request_id'])
                if record and now >= next_attempt and not self._rl_fault:
                    next_attempt = now + max(.2, self._retry_delay)
                    if record['state'] == 'feedback_pending':
                        self._rl_transport.feedback(record['request_id'])
                        self._rl_last = record['request_id']
                        self._rl_message = ''
                    elif record['state'] == 'prepared':
                        self._fetch(record['request_id'])
                if not record and self._infer_requested.is_set() and not self._rl_fault:
                    self._begin_request()
                    next_attempt = 0.
                self._snapshot()
            except RetryRequest as error:
                self._rl_message = str(error)
                self._snapshot()
            except ApiError as error:
                self._handle_api_error(error)
                self._snapshot()
            except Exception as error:
                self._halt(f'RL {type(error).__name__}; check protocol or local journal')
                try:
                    self._snapshot()
                except Exception:
                    self._rl_status = dict(phase=self._rl_phase, fault=self._rl_fault,
                                           message='', pending=True, record=None)
            time.sleep(.1)
        if self._rl_session:
            self._drain_endings()

    def _begin_request(self):
        with self._lock:
            self._infer_requested.clear()
            generation = self._generation
            if not self._running.is_set():
                self._inference_active = False
                return
        if self._rl_phase != 'collecting':
            with self._lock:
                self._inference_active = False
            self._rl_message = f'Waiting for collecting; current phase={self._rl_phase}'
            return
        try:
            observation = self._observe()
        except Exception:
            with self._lock:
                self._inference_active = False
            self._rl_message = 'Observation unavailable; acquire a new observation with Enter'
            return
        with self._lock:
            if generation != self._generation or not self._running.is_set():
                self._inference_active = False
                return
        self._rl_transport.prepare(observation)
        self._rl_owner = generation
        self._rl_pending = True

    def _fetch(self, request_id):
        chunk, response = self._rl_transport.infer(request_id)
        record = self._rl_session.get(request_id)
        age = time.time() - record['observation_time']
        with self._rl_guard:
            with self._lock:
                allowed = (self._rl_owner == self._generation and self._running.is_set()
                           and 0 <= age <= self._rl_max_age and self._rl_phase == 'collecting')
            if not allowed:
                self._rl_session.update(request_id, state='skipped',
                                        reason='Cancelled, recovered or stale response; not executed')
                with self._lock:
                    self._inference_active = False
                return
            limit = min(chunk.horizon, response['execution_chunk_size'])
            if self._horizon > 0:
                limit = min(limit, self._horizon)
            prediction = ActionChunk(
                poses={side: chunk.poses[side][:limit] for side in SIDES},
                grippers={side: chunk.grippers[side][:limit] for side in SIDES})
            self._rl_session.claim(request_id, limit)
            self._rl_active = request_id
            self._rl_steps = 0
            try:
                super()._accept(prediction, age * 1000., self._rl_owner)
            except Exception:
                self._stop('Failed to accept the chunk')
                raise
            if self._chunk is None:
                self._end_execution(False, 'Cancelled before first publication')

    def _handle_api_error(self, error):
        records = self._rl_session.pending()
        record = records[0] if records else None
        if (record and record['state'] == 'feedback_pending'
                and error.status == 503 and error.code == 'updating'):
            self._rl_phase = 'updating'
            self._rl_message = 'Feedback retained; retrying after update'
            return
        if record and record['state'] == 'prepared' and (
                (error.status == 503 and error.code in ('updating', 'stale_observation'))
                or (error.status == 410 and error.code == 'expired')):
            self._rl_session.update(record['request_id'], state='discarded', reason=str(error))
            self._rl_owner = None
            self._rl_phase = 'updating' if error.code == 'updating' else 'refresh_required'
            self._rl_message = 'Old observation discarded; use Enter after collecting resumes'
            with self._lock:
                self._inference_active = False
            return
        if record and record['state'] == 'feedback_pending' and error.status == 410:
            self._rl_session.update(record['request_id'], state='expired', reason=str(error))
            self._rl_message = 'Feedback expired without confirmation; record retained'
            return
        self._halt(str(error))

    def _halt(self, reason):
        self._rl_fault = reason
        self._stop(reason)

    def _end_execution(self, completed, reason='', uncertain=False):
        if self._rl_active:
            self._rl_endings.put((self._rl_active, self._rl_steps, completed, reason, uncertain))
            self._rl_active = None
            self._rl_pending = True

    def _on_tick(self):
        with self._rl_guard:
            if not self._rl_active:
                return
            self._rl_tick = self._cursor
            try:
                super()._on_tick()
            except Exception:
                self._end_execution(False, 'Publication failed; operator verification required', True)
                self._stop('RL publication failed')
                return
            if self._rl_active and self._chunk is None:
                self._end_execution(True)

    def _publish_control(self, command, grip, generation):
        published = super()._publish_control(command, grip, generation)
        if published and self._rl_active and self._chunk is not None:
            limit = self._chunk.horizon
            offset = min(limit - 1., self._rl_tick * self._action_rate / self._execution_rate)
            if self._rl_tick >= math.ceil(
                    limit * self._execution_rate / self._action_rate - 1e-8) - 1:
                offset = limit - 1.
            index = int(math.floor(offset + 1e-8))
            self._rl_steps = max(self._rl_steps, index + 1)
        return published

    def _stop(self, reason, generation=None):
        with self._rl_guard:
            with self._lock:
                current = generation is None or generation == self._generation
            if not current:
                return
            super()._stop(reason, generation)
            self._end_execution(False, reason)

    def _on_task(self, message):
        with self._rl_guard:
            if message.data != self._task and (self._rl_active or self._inference_active):
                self._stop('Task changed')
            return super()._on_task(message)

    def _on_home(self, request, response):
        with self._rl_guard:
            return super()._on_home(request, response)

    def _on_start(self, request, response):
        with self._rl_guard:
            if self._rl_pending or self._rl_fault:
                response.success, response.message = False, 'Resolve pending RL records first'
                return response
            return super()._on_start(request, response)

    def shutdown(self):
        super().shutdown()
        if self._rl_transport:
            self._rl_transport.close()
        if self._rl_session:
            self._rl_session.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = OnlineRLBridge()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

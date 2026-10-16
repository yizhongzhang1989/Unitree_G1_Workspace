"""HTTP footstep editor on the existing motion-session task."""

import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from g1_motion_session.task_base import State
from g1_rgmt_tracking_global.odometry import OdometryFuser
from g1_rgmt_tracking_global.rotations import quat_from_xyzw, quat_to_mat, torso_quat_from_pelvis
from .footstep_task import FootstepTask
from .footstep_planner import (
    Footstep, WorldFootstepQueue, direction_steps, mean_yaw, route_steps, standing_step)


STATIC = Path(__file__).with_name('static')


class FootstepWebAPI:
    def __init__(self, task):
        self.task = task
        self.last_poll = time.monotonic()
        self.revision = 0
        self._signature = self._queue_signature()

    def _queue_signature(self):
        queue = self.task.runtime.controller
        return json.dumps(dict(pending=[step.json() for step in queue.pending],
                               feet=queue.tail_feet().tolist(), next_side=queue.next_side())) if queue else 'null'

    def _sync_revision(self):
        signature = self._queue_signature()
        if signature != self._signature:
            self.revision += 1
            self._signature = signature

    def status(self):
        task = self.task
        with task.execution_lock:
            self.last_poll = time.monotonic()
            self._sync_revision()
            queue = task.runtime.controller
            return dict(task_state=task.state.value, preparing=task._preparing,
                        ready=task.ready, registered=task._registered, error=task._error,
                        localization=task.localization_mode, localization_error=task.localization_error(),
                        revision=self.revision,
                        footstep=task.runtime.status()['footstep'],
                        queue=queue.status() if queue else None)

    def command(self, payload):
        task = self.task
        action = payload.get('action')
        if action in ('engage', 'track', 'release', 'estop'):
            return task.session_command({'engage': 'engage', 'track': 'start',
                                         'release': 'stop', 'estop': 'estop'}[action])
        with task.execution_lock:
            if action == 'localization':
                mode = payload['mode']
                if mode not in ('planned', 'fused'):
                    raise ValueError('Unknown localization mode')
                if task.state not in (State.IDLE, State.STOPPED, State.ERROR):
                    raise ValueError('Stop the task before changing the world frame')
                task.localization_mode = mode
                task.runtime.world_transform = task.world_transform if mode == 'fused' else None
                task.runtime.controller = None
                self.revision += 1
                return dict(ok=True)
            if action == 'theta':
                task.runtime.set_theta(math.radians(float(payload['theta_deg'])))
                return dict(ok=True, revision=self.revision)
            queue = task.runtime.controller
            if queue is None:
                raise ValueError('Complete engage preparation before planning footsteps')
            self._sync_revision()
            if action in ('route', 'route_preview', 'simple', 'custom', 'replace', 'clear'):
                if int(payload.get('revision', -1)) != self.revision:
                    raise ValueError('Queue changed; refresh before editing')
            if action in ('route', 'route_preview'):
                steps = route_steps(payload['points'], queue.tail_feet(), queue.next_side(),
                                    float(payload.get('spacing', .18)), float(payload.get('width', .24)),
                                    bool(payload.get('follow_heading', True)))
                if action == 'route_preview':
                    return dict(ok=True, revision=self.revision, steps=[step.json() for step in steps])
                queue.append(steps)
                self._sync_revision()
                return dict(ok=True, revision=self.revision)
            if action == 'walk':
                if task.state is not State.RUNNING:
                    raise ValueError('Start session tracking before executing the queue')
                reason = task.stale_reason() or task.localization_error()
                if reason:
                    raise ValueError(reason)
                if 'theta_deg' in payload:
                    task.runtime.set_theta(math.radians(float(payload['theta_deg'])))
                queue.start(float(payload.get('frequency', 1.2)))
            elif action == 'pause':
                queue.request_pause()
            elif action == 'frequency':
                frequency = float(payload['frequency'])
                if not math.isfinite(frequency) or not .8 <= frequency <= 1.8:
                    raise ValueError('Frequency must be 0.8..1.8 Hz')
                if queue.kind == 'single' or queue.stop_requested or queue.state == 'closing':
                    raise ValueError('Frequency is frozen during single-step and stopping')
                queue.target_frequency = frequency
            elif action in ('simple', 'custom', 'replace', 'clear'):
                if action == 'simple':
                    direction = payload['direction']
                    feet = queue.tail_feet(planning=direction != 'stand')
                    if direction == 'stand':
                        steps = [standing_step(feet, queue.next_side(), float(payload.get('width', .24)))]
                    else:
                        heading = mean_yaw(task.runtime.feet)
                        side = queue.next_side()
                        if direction == 'rotate':
                            if queue.pending:
                                heading = queue.pending[-1].pose[2]
                            elif queue.committed:
                                heading = queue.committed[-1][1].pose[2]
                        elif not queue.pending and not queue.committed:
                            side = 1 - queue.scheduler.anchor_side
                        steps = direction_steps(
                            feet, side, direction, float(payload.get('distance', .1)),
                            math.radians(float(payload.get('yaw_deg', 15.))),
                            heading=heading, preserve_yaw=queue.has_walked,
                            width=float(payload.get('width', .24)))
                    queue.append(steps)
                elif action in ('custom', 'replace'):
                    steps = [Footstep(int(step['side']), np.asarray(step['pose'], dtype=float),
                                      bool(step.get('closing', False)))
                             for step in payload['steps']]
                    if action == 'custom':
                        queue.append(steps)
                    else:
                        queue.replace_pending(steps)
                else:
                    queue.replace_pending([])
            else:
                raise ValueError('Unknown command')
            self._sync_revision()
            return dict(ok=True, revision=self.revision)


def make_server(api, host, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def send(self, status, content, kind='application/json; charset=utf-8'):
            if not isinstance(content, bytes):
                content = json.dumps(content, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            try:
                if self.path == '/api/state':
                    self.send(200, api.status())
                elif self.path in ('/', '/index.html'):
                    self.send(200, (STATIC / 'index.html').read_bytes(), 'text/html; charset=utf-8')
                elif self.path in ('/app.js', '/style.css', '/lucide.min.js'):
                    kind = 'text/javascript' if self.path.endswith('.js') else 'text/css'
                    self.send(200, (STATIC / self.path[1:]).read_bytes(), kind + '; charset=utf-8')
                else:
                    self.send(404, dict(error='Not found'))
            except Exception as error:
                self.send(500, dict(error=str(error)))

        def do_POST(self):
            try:
                if self.path != '/api/command':
                    self.send(404, dict(error='Not found'))
                    return
                origin = self.headers.get('Origin')
                if origin and origin not in ('http://' + self.headers.get('Host', ''),
                                             'https://' + self.headers.get('Host', '')):
                    self.send(403, dict(error='Cross-origin robot commands are forbidden'))
                    return
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('Commands require application/json')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 262144:
                    raise ValueError('Invalid command size')
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError('Command must be an object')
                self.send(200, api.command(payload))
            except (ValueError, KeyError, TypeError) as error:
                self.send(400, dict(error=str(error)))
            except Exception as error:
                self.send(500, dict(error=str(error)))

    class Server(ThreadingHTTPServer):
        request_queue_size = 128
        daemon_threads = True

    return Server((host, port), Handler)


class FootstepWebTask(FootstepTask):
    def __init__(self):
        super().__init__(ros_input=False)
        self.localization_mode = self.declare('footstep_localization_mode', 'fused')
        if self.localization_mode not in ('fused', 'planned'):
            raise ValueError('footstep_localization_mode must be fused or planned')
        self.fuser = OdometryFuser(
            odom_timeout_s=self.declare('odom_timeout_s', .2),
            lidar_timeout_s=self.declare('lidar_timeout_s', 1.),
            correction_tau_s=self.declare('lidar_correction_tau_s', 2.))
        self.runtime.world_transform = self.world_transform if self.localization_mode == 'fused' else None
        group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Odometry, self.declare('odom_topic', '/dog_odom'),
                                 self.on_odom, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(Odometry, self.declare('lidar_pose_topic', '/g1_localization/torso_pose'),
                                 self.on_lidar, qos_profile_sensor_data, callback_group=group)
        self.engage_client = self.create_client(Trigger, '/motion_control/engage')
        self.start_client = self.create_client(Trigger, '/motion_control/start')
        self.stop_client = self.create_client(Trigger, '/motion_control/stop')
        self.estop_client = self.create_client(Trigger, '/motion_control/estop')
        self.api = FootstepWebAPI(self)
        self.browser_timeout = self.declare('footstep_browser_timeout_s', 2.)
        host = self.declare('footstep_web_host', '0.0.0.0')
        port = self.declare('footstep_web_port', 8210)
        self.server = make_server(self.api, host, port)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f'Footstep editor: http://{host}:{self.server.server_port}')

    @property
    def ready(self):
        return super().ready and not self.localization_error()

    def localization_error(self):
        if getattr(self, 'localization_mode', 'planned') == 'planned':
            return ''
        return self.fuser.stale(self.now()) or ''

    def on_odom(self, message):
        with self.execution_lock:
            if not self.runtime.ready or self.stale_reason():
                return
            try:
                snapshot = self.snapshot()
                torso = self.runtime.torso_pose(snapshot)
                position = message.pose.pose.position
                orientation = message.pose.pose.orientation
                quat = quat_from_xyzw((orientation.x, orientation.y, orientation.z, orientation.w))
                rotation = quat_to_mat(quat)
                torso_position = np.array((position.x, position.y, position.z)) + rotation @ torso.translation
                waist = snapshot.position[self.runtime._joint_slots][12:15] + self.runtime._offsets[12:15]
                torso_quat = torso_quat_from_pelvis(quat, *waist)
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                self.fuser.push_odom(stamp, torso_position, torso_quat)
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=1.)

    def on_lidar(self, message):
        if message.pose.covariance[0] < 0.:
            return
        with self.execution_lock:
            try:
                position, orientation = message.pose.pose.position, message.pose.pose.orientation
                quat = quat_from_xyzw((orientation.x, orientation.y, orientation.z, orientation.w))
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                self.fuser.push_lidar(stamp, (position.x, position.y, position.z), quat)
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=1.)

    def world_transform(self, snapshot, torso):
        reason = self.localization_error()
        if reason:
            raise ValueError(reason)
        quaternion = self.fuser.orientation_in_world(quat_from_xyzw(snapshot.orientation))
        rotation = quat_to_mat(quaternion)
        return rotation, self.fuser.torso_position() - rotation @ torso.translation

    def engage(self, positions31):
        self.runtime.controller = None
        super().engage(positions31)

    def prepared(self, snapshot):
        self.runtime.reset(self.snapshot(), self.now())
        self.runtime.controller = WorldFootstepQueue(
            self.runtime.scheduler, assumed_landing=self.localization_mode == 'planned',
            reference_yaw=mean_yaw(self.runtime.feet))
        self.api.revision += 1

    def start(self, snapshot):
        if self.runtime.controller is None:
            self.prepared(snapshot)
        self._reset_waist_filter(snapshot)
        self._stopped = False

    def control(self):
        with self.execution_lock:
            queue = self.runtime.controller
            if queue is not None and time.monotonic() - self.api.last_poll > self.browser_timeout:
                queue.request_pause()
            if self.state is State.STAND and not self._preparing and queue is not None:
                try:
                    reason = self.stale_reason() or self.localization_error()
                    if reason:
                        raise ValueError(reason)
                    feet = self.runtime.observe(self.snapshot())
                    queue.tick(self.dt, feet)
                except Exception as error:
                    self.estop(str(error))
                return
            super().control()

    def status(self):
        detail = super().status()
        detail['footstep_queue'] = self.runtime.controller.status() if self.runtime.controller else None
        return detail

    def session_command(self, command):
        client = {'engage': self.engage_client, 'start': self.start_client,
                  'stop': self.stop_client, 'estop': self.estop_client}[command]
        if not client.service_is_ready():
            raise ValueError(f'/motion_control/{command} is unavailable')
        future = client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda completed: done.set())
        if not done.wait(3.):
            raise RuntimeError('Session response timed out; check robot state before retrying')
        response = future.result()
        if response is None:
            raise RuntimeError('Session returned an empty response')
        if not response.success:
            raise ValueError(response.message)
        return dict(ok=True, message=response.message)

    def destroy_node(self):
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server_thread.join()
            self.server.server_close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    executor = MultiThreadedExecutor()
    node = None
    try:
        node = FootstepWebTask()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

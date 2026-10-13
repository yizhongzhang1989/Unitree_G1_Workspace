import json
import math
from types import SimpleNamespace
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest

from g1_motion_session.task_base import State
from lower_footstep_tracking.footstep_web import FootstepWebAPI, FootstepWebTask, make_server
from test_footstep_planner import finish, queue
from lower_footstep_tracking.footstep_planner import Footstep, relative, validate_steps, world


def make_api():
    controller = queue()
    controller.reference_yaw = 0.
    runtime = SimpleNamespace(controller=controller, robot_pose=np.zeros(3))
    runtime.status = lambda: {'footstep': {
        'feet': runtime.controller.scheduler.slots[:2].tolist(),
        'robot_pose': runtime.robot_pose.tolist(), 'phase': runtime.controller.scheduler.phase,
        'frequency': runtime.controller.scheduler.frequency,
        'anchor': runtime.controller.scheduler.anchor.tolist()}}

    def session_command(command):
        with task.execution_lock:
            if command == 'engage':
                if task.state is State.RUNNING:
                    raise ValueError('Stop preview tracking before engaging')
                runtime.controller = queue()
                task.state = State.STAND
            elif command == 'start':
                if task.state is not State.STAND:
                    raise ValueError('Engage preview before starting tracking')
                task.state = State.RUNNING
            elif command in ('stop', 'estop'):
                runtime.controller.committed.clear()
                runtime.controller.state = 'standing'
                runtime.controller.scheduler.frequency = 0.
                task.state = State.STOPPED
            else:
                raise ValueError('Unknown preview session command')
        return dict(ok=True, message='Isolated preview session command completed')

    task = SimpleNamespace(execution_lock=threading.RLock(), runtime=runtime,
                           state=State.RUNNING, _preparing=False, _registered=True,
                           _error='', ready=True, localization_mode='planned',
                           localization_error=lambda: '', stale_reason=lambda: '',
                           session_command=session_command)
    return FootstepWebAPI(task)


def test_api_append_edit_pause_and_reject_stale_revision():
    api = make_api()
    api.command(dict(action='simple', direction='forward', distance=.1, revision=0))
    assert len(api.status()['queue']['pending']) == 2
    with pytest.raises(ValueError, match='Queue changed'):
        api.command(dict(action='clear', revision=0))
    api.command(dict(action='clear', revision=1))
    api.task.runtime.controller.kind = 'continuous'
    api.command(dict(action='route', points=[[0., 0.], [1., 0.]], revision=2))
    api.command(dict(action='walk'))
    api.command(dict(action='pause'))
    assert api.task.runtime.controller.stop_requested
    committed = api.status()['queue']['committed']
    api.command(dict(action='clear', revision=api.status()['revision']))
    assert api.status()['queue']['committed'] == committed


def test_stand_button_appends_next_foot_alignment_without_starting():
    api = make_api()
    api.task.runtime.controller.append([Footstep(0, np.array((.2, .12, 0.)))])
    before = api.status()
    api.command(dict(action='simple', direction='stand', width=.12, revision=before['revision']))
    planned = api.status()
    assert planned['queue']['state'] == 'standing'
    assert planned['queue']['pending'][0] == before['queue']['pending'][0]
    step = planned['queue']['pending'][-1]
    assert step['side'] == 1 and step['closing']
    np.testing.assert_allclose(relative(step['pose'], [.2, .12, 0.]), [0., -.12, 0.], atol=1e-12)
    api.command(dict(action='replace', steps=planned['queue']['pending'], revision=planned['revision']))
    assert api.status()['queue']['pending'][-1]['closing']
    with pytest.raises(ValueError, match='Queue changed'):
        api.command(dict(action='simple', direction='stand', revision=before['revision']))
    api.command(dict(action='walk'))
    finish(api.task.runtime.controller)
    feet = api.status()['footstep']['feet']
    np.testing.assert_allclose(feet[0], [.2, .12, 0.], atol=1e-12)
    np.testing.assert_allclose(relative(feet[1], feet[0]), [0., -.12, 0.], atol=1e-12)


@pytest.mark.parametrize('direction,support_side,has_walked,width', [
    ('forward', 1, False, .18), ('backward', 0, True, .30),
    ('left', 1, True, .24), ('right', 0, False, .24)])
def test_direction_buttons_step_from_current_support_with_staggered_feet(direction, support_side, has_walked, width):
    api = make_api()
    controller = api.task.runtime.controller
    controller.scheduler.slots[:2] = [[.24756208110323324, .3561729059209607, -.42392661990417885],
                                      [0., 0., 0.]]
    controller.scheduler.anchor_side = support_side
    controller.has_walked = has_walked
    controller.reference_yaw = -.1278201221774715
    api.task.runtime.robot_pose[2] = controller.reference_yaw
    feet = controller.tail_feet()
    measured = controller.scheduler.slots[:2].copy()
    api.command(dict(action='simple', direction=direction, distance=.1, width=width,
                     revision=api.status()['revision']))
    steps = list(controller.pending)
    assert [step.side for step in steps] == [1 - support_side, support_side]
    delta = {'forward': (.1, 0.), 'backward': (-.1, 0.),
             'left': (0., .1), 'right': (0., -.1)}[direction]
    shift = world((*delta, 0.), (0., 0., controller.reference_yaw))[:2]
    sign = 1 if steps[0].side == 0 else -1
    first = world((0., sign * width, 0.), feet[support_side])
    np.testing.assert_allclose(steps[0].pose[:2], first[:2] + shift, atol=1e-12)
    np.testing.assert_allclose(relative(steps[1].pose, steps[0].pose)[:2], [0., -sign * width], atol=1e-12)
    if has_walked:
        for step in steps:
            assert step.pose[2] == measured[step.side, 2]
    np.testing.assert_array_equal(controller.scheduler.slots[:2], measured)
    validate_steps(steps, feet)
    api.command(dict(action='walk'))
    finish(controller)
    np.testing.assert_allclose(relative(controller.scheduler.slots[0], controller.scheduler.slots[1]),
                               [0., .24, 0.], atol=1e-12)


@pytest.mark.parametrize('committed', [False, True])
def test_appended_direction_uses_torso_heading_and_tail_support(committed):
    api = make_api()
    controller = api.task.runtime.controller
    controller.append([Footstep(0, np.array((0., .12, .6)))])
    if committed:
        api.command(dict(action='walk'))
    api.task.runtime.robot_pose[2] = -.2
    support = controller.tail_feet()[0].copy()
    api.command(dict(action='simple', direction='forward', distance=.1, revision=api.status()['revision']))
    first, following = list(controller.pending)[-2:]
    assert [first.side, following.side] == [1, 0]
    shift = world((.1, 0., 0.), (0., 0., -.2))[:2]
    target = world((0., -.24, 0.), support)
    np.testing.assert_allclose(first.pose[:2], target[:2] + shift, atol=1e-12)
    np.testing.assert_allclose(relative(following.pose, first.pose)[:2], [0., .24], atol=1e-12)


@pytest.mark.parametrize('committed', [False, True])
def test_appended_turn_uses_last_planned_heading(committed):
    api = make_api()
    controller = api.task.runtime.controller
    controller.append([Footstep(0, np.array((0., .12, .6)))])
    if committed:
        api.command(dict(action='walk'))
        assert controller.committed and not controller.pending
    api.command(dict(action='simple', direction='rotate', yaw_deg=15., revision=api.status()['revision']))
    np.testing.assert_allclose([step.pose[2] for step in list(controller.pending)[-2:]],
                               .6 + math.radians(15.), atol=1e-12)


def test_initial_custom_step_checks_body_frame_without_rotating_measured_hold_targets():
    api = make_api()
    controller = api.task.runtime.controller
    body_yaw = .5
    body = np.array((1., -2., body_yaw))
    api.task.runtime.robot_pose[:] = body
    controller.reference_yaw = body_yaw
    controller.scheduler.slots[:2] = [world((0., .12, -.5), body), world((0., -.12, -.5), body)]
    measured = controller.scheduler.slots[:2].copy()
    steps = [dict(side=1, pose=world((.21, -.10, -.6), body).tolist()),
             dict(side=0, pose=world((.38, .16, -.6), body).tolist())]
    with pytest.raises(ValueError, match='spacing'):
        validate_steps([Footstep(1, np.array(steps[0]['pose']))], measured)
    api.command(dict(action='custom', steps=steps, revision=api.status()['revision']))
    assert len(controller.pending) == 2
    np.testing.assert_array_equal(controller.scheduler.slots[:2], measured)
    api.command(dict(action='replace', steps=steps, revision=api.status()['revision']))
    api.command(dict(action='walk'))
    np.testing.assert_array_equal(controller.scheduler.slots[:2], measured)
    assert controller.committed and controller.has_walked


def test_initial_body_spacing_does_not_bypass_measured_foot_yaw_limit():
    api = make_api()
    api.task.runtime.controller.scheduler.slots[:2, 2] = -1.2
    with pytest.raises(ValueError, match='90 degrees'):
        api.command(dict(action='custom', steps=[dict(side=0, pose=[.1, .12, .4])],
                         revision=api.status()['revision']))
    assert not api.task.runtime.controller.pending


def test_spacing_rejection_identifies_step_and_keeps_queue_unchanged():
    api = make_api()
    before = api.status()
    with pytest.raises(ValueError, match=r'Step 1 \(left\): Foot spacing 0\.040 m'):
        api.command(dict(action='custom', steps=[dict(side=0, pose=[.1, -.08, 0.])], revision=0))
    assert api.status()['queue'] == before['queue']
    assert api.status()['revision'] == before['revision']


def test_automatic_closing_invalidates_preview_when_pending_is_empty():
    api = make_api()
    api.command(dict(action='simple', direction='forward', distance=.1, revision=0))
    api.command(dict(action='walk'))
    controller = api.task.runtime.controller
    assert not controller.pending
    revision = api.status()['revision']
    payload = dict(points=[[.1, 0.], [.5, 0.]], revision=revision)
    preview = api.command(dict(action='route_preview', **payload))
    side = controller.next_side()
    for _ in range(100):
        controller.tick(.02, controller.scheduler.slots[:2].copy())
        if controller.state == 'closing':
            break
    assert controller.state == 'closing'
    assert not controller.pending
    assert controller.next_side() != side
    assert preview['steps'][0]['side'] == side
    with pytest.raises(ValueError, match='Queue changed'):
        api.command(dict(action='route', **payload))
    closing_revision = api.status()['revision']
    landing = controller.committed[0][0]
    for _ in range(1000):
        controller.tick(.02, controller.scheduler.slots[:2].copy())
        if controller.phase >= landing - 1e-9:
            break
    assert controller.phase >= landing - 1e-9
    assert api.status()['revision'] == closing_revision


@pytest.mark.parametrize('points,count', [
    ([[0., 0.], [.6, 0.]], 4),
    ([[0., 0.], [.6, 0.], [.6, .6]], 7)])
def test_route_preview_matches_commit_without_mutating_queue(points, count):
    api = make_api()
    payload = dict(points=points, revision=0, spacing=.18, width=.24)
    before = api.status()
    preview = api.command(dict(action='route_preview', **payload))
    assert len(preview['steps']) == count
    assert api.status()['queue'] == before['queue']
    assert api.status()['revision'] == before['revision']
    api.command(dict(action='route', **payload))
    assert api.status()['queue']['pending'] == preview['steps']
    with pytest.raises(ValueError, match='Queue changed'):
        api.command(dict(action='route_preview', **payload))
    api.command(dict(action='clear', revision=api.status()['revision']))
    with pytest.raises(ValueError, match='spacing'):
        api.command(dict(action='route_preview', points=[[0., 0.], [0., .6]],
                         spacing=.5, revision=api.status()['revision']))
    assert not api.task.runtime.controller.pending


def test_http_errors_and_cross_origin_command_rejection():
    api = make_api()
    server = make_server(api, '127.0.0.1', 0)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base + '/api/state') as response:
            assert json.load(response)['localization'] == 'planned'
        payload = json.dumps(dict(action='simple', direction='forward', distance=.1, revision=0)).encode()
        request = Request(base + '/api/command', data=payload,
                          headers={'Content-Type': 'application/json', 'Origin': 'http://other-host'})
        with pytest.raises(HTTPError) as error:
            urlopen(request)
        assert error.value.code == 403
        request = Request(base + '/api/command', data=b'{}', headers={'Content-Type': 'application/json'})
        with pytest.raises(HTTPError) as error:
            urlopen(request)
        assert error.value.code == 400
        assert not api.task.runtime.controller.pending
    finally:
        server.shutdown()
        thread.join()
        server.server_close()


def test_real_web_task_preparation_and_fused_fk_without_hardware():
    import time
    from pathlib import Path
    import rclpy
    from nav_msgs.msg import Odometry

    rclpy.init(args=['--ros-args', '-p', 'footstep_localization_mode:=planned',
                     '-p', 'footstep_web_host:=127.0.0.1', '-p', 'footstep_web_port:=0',
                     '-p', 'foot_sole_offsets:=[0.0, 0.0, -0.035, 0.0, 0.0, -0.035]'],
               domain_id=217)
    node = None
    try:
        node = FootstepWebTask()
        root = Path(__file__).resolve().parents[3]
        node.on_description(SimpleNamespace(data=(root / 'src/unitree_g1_description/model/final.urdf').read_text()))
        node._q[:29] = node.runtime.contract['default_joint_pos']
        node._joint_stamp = node._imu_stamp = time.monotonic()
        node.prepared(node.snapshot())
        node.state, node._preparing = State.STAND, False
        node.control()
        measured = node.runtime.feet.copy()
        odom = Odometry()
        now = node.get_clock().now().to_msg()
        odom.header.stamp = now
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.z = .7
        node.on_odom(odom)
        np.testing.assert_array_equal(node.runtime.feet, measured)
        torso = node.fuser._last_odom.pos.copy()
        odom.pose.pose.position.x = float(torso[0] + 2.)
        odom.pose.pose.position.y = float(torso[1] - 1.)
        node.on_lidar(odom)
        node.state = State.STOPPED
        node.api.command(dict(action='localization', mode='fused'))
        node.localization_mode = 'fused'
        node.runtime.world_transform = node.world_transform
        feet = node.runtime.observe(node.snapshot())
        np.testing.assert_allclose(node.runtime.robot_pose[:2], torso[:2] + [2., -1.], atol=1e-9)
        assert np.all(np.isfinite(feet))
        assert not node.localization_error()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

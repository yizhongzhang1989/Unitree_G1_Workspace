import threading
import importlib
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast
from unittest.mock import patch

import numpy as np
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped
from ikt_pose_commander.commander_node import PoseCommander
from ikt_pose_commander.dashboard_node import CommanderDashboard
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer

from robot_bringup.ik_model_view import ActiveJointModel, joints_between
from robot_bringup.ikt_dashboard_compat import (
    G1CommanderDashboard,
    patch_viewer_target_sender,
)
from robot_bringup.ikt_pose_commander_compat import EXECUTOR_THREADS, FPC_NAME, JTC_NAME, G1PoseCommander, _filter_switch, _joint_map_error, _parameter_service
from robot_bringup.dashboard_compat_node import _correct_mimic_transforms, _filter_mimic_snapshot, _hidden_mimic_names, _parse_mimic_joints


FINAL_URDF = Path(__file__).parents[2] / "unitree_g1_description" / "model" / "final.urdf"
ikt_core = importlib.import_module("ikt_core")
RobotModel = ikt_core.RobotModel
SolveParams = ikt_core.SolveParams
Task = ikt_core.Task
solve = ikt_core.solve


class _Future:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class _Client:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _Future(self.responses.pop(0))


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(list(message.data))


class _ActionClient:
    def __init__(self):
        self.goals = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _Future(SimpleNamespace(accepted=True))


def _state(name, state):
    controller = ControllerState()
    controller.name = name
    controller.state = state
    return controller


def _list_response(*controllers):
    response = ListControllers.Response()
    response.controller = list(controllers)
    return response


def _fake_commander(before, after=None, switch_ok=True):
    switch_response = SwitchController.Response()
    switch_response.ok = switch_ok
    commander = SimpleNamespace(_cli_list=_Client([before] + ([after] if after else [])), _cli_switch=_Client([switch_response]), _set_msg=lambda message: setattr(commander, "message", message), message="")
    pose_commander = cast(G1PoseCommander, commander)
    commander._controller_states = lambda: G1PoseCommander._controller_states(pose_commander)
    commander._switch_failed = lambda message: G1PoseCommander._switch_failed(pose_commander, message)
    return commander


def test_parameter_service_follows_manager_namespace():
    assert _parameter_service("/robot/controller_manager", FPC_NAME) == "/robot/forward_position_controller/get_parameters"


def test_executor_has_workers_for_services_and_control():
    assert EXECUTOR_THREADS >= 3


def test_g1_commander_uses_latest_only_target_and_fpc_qos():
    commander = object.__new__(G1PoseCommander)
    commander._target_topic = "~/target_pose"
    subscriptions = []
    publishers = []

    def create_subscription(
            _self, _msg_type, _topic, _callback, qos_profile,
            *args, **kwargs):
        del args, kwargs
        subscriptions.append(qos_profile)
        return object()

    def create_publisher(
            _self, _msg_type, _topic, qos_profile, *args, **kwargs):
        del args, kwargs
        publishers.append(qos_profile)
        return object()

    with patch.object(
            PoseCommander, "create_subscription",
            new=create_subscription), patch.object(
                PoseCommander, "create_publisher", new=create_publisher):
        commander.create_subscription(
            PoseStamped, "~/target_pose", lambda message: message, 10)
        commander.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)

    assert subscriptions[0].depth == 1
    assert subscriptions[0].reliability == ReliabilityPolicy.RELIABLE
    assert publishers[0].depth == 1
    assert publishers[0].reliability == ReliabilityPolicy.BEST_EFFORT


def test_fpc_updates_only_active_ik_targets_and_publishes_full_cache():
    commander = object.__new__(G1PoseCommander)
    dynamic = cast(Any, commander)
    commander._lock = threading.Lock()
    commander._fpc_joints = ["arm", "base", "passive"]
    commander._joints = ["arm", "base"]
    commander._joint_pos = {"arm": 0.0, "base": 0.9, "passive": -0.2}
    commander._fixed_hold = {"base": 0.4}
    commander._last_solution = SimpleNamespace(active_joints=["arm"])
    commander._joint_targets = {}
    commander._control_rate = 200.0
    commander._max_speed = 16.0
    commander._joint_speed_limits = {}
    commander._last_fpc_cmd = None
    dynamic._traj = object()
    commander._decoupled_active = True
    dynamic._fpc_pub = _Publisher()
    dynamic._set_msg = lambda msg: setattr(dynamic, "message", msg)

    commander._command_fpc({"arm": 1.0, "base": 2.0})
    commander._joint_pos.update({"arm": 0.0, "base": 0.7, "passive": -0.4})
    commander._command_fpc({"arm": 1.0})
    commander._command_fpc({"arm": -1.0})
    commander._joint_pos["arm"] = 0.04
    commander._command_fpc({"arm": 1.0}, best_effort=True)

    assert dynamic._fpc_pub.messages == [
        [0.08, 0.9, -0.2],
        [0.16, 0.9, -0.2],
        [0.08, 0.9, -0.2],
        [0.16, 0.9, -0.2],
    ]
    assert commander._traj is None
    assert not commander._decoupled_active
    assert dynamic.message.endswith("[best-effort]")


def test_jtc_updates_only_active_ik_targets_and_publishes_full_cache():
    commander = object.__new__(G1PoseCommander)
    dynamic = cast(Any, commander)
    commander._lock = threading.Lock()
    commander._jtc_joints = ["left_arm", "right_arm", "waist"]
    commander._joints = ["right_arm"]
    commander._joint_pos = {
        "left_arm": 0.2, "right_arm": -0.1, "waist": 0.0}
    commander._joint_targets = {
        "left_arm": 1.1, "right_arm": -0.1, "waist": 0.3}
    commander._fixed_hold = {}
    commander._last_solution = SimpleNamespace(active_joints=["right_arm"])
    commander._control_rate = 200.0
    commander._last_jtc_cmd = None
    commander._min_time = 0.5
    commander._max_speed = 16.0
    commander._jtc = "joint_trajectory_controller"
    commander._jtc_client = _ActionClient()
    commander._goal_handle = None
    dynamic._set_msg = lambda msg: setattr(dynamic, "message", msg)

    commander._command_jtc({"right_arm": 0.8}, 0.9)

    goal = commander._jtc_client.goals[0]
    assert goal.trajectory.joint_names == [
        "left_arm", "right_arm", "waist"]
    assert list(goal.trajectory.points[0].positions) == [1.1, 0.8, 0.3]
    assert commander._joint_targets == {
        "left_arm": 1.1, "right_arm": 0.8, "waist": 0.3}


def test_enable_republishes_cached_full_target_after_controller_switch():
    commander = object.__new__(G1PoseCommander)
    dynamic = cast(Any, commander)
    commander._lock = threading.Lock()
    commander._mode = "fpc"
    commander._fpc_joints = ["left_arm", "right_arm"]
    commander._joints = ["right_arm"]
    commander._joint_pos = {"left_arm": 0.2, "right_arm": -0.1}
    commander._joint_targets = {"left_arm": 1.1, "right_arm": 0.6}
    commander._fixed_hold = {}
    commander._last_solution = SimpleNamespace(active_joints=["right_arm"])
    commander._control_rate = 200.0
    commander._max_speed = 16.0
    commander._joint_speed_limits = {}
    commander._last_fpc_cmd = None
    commander._traj = None
    commander._decoupled_active = False
    dynamic._fpc_pub = _Publisher()
    dynamic._set_msg = lambda msg: setattr(dynamic, "message", msg)

    with patch.object(
            PoseCommander, "_try_enable", autospec=True,
            return_value=(True, "enabled")):
        result = commander._try_enable()

    assert result == (True, "enabled")
    assert dynamic._fpc_pub.messages == [[1.1, 0.6]]


def test_return_to_start_keeps_other_joint_targets():
    commander = object.__new__(G1PoseCommander)
    dynamic = cast(Any, commander)
    commander._lock = threading.Lock()
    dynamic._goal_handle = object()
    commander._joints = ["right_arm"]
    commander._start_q = {"right_arm": 0.8}
    commander._jtc_joints = ["left_arm", "right_arm"]
    commander._joint_pos = {"left_arm": 0.2, "right_arm": -0.1}
    commander._joint_targets = {"left_arm": 1.1, "right_arm": 0.6}
    dynamic._pause_targets = lambda: None
    dynamic._confirm_return_result = lambda result: result
    dynamic._set_msg = lambda msg: setattr(dynamic, "message", msg)
    seen = {}

    def return_to_start(_self, _timeout):
        seen["joints"] = list(_self._joints)
        seen["targets"] = dict(_self._start_q)
        return True, "returned to start"

    with patch.object(
            PoseCommander, "_return_to_start", autospec=True,
            side_effect=return_to_start):
        result = commander._return_to_start()

    assert result == (
        True, "returned to start; disabled in JTC hold")
    assert seen == {
        "joints": ["left_arm", "right_arm"],
        "targets": {"left_arm": 1.1, "right_arm": 0.8},
    }
    assert commander._joints == ["right_arm"]
    assert commander._start_q == {"right_arm": 0.8}
    assert commander._joint_targets == {
        "left_arm": 1.1, "right_arm": 0.8}


def test_g1_viewer_keeps_only_one_target_post_in_flight():
    source = '''before
function sendProxyTarget(stream) { api(
    stream ? "/api/target" : "/api/send", targetPoseBody()
); }

// "Snap target -> link": next section
after'''

    patched = patch_viewer_target_sender(source)

    assert "let _targetPostInFlight = false" in patched
    assert "_pendingTargetBody = body" in patched
    assert "while (_pendingTargetBody !== null)" in patched
    assert 'api(stream ? "/api/target"' not in patched


def test_g1_viewer_overlay_disables_browser_cache():
    source = Path(
        G1CommanderDashboard._install_viewer_overlay.__code__.co_filename
    ).read_text(encoding="utf-8")

    assert 'handler.send_header("Cache-Control", "no-store")' in source


def test_g1_dashboard_targets_use_latest_transform():
    dashboard = object.__new__(G1CommanderDashboard)
    dashboard._base_frame = "torso_link"
    cast(Any, dashboard).get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=123, nanosec=456)))

    message = dashboard._build_target_msg(
        [0.1, -0.2, 0.3], [1.0, 0.0, 0.0, 0.0], "torso_link")

    assert message.header.frame_id == "torso_link"
    assert message.header.stamp.sec == 0
    assert message.header.stamp.nanosec == 0


def test_filter_switch_drops_states_that_already_hold():
    assert _filter_switch({FPC_NAME: "inactive", JTC_NAME: "inactive"}, [FPC_NAME], [JTC_NAME]) == ([FPC_NAME], [])
    assert _filter_switch({FPC_NAME: "active", JTC_NAME: "inactive"}, [FPC_NAME], [JTC_NAME]) == ([], [])


def test_joint_map_validation_requires_full_equal_claim_sets():
    assert _joint_map_error(["arm"], ["arm", "hold"], ["hold", "arm"]) == ""
    assert "different joint sets" in _joint_map_error(["arm"], ["arm", "left"], ["arm", "right"])
    assert "metadata unavailable" in _joint_map_error(["arm"], [], ["arm"])


def test_solver_dimension_follows_dynamic_base_target_interval():
    model = RobotModel(FINAL_URDF.read_text(encoding="utf-8"))
    all_joints = model.supporting_joints("right_gripper_base")
    expected_dimensions = {
        "pelvis": 10,
        "torso_link": 7,
        "right_shoulder_yaw_link": 4,
        "right_wrist_roll_link": 2,
    }
    for base, expected in expected_dimensions.items():
        assert len(joints_between(
            model, base, "right_gripper_base", all_joints)) == expected

    arm_joints = joints_between(
        model, "torso_link", "right_gripper_base", all_joints)
    seed = model.neutral()
    reduced = ActiveJointModel(model, seed, arm_joints)
    xyz, quat = model.fk(seed, "right_gripper_base")
    target = np.asarray(xyz) + np.asarray([0.002, 0.0, 0.0])
    dimensions = []
    numpy_solve = np.linalg.solve

    def record_dimension(matrix, right_hand_side):
        dimensions.append(matrix.shape)
        return numpy_solve(matrix, right_hand_side)

    with patch("numpy.linalg.solve", side_effect=record_dimension):
        solution = solve(
            reduced,
            reduced.reduce(seed),
            [Task.pose(
                "right_gripper_base", tuple(target), tuple(quat))],
            params=SolveParams(max_iters=5),
            active_joints=arm_joints,
        )

    assert dimensions
    assert all(shape == (len(arm_joints), len(arm_joints))
               for shape in dimensions)
    assert len(arm_joints) < len(all_joints) < model.nq
    assert reduced.expand(solution.q).shape == seed.shape


def test_dynamic_interval_uses_chain_through_common_ancestor():
    model = RobotModel(FINAL_URDF.read_text(encoding="utf-8"))
    path = joints_between(
        model,
        "torso_link",
        "left_ankle_roll_link",
        model.joint_names,
    )

    assert len(path) == 9
    assert set(path) == {
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    }


def test_cross_branch_model_uses_relative_chain_jacobian():
    model = RobotModel(FINAL_URDF.read_text(encoding="utf-8"))
    seed = model.neutral()
    base = "torso_link"
    target = "left_ankle_roll_link"
    path = joints_between(model, base, target, model.joint_names)
    reduced = ActiveJointModel(model, seed, path, base)
    q = reduced.reduce(seed)
    xyz, quat = model.fk(seed, target)
    jacobian = reduced.frame_jacobian(q, target)

    for joint in ("waist_yaw_joint", "waist_roll_joint",
                  "waist_pitch_joint"):
        assert np.linalg.norm(jacobian[:, reduced.q_index(joint)]) > 1e-6

    epsilon = 1e-7
    numeric = np.empty_like(jacobian)
    for column in range(reduced.nq):
        perturbed = q.copy()
        perturbed[column] += epsilon
        numeric[:, column] = -reduced.pose_error(
            perturbed, target, xyz, quat) / epsilon

    np.testing.assert_allclose(jacobian, numeric, atol=2e-6, rtol=2e-5)


def test_cross_branch_model_solves_relative_ankle_target():
    model = RobotModel(FINAL_URDF.read_text(encoding="utf-8"))
    seed = model.neutral()
    base = "torso_link"
    target = "left_ankle_roll_link"
    path = joints_between(model, base, target, model.joint_names)
    reduced = ActiveJointModel(model, seed, path, base)
    q_seed = reduced.reduce(seed)
    xyz, quat = model.fk(seed, target)
    desired_xyz = np.asarray(xyz) + np.asarray([0.002, 0.0, 0.0])

    solution = solve(
        reduced,
        q_seed,
        [Task.pose(target, tuple(desired_xyz), tuple(quat))],
        params=SolveParams(max_iters=30, tol_pos=1e-5),
        active_joints=path,
    )

    initial_error = np.linalg.norm(reduced.pose_error(
        q_seed, target, desired_xyz, quat)[:3])
    final_error = np.linalg.norm(reduced.pose_error(
        solution.q, target, desired_xyz, quat)[:3])
    waist_delta = solution.q[:3] - q_seed[:3]
    leg_delta = solution.q[3:] - q_seed[3:]
    assert final_error < initial_error * 0.1
    assert np.linalg.norm(waist_delta) > 1e-6
    assert np.linalg.norm(leg_delta) > 1e-6


def test_control_tick_drops_overlapping_callback():
    commander = cast(
        G1PoseCommander,
        SimpleNamespace(
            _lock=threading.Lock(),
            _mode="jtc",
            _control_tick_lock=threading.Lock(),
        ),
    )
    commander._control_tick_lock.acquire()

    with patch.object(PoseCommander, "_control_tick", autospec=True) as base:
        G1PoseCommander._control_tick(commander)

    base.assert_not_called()
    commander._control_tick_lock.release()


def test_model_root_uses_urdf_root_link_without_tf_round_trip():
    model = SimpleNamespace(link_frame_names=lambda: ["pelvis", "torso_link"])
    commander = object.__new__(G1PoseCommander)
    commander._lock = threading.Lock()
    commander._model = model
    commander._base_frame = ""
    def fail_transform(*arguments, **keyword_arguments):
        del arguments, keyword_arguments
        raise AssertionError("root-link target must not use TF")

    commander._tf_buffer = cast(
        Buffer, SimpleNamespace(transform=fail_transform))
    target = PoseStamped()
    target.header.frame_id = "pelvis"
    target.pose.orientation.w = 1.0

    xyz, quat = commander._resolve_pose(target)

    assert commander._model_root() == "pelvis"
    assert xyz is not None and quat is not None
    assert xyz.tolist() == [0.0, 0.0, 0.0]
    assert quat.tolist() == [1.0, 0.0, 0.0, 0.0]


def test_switch_omits_inactive_jtc_from_stop_request():
    before = _list_response(_state(FPC_NAME, "inactive"), _state(JTC_NAME, "inactive"))
    after = _list_response(_state(FPC_NAME, "active"), _state(JTC_NAME, "inactive"))
    commander = _fake_commander(before, after)

    assert G1PoseCommander._switch(cast(G1PoseCommander, commander), [FPC_NAME], [JTC_NAME]) is True
    request = commander._cli_switch.requests[0]
    assert request.start_controllers == [FPC_NAME]
    assert request.stop_controllers == []


def test_switch_skips_service_if_target_states_already_hold():
    states = _list_response(_state(FPC_NAME, "active"), _state(JTC_NAME, "inactive"))
    commander = _fake_commander(states)

    assert G1PoseCommander._switch(cast(G1PoseCommander, commander), [FPC_NAME], [JTC_NAME]) is True
    assert commander._cli_switch.requests == []


def test_switch_rejects_best_effort_partial_success():
    before = _list_response(_state(FPC_NAME, "active"), _state(JTC_NAME, "inactive"))
    after = _list_response(_state(FPC_NAME, "inactive"), _state(JTC_NAME, "inactive"))
    commander = _fake_commander(before, after)

    assert G1PoseCommander._switch(cast(G1PoseCommander, commander), [JTC_NAME], [FPC_NAME]) is False
    assert "state mismatch" in commander.message


def test_disable_is_idempotent_and_releases_active_controller():
    switches = []
    commander = SimpleNamespace(
        _lock=threading.Lock(), _fpc=FPC_NAME, _jtc=JTC_NAME,
        _goal_handle=None, _pause_targets=lambda: None,
        _controller_states=lambda: {
            FPC_NAME: "inactive", JTC_NAME: "inactive"},
        _switch=lambda activate, deactivate: switches.append(
            (activate, deactivate)) or True,
        _set_msg=lambda message: setattr(commander, "message", message),
    )
    response = SimpleNamespace(success=False, message="")

    result = G1PoseCommander._srv_disable(cast(G1PoseCommander, commander), None, response)

    assert result.success is True
    assert result.message == "disabled; motion controllers inactive"
    assert switches == []

    commander._controller_states = lambda: {
        FPC_NAME: "active", JTC_NAME: "inactive"}
    result = G1PoseCommander._srv_disable(
        cast(G1PoseCommander, commander), None, response)

    assert result.success is True
    assert switches == [([], [FPC_NAME])]


class _GoalHandle:
    def __init__(self, response):
        self.response = response

    def get_result_async(self):
        return _Future(self.response)


def test_return_result_checks_action_error_code():
    response = FollowJointTrajectory.Impl.GetResultService.Response()
    response.status = GoalStatus.STATUS_SUCCEEDED
    response.result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
    response.result.error_string = "goal tolerance violated"
    commander = SimpleNamespace(_lock=threading.Lock(), _goal_handle=_GoalHandle(response))

    result = G1PoseCommander._confirm_return_result(cast(G1PoseCommander, commander), (True, "returned to start"))

    assert result[0] is False
    assert "code=-5" in result[1]


def test_dashboard_timeouts_cover_hardware_takeover():
    assert G1CommanderDashboard.call_trigger.__defaults__ == (35.0,)
    assert G1CommanderDashboard.call_return_to_start.__defaults__ == (55.0,)


def test_dashboard_uses_urdf_root_link_for_targets():
    dashboard = object.__new__(G1CommanderDashboard)
    dashboard._lock = threading.Lock()
    dashboard._model = SimpleNamespace(
        link_frame_names=lambda: ["pelvis"])
    with patch.object(
            CommanderDashboard, "snapshot", autospec=True,
            return_value={"has_model_viz": False,
                          "root_frame": "root_joint"}):
        state = dashboard.snapshot()

    assert state["root_frame"] == "pelvis"


def test_dashboard_stream_keeps_only_latest_target():
    dashboard = object.__new__(G1CommanderDashboard)
    dashboard._lock = threading.Lock()
    dashboard._sm_forwarding = False
    dashboard._stream_active = False
    dashboard._stream_pose = None
    dashboard._base_frame = "pelvis"
    published = []
    test_dashboard = cast(Any, dashboard)
    test_dashboard._target_pub = SimpleNamespace(
        publish=lambda message: published.append(message))
    test_dashboard._build_target_msg = lambda xyz, quat, frame_id: (
        list(xyz), list(quat), frame_id)

    for index in range(100):
        result = dashboard.stream_pose(
            [index / 1000.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], "pelvis")

    assert result["ok"] is True
    assert dashboard._stream_active is True
    assert dashboard._stream_pose == (
        [0.099, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], "pelvis")
    assert len(published) == 100
    assert published[-1] == (
        [0.099, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], "pelvis")


def test_ik_dashboard_hides_internal_model_data_after_correcting_fk():
    specs = _parse_mimic_joints(FINAL_URDF.read_text(encoding="utf-8"))
    hidden_links, hidden_joints = _hidden_mimic_names(specs)
    theta = 2.0 * 2.76377472169236 / 3.0
    identity = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    link_tf = _correct_mimic_transforms({"left_gripper_base": identity, "right_gripper_base": identity}, specs, {"left_eccentric_joint": theta, "right_eccentric_joint": theta})
    state = {
        "link_tf": link_tf,
        "skeleton": {},
        "links": list(link_tf),
        "joints": [spec.name for spec in specs],
        "joint_values": {spec.name: 0.0 for spec in specs},
        "joint_limits": {spec.name: [0.0, 1.0] for spec in specs},
        "visuals": [],
        "has_meshes": True,
        "joint_tree": [
            {"name": spec.name, "parent": spec.parent, "child": spec.child, "type": spec.joint_type}
            for spec in specs
        ],
        "status": {
            "available_links": list(link_tf),
            "available_joints": [spec.name for spec in specs],
            "fixed_joints": [spec.name for spec in specs],
            "joints": [spec.name for spec in specs],
            "group_joints": [spec.name for spec in specs],
            "command_joints": ["left_eccentric_joint", "right_eccentric_joint"],
            "joint_speed_limits": {spec.name: 1.0 for spec in specs},
            "joint_accel_limits": {spec.name: 1.0 for spec in specs},
        },
    }

    filtered = _filter_mimic_snapshot(state, hidden_links, hidden_joints)

    assert "left_left_connecting_rod" in filtered["link_tf"]
    assert "right_right_connecting_rod" in filtered["link_tf"]
    assert not hidden_links.intersection(filtered["links"])
    assert not hidden_joints.intersection(filtered["joints"])
    assert not hidden_links.intersection(filtered["status"]["available_links"])
    assert not hidden_joints.intersection(filtered["status"]["available_joints"])
    assert filtered["status"]["fixed_joints"] == []
    assert filtered["status"]["command_joints"] == ["left_eccentric_joint", "right_eccentric_joint"]

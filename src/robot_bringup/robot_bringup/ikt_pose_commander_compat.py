#!/usr/bin/env python3
"""Foxy adapter for the G1 pose commander and its dashboard."""

import os
import threading
from typing import Dict, List, Optional, Sequence, Tuple

# Small dynamic IK systems are much faster without a BLAS thread pool.
for _thread_env in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_thread_env] = "1"

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped
from ikt_pose_commander.commander_node import PoseCommander
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from robot_bringup.ik_model_view import ActiveJointModel, joints_between
from robot_bringup.ikt_dashboard_compat import G1CommanderDashboard


FPC_NAME = "forward_position_controller"
JTC_NAME = "joint_trajectory_controller"
QUERY_TIMEOUT_S = 3.0
SWITCH_TIMEOUT_S = 30.0
RETURN_TIMEOUT_S = 50.0
EXECUTOR_THREADS = 3

def _wait_result(future, timeout_s: float):
    done = threading.Event()
    future.add_done_callback(
        lambda completed_future: (completed_future, done.set())[1])
    if not done.wait(timeout=timeout_s):
        return None
    try:
        return future.result()
    except Exception:  # noqa: BLE001
        return None


def _parameter_service(controller_manager: str, controller: str) -> str:
    manager = "/" + controller_manager.strip("/")
    namespace = manager.rpartition("/")[0]
    path = "/" + controller.strip("/")
    if namespace and not path.startswith(namespace + "/"):
        path = namespace + path
    return path + "/get_parameters"


def _filter_switch(states: Dict[str, str], activate: List[str],
                   deactivate: List[str]) -> Tuple[List[str], List[str]]:
    return (
        [name for name in activate if states.get(name) != "active"],
        [name for name in deactivate if states.get(name) != "inactive"],
    )


def _joint_map_error(active: List[str], fpc: List[str], jtc: List[str]) -> str:
    if not fpc or not jtc:
        return "FPC/JTC joint metadata unavailable"
    if len(fpc) != len(set(fpc)) or len(jtc) != len(set(jtc)):
        return "FPC/JTC joint metadata contains duplicates"
    missing_fpc = sorted(set(active) - set(fpc))
    missing_jtc = sorted(set(active) - set(jtc))
    if missing_fpc or missing_jtc:
        return f"FPC missing {missing_fpc}; JTC missing {missing_jtc}"
    if set(fpc) != set(jtc):
        return "FPC and JTC command different joint sets"
    return ""


def _latest_qos(reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=reliability,
    )


class G1PoseCommander(PoseCommander):
    """Use fixed G1 controllers while adapting Foxy service fields."""

    def create_subscription(self, msg_type, topic, callback, qos_profile,
                            *args, **kwargs):
        if (msg_type is PoseStamped and
                topic == getattr(self, "_target_topic", None)):
            qos_profile = _latest_qos(ReliabilityPolicy.RELIABLE)
        return super().create_subscription(
            msg_type, topic, callback, qos_profile, *args, **kwargs)

    def create_publisher(self, msg_type, topic, qos_profile, *args, **kwargs):
        if (msg_type is Float64MultiArray and
                str(topic).rstrip("/").endswith("/commands")):
            qos_profile = _latest_qos(ReliabilityPolicy.BEST_EFFORT)
        return super().create_publisher(
            msg_type, topic, qos_profile, *args, **kwargs)

    def __init__(self) -> None:
        self._compat_lock = threading.Lock()
        self._control_tick_lock = threading.Lock()
        self._joint_clients = {}
        self._joint_cache: Dict[str, List[str]] = {}
        super().__init__()

    def _solve(self, model, seed, xyz, quat):
        with self._lock:
            base = self._base_frame
            target = self._frame
            allowed = list(self._joints)
        joints = joints_between(model, base, target, allowed)
        reduced = ActiveJointModel(model, seed, joints)
        solution = super()._solve(
            reduced, reduced.reduce(seed), xyz, quat)
        solution.q = reduced.expand(solution.q)
        if solution.q_seed is not None:
            solution.q_seed = reduced.expand(solution.q_seed)
        solution.joint_names = list(model.joint_names)
        solution.active_joints = joints
        return solution

    def _apply_live(self, request: dict) -> str:
        if "base_frame" in request and request["base_frame"] is not None:
            base = str(request["base_frame"] or "")
            with self._lock:
                model = self._model
                target = self._frame
                allowed = list(self._joints)
            if model is not None and target:
                try:
                    joints_between(model, base, target, allowed)
                except ValueError as error:
                    return f"base_frame rejected: {error}"
        return super()._apply_live(request)

    def _control_tick(self) -> None:
        if not self._control_tick_lock.acquire(blocking=False):
            return
        try:
            super()._control_tick()
        finally:
            self._control_tick_lock.release()

    def _model_root(self) -> str:
        with self._lock:
            model = self._model
        if model is not None:
            links = model.link_frame_names()
            if links:
                return links[0]
        return super()._model_root()

    def _controller_joints(self, name: str) -> List[str]:
        with self._compat_lock:
            if name in self._joint_cache:
                return list(self._joint_cache[name])
            client = self._joint_clients.get(name)
            if client is None:
                client = self.create_client(
                    GetParameters,
                    _parameter_service(self._cm, name),
                    callback_group=self._cbg,
                )
                self._joint_clients[name] = client
        if not client.wait_for_service(timeout_sec=QUERY_TIMEOUT_S):
            return []
        request = GetParameters.Request()
        request.names = ["joints"]
        response = _wait_result(
            client.call_async(request), QUERY_TIMEOUT_S + 1.0)
        values = list(getattr(response, "values", []) or []) if response else []
        joints = list(values[0].string_array_value) if values else []
        if joints:
            with self._compat_lock:
                self._joint_cache[name] = list(joints)
        return joints

    def _controller_joint_map(self) -> Dict[str, List[str]]:
        names = (self._fpc or FPC_NAME, self._jtc or JTC_NAME)
        output = {}
        for name in dict.fromkeys(names):
            joints = self._controller_joints(name)
            if joints:
                output[name] = joints
        return output

    def _discover_controllers(self, joints) -> Dict[str, str]:
        maps = self._controller_joint_map()
        wanted = set(joints)
        fpc = self._fpc or FPC_NAME
        jtc = self._jtc or JTC_NAME
        return {
            "fpc": fpc if wanted.issubset(maps.get(fpc, [])) else "",
            "jtc": jtc if wanted.issubset(maps.get(jtc, [])) else "",
        }

    def _apply_structural(self, _request: dict):
        result = super()._apply_structural(_request)
        if not result[0]:
            return result
        with self._lock:
            active = list(self._joints)
            fpc = list(self._fpc_joints)
            jtc = list(self._jtc_joints)
        error = _joint_map_error(active, fpc, jtc)
        if not error:
            return result
        with self._lock:
            self._configured = False
        return False, "controller metadata incomplete: " + error

    def _controller_states(self) -> Dict[str, str]:
        if self._cli_list is None or not self._cli_list.wait_for_service(
                timeout_sec=QUERY_TIMEOUT_S):
            return {}
        response = _wait_result(
            self._cli_list.call_async(ListControllers.Request()),
            QUERY_TIMEOUT_S + 1.0,
        )
        return {
            str(controller.name): str(controller.state)
            for controller in (getattr(response, "controller", []) or [])
        }

    def _switch(self, activate: List[str], deactivate: List[str]) -> bool:
        states = self._controller_states()
        if any(name not in states for name in set(activate + deactivate)):
            return self._switch_failed("controller state unavailable")
        required_activate, required_deactivate = _filter_switch(
            states, activate, deactivate)
        if not required_activate and not required_deactivate:
            return True
        if self._cli_switch is None or not self._cli_switch.wait_for_service(
                timeout_sec=QUERY_TIMEOUT_S):
            return self._switch_failed("switch_controller unavailable")

        request = SwitchController.Request()
        request.start_controllers = required_activate
        request.stop_controllers = required_deactivate
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.start_asap = True
        request.timeout.sec = int(SWITCH_TIMEOUT_S)
        response = _wait_result(
            self._cli_switch.call_async(request), SWITCH_TIMEOUT_S + 1.0)
        if response is None or not response.ok:
            return self._switch_failed("controller_manager refused the switch")

        states = self._controller_states()
        mismatches = [
            name for name in activate if states.get(name) != "active"] + [
            name for name in deactivate
            if name not in activate and states.get(name) != "inactive"]
        if mismatches:
            return self._switch_failed(
                "controller switch state mismatch: " + ", ".join(mismatches))
        return True

    def _switch_failed(self, message: str) -> bool:
        self._set_msg(message)
        return False

    def _srv_disable(self, request, response):
        del request
        self._pause_targets()
        with self._lock:
            handle = self._goal_handle
            self._goal_handle = None
            controllers = list(dict.fromkeys(
                name for name in (self._fpc, self._jtc) if name))
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

        states = self._controller_states()
        if any(name not in states for name in controllers):
            response.success = False
            response.message = "disabled locally; controller state unavailable"
            return response
        active = [name for name in controllers if states[name] == "active"]
        if active and not self._switch(activate=[], deactivate=active):
            response.success = False
            response.message = "disabled locally; controller release not confirmed"
            return response
        self._set_msg("DISABLED; motion controllers inactive")
        response.success = True
        response.message = "disabled; motion controllers inactive"
        return response

    def _pause_targets(self) -> None:
        with self._lock:
            self._enabled = False
            self._last_target = None
            self._traj = None
            self._decoupled_active = False
            self._cached_goal = None
            self._cached_target_xyz = None
            self._cached_target_quat = None
            self._rejected_target_xyz = None
            self._rejected_target_quat = None

    def _return_to_start(self, timeout: float = RETURN_TIMEOUT_S):
        with self._lock:
            previous_handle = self._goal_handle
        self._pause_targets()
        result = super()._return_to_start(timeout)
        if not result[0]:
            self._cancel_new_goal(previous_handle)
            return result
        result = self._confirm_return_result(result)
        if not result[0]:
            self._cancel_new_goal(previous_handle)
            return result
        with self._lock:
            self._goal_handle = None
        message = result[1] + "; disabled in JTC hold"
        self._set_msg(message)
        return True, message

    def _confirm_return_result(self, result):
        with self._lock:
            handle = self._goal_handle
        if handle is None:
            return False, "return-to-start result unavailable"
        response = _wait_result(handle.get_result_async(), 2.0)
        if response is None:
            return False, "return-to-start action result not confirmed"
        code = int(getattr(response.result, "error_code", -1))
        if (response.status == GoalStatus.STATUS_SUCCEEDED and
                code == FollowJointTrajectory.Result.SUCCESSFUL):
            return result
        detail = str(getattr(response.result, "error_string", "") or "")
        return False, (
            f"return-to-start action failed (status={response.status}, "
            f"code={code})" + (f": {detail}" if detail else ""))

    def _cancel_new_goal(self, previous_handle) -> None:
        with self._lock:
            handle = self._goal_handle
        if handle is not None and handle is not previous_handle:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    def _seed_fpc_current(self) -> None:
        return


def _spin(node_type, args: Optional[Sequence[str]]) -> None:
    rclpy.init(args=list(args) if args is not None else None)
    node = node_type()
    executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: Optional[Sequence[str]] = None) -> None:
    _spin(G1PoseCommander, args)


def dashboard_main(args: Optional[Sequence[str]] = None) -> None:
    _spin(G1CommanderDashboard, args)


if __name__ == "__main__":
    main()

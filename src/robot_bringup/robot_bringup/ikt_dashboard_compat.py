"""G1-specific presentation fixes for the IKT Commander dashboard."""

from pathlib import Path
from typing import Tuple

from geometry_msgs.msg import PoseStamped
from ikt_pose_commander.dashboard_node import CommanderDashboard

from robot_bringup.dashboard_compat_node import (
    MimicJointSpec,
    _correct_mimic_transforms,
    _filter_mimic_snapshot,
    _hidden_mimic_names,
    _parse_mimic_joints,
)


_VIEWER_TARGET_START = "function sendProxyTarget(stream) {"
_VIEWER_TARGET_END = '\n\n// "Snap target -> link"'
_VIEWER_TARGET_SENDER = '''let _targetPostInFlight = false;
let _pendingTargetBody = null;

async function flushLatestTarget() {
    _targetPostInFlight = true;
    while (_pendingTargetBody !== null) {
        const body = _pendingTargetBody;
        _pendingTargetBody = null;
        const out = await api("/api/target", body);
        if (out && out.ok === false) actionMsg("target failed: " + (out.message || ""));
    }
    _targetPostInFlight = false;
}

function sendProxyTarget(stream) {
    const body = targetPoseBody();
    if (!stream) {
        api("/api/send", body).then((out) => {
            if (out && out.ok === false) actionMsg("target failed: " + (out.message || ""));
        });
        return;
    }
    _pendingTargetBody = body;
    if (!_targetPostInFlight) flushLatestTarget();
}'''


def patch_viewer_target_sender(source: str) -> str:
    if source.count(_VIEWER_TARGET_START) != 1:
        raise RuntimeError("IKT viewer target sender no longer matches G1 overlay")
    start = source.index(_VIEWER_TARGET_START)
    end = source.find(_VIEWER_TARGET_END, start)
    if end < 0:
        raise RuntimeError("IKT viewer target sender end marker is unavailable")
    return source[:start] + _VIEWER_TARGET_SENDER + source[end:]


class G1CommanderDashboard(CommanderDashboard):
    def __init__(self) -> None:
        self._mimic_specs: Tuple[MimicJointSpec, ...] = ()
        self._hidden_mimic_links = set()
        self._hidden_mimic_joints = set()
        super().__init__()
        self._install_viewer_overlay()
        old_target_pub = self._target_pub
        self._target_pub = self.create_publisher(
            PoseStamped, f"{self._ns}/target_pose", 1)
        self.destroy_publisher(old_target_pub)

    def _install_viewer_overlay(self) -> None:
        httpd = self._httpd
        if httpd is None:
            raise RuntimeError("IKT dashboard HTTP server is unavailable")
        handler_type = httpd.RequestHandlerClass
        original_serve_static = handler_type._serve_static
        module = __import__(
            CommanderDashboard.__module__, fromlist=["__file__"])
        viewer_path = Path(module.__file__).resolve().parent / "static" / "viewer.js"
        viewer = patch_viewer_target_sender(
            viewer_path.read_text(encoding="utf-8")).encode("utf-8")

        def serve_static(handler, relpath):
            if relpath != "viewer.js":
                return original_serve_static(handler, relpath)
            handler.send_response(200)
            handler.send_header("Content-Type", "text/javascript")
            handler.send_header("Content-Length", str(len(viewer)))
            handler.send_header("Cache-Control", "no-store")
            handler.end_headers()
            handler.wfile.write(viewer)
            return None

        handler_type._serve_static = serve_static

    def _build_target_msg(self, xyz: list, quat: list,
                          frame_id: str) -> PoseStamped:
        message = super()._build_target_msg(xyz, quat, frame_id)
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        return message

    def _on_urdf(self, message) -> None:
        super()._on_urdf(message)
        try:
            specs = _parse_mimic_joints(message.data)
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f"cannot parse mimic joints: {error}")
            specs = ()
        hidden_links, hidden_joints = _hidden_mimic_names(specs)
        with self._lock:
            self._mimic_specs = specs
            self._hidden_mimic_links = hidden_links
            self._hidden_mimic_joints = hidden_joints

    def snapshot(self) -> dict:
        state = super().snapshot()
        with self._lock:
            model = self._model
        if model is not None:
            links = model.link_frame_names()
            if links:
                state["root_frame"] = links[0]
        if not state.get("has_model_viz"):
            return state
        with self._lock:
            specs = self._mimic_specs
            positions = dict(self._joint_pos)
            hidden_links = set(self._hidden_mimic_links)
            hidden_joints = set(self._hidden_mimic_joints)
        state["link_tf"] = _correct_mimic_transforms(
            state.get("link_tf", {}), specs, positions)
        return _filter_mimic_snapshot(state, hidden_links, hidden_joints)

    def stream_pose(self, xyz: list, quat: list, frame_id: str) -> dict:
        target = ([float(value) for value in xyz],
                  [float(value) for value in quat], frame_id)
        with self._lock:
            active = not self._sm_forwarding
            if active:
                self._stream_pose = target
                self._stream_active = True
        if active:
            self._target_pub.publish(self._build_target_msg(*target))
        return {"ok": True, "xyz": target[0],
                "frame_id": frame_id or self._base_frame}

    def call_trigger(self, enable: bool, timeout: float = 35.0) -> dict:
        return super().call_trigger(enable, timeout)

    def call_return_to_start(self, timeout: float = 55.0) -> dict:
        return super().call_return_to_start(timeout)

"""Topic-based client for the Unitree G1 motion switcher API."""

from __future__ import annotations

import json
import threading
import time
from typing import Optional, Tuple

from unitree_api.msg import Request, Response


CHECK_MODE_API_ID = 1001
SELECT_MODE_API_ID = 1002
RELEASE_MODE_API_ID = 1003


class MotionSwitcherClient:
    """Synchronously call the asynchronous Unitree request/response topics."""

    def __init__(self, node, callback_group=None) -> None:
        self._node = node
        self._condition = threading.Condition()
        self._call_lock = threading.Lock()
        self._pending_id: Optional[int] = None
        self._response: Optional[Response] = None
        self._publisher = node.create_publisher(
            Request, "/api/motion_switcher/request", 1)
        self._subscription = node.create_subscription(
            Response,
            "/api/motion_switcher/response",
            self._on_response,
            1,
            callback_group=callback_group,
        )

    def _on_response(self, message: Response) -> None:
        with self._condition:
            if message.header.identity.id != self._pending_id:
                return
            self._response = message
            self._condition.notify_all()

    def call(
            self, api_id: int, parameter: str = "",
            timeout_s: float = 1.0) -> Optional[Response]:
        with self._call_lock:
            identity = time.monotonic_ns()
            request = Request()
            request.header.identity.id = identity
            request.header.identity.api_id = api_id
            request.parameter = parameter

            with self._condition:
                self._pending_id = identity
                self._response = None
                self._publisher.publish(request)
                deadline = time.monotonic() + timeout_s
                while self._response is None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        self._pending_id = None
                        return None
                    self._condition.wait(remaining)
                response = self._response
                self._pending_id = None
                self._response = None
                return response

    def check_mode(self, timeout_s: float) -> Tuple[bool, str, str]:
        response = self.call(CHECK_MODE_API_ID, timeout_s=timeout_s)
        if response is None:
            return False, "", "motion switcher CheckMode timed out"
        if response.header.status.code != 0:
            return (
                False,
                "",
                f"motion switcher CheckMode failed: "
                f"status={response.header.status.code}",
            )
        try:
            data = json.loads(response.data or "{}")
        except (TypeError, ValueError) as exc:
            return False, "", f"invalid CheckMode response: {exc}"
        name = data.get("name", "")
        if not isinstance(name, str):
            return False, "", "invalid CheckMode response: name is not a string"
        return True, name, ""

    def release_mode(self, timeout_s: float) -> Tuple[bool, str]:
        response = self.call(RELEASE_MODE_API_ID, timeout_s=timeout_s)
        if response is None:
            return False, "motion switcher ReleaseMode timed out"
        if response.header.status.code != 0:
            return (
                False,
                f"motion switcher ReleaseMode failed: "
                f"status={response.header.status.code}",
            )
        return True, ""

    def select_mode(self, name: str, timeout_s: float) -> Tuple[bool, str]:
        parameter = json.dumps({"name": name}, separators=(",", ":"))
        response = self.call(
            SELECT_MODE_API_ID, parameter=parameter, timeout_s=timeout_s)
        if response is None:
            return False, f"motion switcher SelectMode({name!r}) timed out"
        if response.header.status.code != 0:
            return (
                False,
                f"motion switcher SelectMode({name!r}) failed: "
                f"status={response.header.status.code}",
            )
        return True, ""
import json
import threading
import time

from unitree_api.msg import Response

from unitree_g1_description.motion_switcher import (
    CHECK_MODE_API_ID,
    SELECT_MODE_API_ID,
    MotionSwitcherClient,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self):
        self.publisher = _Publisher()
        self.callback = None

    def create_publisher(self, *_args, **_kwargs):
        return self.publisher

    def create_subscription(self, _type, _topic, callback, *_args, **_kwargs):
        self.callback = callback
        return object()


def _respond(node, data="", status=0):
    deadline = time.monotonic() + 1.0
    while not node.publisher.messages and time.monotonic() < deadline:
        time.sleep(0.001)
    request = node.publisher.messages[-1]
    response = Response()
    response.header.identity.id = request.header.identity.id
    response.header.identity.api_id = request.header.identity.api_id
    response.header.status.code = status
    response.data = data
    node.callback(response)


def test_check_mode_matches_response_identity_and_parses_name():
    node = _Node()
    client = MotionSwitcherClient(node)
    responder = threading.Thread(
        target=_respond, args=(node, '{"form":"0","name":"ai"}'))
    responder.start()

    result = client.check_mode(timeout_s=1.0)
    responder.join()

    assert result == (True, "ai", "")
    assert node.publisher.messages[0].header.identity.api_id == CHECK_MODE_API_ID


def test_select_mode_uses_official_json_parameter():
    node = _Node()
    client = MotionSwitcherClient(node)
    responder = threading.Thread(target=_respond, args=(node,))
    responder.start()

    result = client.select_mode("ai", timeout_s=1.0)
    responder.join()

    assert result == (True, "")
    request = node.publisher.messages[0]
    assert request.header.identity.api_id == SELECT_MODE_API_ID
    assert json.loads(request.parameter) == {"name": "ai"}


def test_check_mode_timeout_is_reported():
    client = MotionSwitcherClient(_Node())

    ok, mode, error = client.check_mode(timeout_s=0.01)

    assert not ok
    assert mode == ""
    assert "timed out" in error
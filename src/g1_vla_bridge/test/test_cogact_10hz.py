# pyright: reportArgumentType=false, reportCallIssue=false

import json
from email import policy
from email.parser import BytesParser
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from g1_vla_bridge.backends.cogact_unitree import CogACTUnitreeBackend, SPEC, build_payload, parse_action
from g1_vla_bridge.control_history import ControlHistory
from test_cogact_unitree_backend import _observation, action_response, history_config


@pytest.mark.parametrize('suffix', ['TRANS', 'ROT_MAT', 'GRIPPER'])
def test_bad_shape_rejected(suffix):
    body = action_response()
    body['action'][f'ROBOT_LEFT_{suffix}'].pop()
    with pytest.raises(ValueError, match='expected'):
        parse_action(body)


@pytest.mark.parametrize('history_length', [1, 15, 16, 30])
def test_empty_short_full_over_real_local_http(history_length):
    received = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            _ = args
            pass

        def reply(self, body):
            data = json.dumps(body).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self.reply({'model': 'CogACT', 'status': 'healthy'}
                       if self.path == '/api/health' else history_config(history_length=history_length))

        def do_POST(self):
            assert self.path == '/api/inference'
            data = self.rfile.read(int(self.headers['Content-Length']))
            message = BytesParser(policy=policy.default).parsebytes(
                f'Content-Type: {self.headers["Content-Type"]}\r\n\r\n'.encode() + data)
            parts = {part.get_param('name', header='content-disposition'): part
                     for part in message.iter_parts()}
            assert set(parts) == {'json', 'image_0', 'image_1', 'image_2'}
            assert parts['json'].get_filename() == 'query.json'
            assert parts['json'].get_content_type() == 'application/json'
            for index in range(3):
                image = np.frombuffer(parts[f'image_{index}'].get_payload(decode=True), np.uint8)
                assert cv2.imdecode(image, 1).shape == (360, 640, 3)
            received.append(json.loads(parts['json'].get_payload(decode=True)))
            self.reply(action_response())

    server = HTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever)
    worker.start()
    backend = CogACTUnitreeBackend(
        f'http://127.0.0.1:{server.server_port}/api/inference', history_length=history_length)
    backend._session.trust_env = False
    backend.dump = Mock(side_effect=AssertionError('disk writes forbidden'))
    try:
        backend.configure()
        current = _observation()
        for count in (0, 3, history_length, history_length + 7):
            history = ControlHistory(history_length=backend.history_length)
            for index in range(count):
                measured = {side: pose.copy() for side, pose in current.poses.items()}
                command = {side: pose.copy() for side, pose in current.poses.items()}
                for side in measured:
                    measured[side][0] = index
                    command[side][0] = index + 100
                history.append(index / 10 + .01, command, measured,
                               current.grippers, current.grippers, index / 10)
            current.history = history.snapshot()
            assert backend.infer(current).horizon == 30
            backend.dump.assert_not_called()
            assert received[-1] == build_payload(current, SPEC.frame.transform(), history_length)
            for field, offset in (('history_state', 0), ('history_action', 100)):
                if count:
                    assert len(received[-1][field]) == 6
                    assert all(len(value) == min(count, history_length) for value in received[-1][field].values())
                    for side in ('LEFT', 'RIGHT'):
                        positions = np.asarray(received[-1][field][f'ROBOT_{side}_TRANS'])[:, 0]
                        np.testing.assert_allclose(positions, np.arange(max(0, count - history_length), count) + offset)
                        assert np.shape(received[-1][field][f'ROBOT_{side}_ROT_MAT']) == (min(count, history_length), 3, 3)
                        assert np.shape(received[-1][field][f'ROBOT_{side}_GRIPPER']) == (min(count, history_length), 1)
                else:
                    assert received[-1][field] is None
    finally:
        backend.close()
        server.shutdown()
        worker.join()
        server.server_close()

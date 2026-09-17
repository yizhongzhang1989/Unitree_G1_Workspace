import json
from email import policy
from email.parser import BytesParser
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import numpy as np
import pytest

from g1_vla_bridge.backends.cogact_unitree import CogACTUnitreeBackend, SPEC, build_payload, parse_action
from g1_vla_bridge.control_history import ControlHistory
from test_cogact_unitree_backend import _observation, history_config


def response():
    action = {}
    for side in ('LEFT', 'RIGHT'):
        action[f'ROBOT_{side}_TRANS'] = np.zeros((30, 3)).tolist()
        action[f'ROBOT_{side}_ROT_MAT'] = np.tile(np.eye(3), (30, 1, 1)).tolist()
        action[f'ROBOT_{side}_GRIPPER'] = np.zeros((30, 1)).tolist()
    return {'action': action, 'action_type_info': {'translation': 'abs', 'rotation': 'abs'}}


@pytest.mark.parametrize('suffix', ['TRANS', 'ROT_MAT', 'GRIPPER'])
def test_bad_shape_rejected(suffix):
    body = response()
    body['action'][f'ROBOT_LEFT_{suffix}'].pop()
    with pytest.raises(ValueError, match='expected'):
        parse_action(body)


def test_finite_and_real_extrinsics():
    current = _observation()
    current.grippers['left'] = float('nan')
    with pytest.raises(ValueError, match='nonfinite'):
        build_payload(current, SPEC.frame.transform())
    current.grippers['left'] = 0.
    current.camera_poses['head'] = np.eye(4)
    with pytest.raises(ValueError, match='placeholder'):
        build_payload(current, SPEC.frame.transform())


def test_empty_short_full_over_real_local_http():
    received = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
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
                       if self.path == '/api/health' else history_config())

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
            received.append(json.loads(parts['json'].get_payload(decode=True)))
            self.reply(response())

    server = HTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever)
    worker.start()
    backend = CogACTUnitreeBackend(f'http://127.0.0.1:{server.server_port}/api/inference')
    backend._session.trust_env = False
    try:
        backend.configure()
        current = _observation()
        for count in (0, 3, 30):
            history = ControlHistory()
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
            for field, offset in (('history_state', 0), ('history_action', 100)):
                if count:
                    assert len(received[-1][field]) == 6
                    assert all(len(value) == min(count, 15) for value in received[-1][field].values())
                    for side in ('LEFT', 'RIGHT'):
                        positions = np.asarray(received[-1][field][f'ROBOT_{side}_TRANS'])[:, 0]
                        np.testing.assert_allclose(positions, np.arange(max(0, count - 15), count) + offset)
                else:
                    assert received[-1][field] is None
    finally:
        backend.close()
        server.shutdown()
        worker.join()
        server.server_close()

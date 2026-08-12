import json
import math
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import ProxyHandler, build_opener

from unitree_hg.msg import IMUState, LowState

from robot_bringup.lowlevel.dashboard_node import (
    G1_JOINT_NAMES,
    MOTOR_FIELDS,
    MOTOR_NAMES,
    MOTOR_SLOTS,
    LowLevelDashboard,
    _handler,
    imu_payload,
    lowstate_header,
    motor_rows,
    under,
)


def _lowstate() -> LowState:
    message = LowState()
    message.mode_pr = 1
    message.mode_machine = 5
    message.tick = 4321
    message.crc = 0xDEADBEEF
    for index, motor in enumerate(message.motor_state):
        motor.mode = 1
        motor.q = index / 10.0
        motor.dq = -index / 100.0
        motor.tau_est = index / 4.0
        motor.vol = 47.5
        motor.temperature = [40 + index, 50 + index]
        motor.sensor = [index, index * 2]
    message.motor_state[13].motorstate = 0x200
    message.wireless_remote = list(range(40))
    return message


class MotorRowsTest(unittest.TestCase):
    def test_every_slot_follows_the_declared_field_order(self) -> None:
        rows = motor_rows(_lowstate())
        self.assertEqual(len(rows), MOTOR_SLOTS)
        self.assertTrue(all(len(row) == len(MOTOR_FIELDS) for row in rows))
        column = {field: index for index, field in enumerate(MOTOR_FIELDS)}
        row = rows[13]
        self.assertAlmostEqual(row[column['q']], 1.3, places=6)
        self.assertAlmostEqual(row[column['dq']], -0.13, places=6)
        self.assertAlmostEqual(row[column['tau_est']], 3.25, places=6)
        self.assertEqual(row[column['temp_shell']], 53)
        self.assertEqual(row[column['temp_winding']], 63)
        self.assertEqual(row[column['motorstate']], 0x200)
        self.assertEqual(row[column['sensor0']], 13)
        self.assertEqual(row[column['sensor1']], 26)

    def test_motor_index_13_is_waist_roll(self) -> None:
        """LowState 没有关节名，错一位就会把故障码标到别的电机上。"""
        self.assertEqual(len(G1_JOINT_NAMES), 29)
        self.assertEqual(len(MOTOR_NAMES), MOTOR_SLOTS)
        self.assertEqual(MOTOR_NAMES[0], 'left_hip_pitch_joint')
        self.assertEqual(MOTOR_NAMES[13], 'waist_roll_joint')
        self.assertEqual(MOTOR_NAMES[28], 'right_wrist_yaw_joint')
        self.assertEqual(MOTOR_NAMES[29], 'reserved_29')

    def test_non_finite_values_survive_json(self) -> None:
        """json 会把 NaN 写成非法字面量，浏览器 JSON.parse 直接抛。"""
        message = _lowstate()
        message.motor_state[2].q = math.nan
        message.motor_state[3].tau_est = math.inf
        column = {field: index for index, field in enumerate(MOTOR_FIELDS)}
        rows = motor_rows(message)
        self.assertIsNone(rows[2][column['q']])
        self.assertIsNone(rows[3][column['tau_est']])
        self.assertNotIn('NaN', json.dumps(rows))

    def test_missing_lowstate_yields_empty_payload(self) -> None:
        self.assertEqual(motor_rows(None), [])
        self.assertIsNone(lowstate_header(None))


class HeaderAndImuTest(unittest.TestCase):
    def test_header_exposes_raw_frame_metadata(self) -> None:
        header = lowstate_header(_lowstate())
        assert header is not None
        self.assertEqual(header['mode_pr'], 1)
        self.assertEqual(header['mode_machine'], 5)
        self.assertEqual(header['tick'], 4321)
        self.assertEqual(header['crc'], 0xDEADBEEF)
        self.assertEqual(len(header['wireless_remote']), 80)
        self.assertTrue(header['wireless_remote'].startswith('000102'))

    def test_quaternion_keeps_the_firmware_wxyz_order(self) -> None:
        imu = IMUState()
        imu.quaternion = [1.0, 2.0, 3.0, 4.0]
        imu.rpy = [0.1, 0.2, 0.3]
        imu.temperature = 37
        payload = imu_payload(imu)
        assert payload is not None
        self.assertEqual(payload['quaternion'], [1.0, 2.0, 3.0, 4.0])
        self.assertEqual(payload['temperature'], 37)

    def test_absent_imu_is_null_not_zeros(self) -> None:
        self.assertIsNone(imu_payload(None))


class ConfigurationTest(unittest.TestCase):
    def _validate(self, port: int, idle: float) -> None:
        stub = SimpleNamespace(_idle_release=idle)
        LowLevelDashboard._validate_configuration(stub, port)

    def test_accepts_sane_values(self) -> None:
        self._validate(8210, 3.0)

    def test_rejects_out_of_range_port_and_idle_release(self) -> None:
        for port, idle in ((70000, 3.0), (-1, 3.0),
                           (8210, 0.0), (8210, -1.0), (8210, math.nan)):
            with self.subTest(port=port, idle=idle):
                with self.assertRaises(ValueError):
                    self._validate(port, idle)


class PathContainmentTest(unittest.TestCase):
    def test_traversal_and_absolute_paths_are_refused(self) -> None:
        base = Path('/tmp/static')
        self.assertEqual(under(base, 'index.html'), base / 'index.html')
        for relative in ('', '../setup.py', '/etc/passwd', 'a/../../b'):
            with self.subTest(relative=relative):
                self.assertIsNone(under(base, relative))


class _FakeDashboard:
    def __init__(self) -> None:
        self.polls = 0

    def snapshot(self) -> dict:
        self.polls += 1
        return {'motors': [], 'seq': self.polls}

    def read_static(self, relative: str):
        if relative != 'index.html':
            return None
        return b'<title>lowlevel test</title>', 'text/html'


class HandlerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.node = _FakeDashboard()
        self.server = ThreadingHTTPServer(
            ('127.0.0.1', 0), _handler(self.node))
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f'http://127.0.0.1:{self.server.server_address[1]}'
        self.opener = build_opener(ProxyHandler({}))

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1.0)

    def test_serves_page_and_state(self) -> None:
        with self.opener.open(self.base_url + '/', timeout=2.0) as response:
            self.assertIn(b'lowlevel test', response.read())
        with self.opener.open(
                self.base_url + '/api/state', timeout=2.0) as response:
            self.assertEqual(json.loads(response.read())['seq'], 1)

    def test_constant_metadata_is_off_the_polling_path(self) -> None:
        with self.opener.open(
                self.base_url + '/api/motors', timeout=2.0) as response:
            meta = json.loads(response.read())
        self.assertEqual(meta['motor_fields'], list(MOTOR_FIELDS))
        self.assertEqual(meta['motor_names'], list(MOTOR_NAMES))
        self.assertEqual(meta['named_motors'], len(G1_JOINT_NAMES))
        self.assertNotIn('motor_names', self.node.snapshot())

    def test_unknown_and_traversal_paths_are_404(self) -> None:
        for path in ('/nope.js', '/../setup.py'):
            with self.subTest(path=path):
                with self.assertRaises(HTTPError) as caught:
                    self.opener.open(self.base_url + path, timeout=2.0)
                self.assertEqual(caught.exception.code, 404)


if __name__ == '__main__':
    unittest.main()

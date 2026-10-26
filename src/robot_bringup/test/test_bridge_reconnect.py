import signal
import sys
from pathlib import Path
from unittest.mock import Mock, patch

import pytest
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events.process import ProcessExited

from robot_bringup.end_effectors.nodes import bridge
from robot_bringup.end_effectors.topology import CanBus


def _bridge_arguments():
    with patch("robot_bringup.end_effectors.nodes.Node") as node_type:
        bridge([CanBus(name="can0", channel_id=0)], [], [])
    return node_type.call_args.kwargs


def test_bridge_reconnect_is_bounded():
    on_exit = _bridge_arguments()["on_exit"]
    context = LaunchContext()
    failure = Mock(spec=ProcessExited, returncode=1)

    with patch("robot_bringup.end_effectors.nodes.get_logger") as logger:
        for _ in range(3):
            assert on_exit(failure, context) is None
        assert logger.return_value.warning.call_count == 3
        assert isinstance(on_exit(failure, context), Shutdown)
        logger.return_value.error.assert_called_once()


def test_bridge_reconnect_budget_is_not_shared_between_launches():
    first = _bridge_arguments()["on_exit"]
    second = _bridge_arguments()["on_exit"]
    context = LaunchContext()
    failure = Mock(spec=ProcessExited, returncode=1)
    for _ in range(3):
        assert first(failure, context) is None
    assert isinstance(first(failure, context), Shutdown)
    assert second(failure, context) is None


@pytest.mark.parametrize("returncode", [0, -signal.SIGINT, -signal.SIGTERM, -signal.SIGSEGV])
def test_bridge_does_not_reconnect_after_clean_exit_or_unexpected_signal(returncode):
    on_exit = _bridge_arguments()["on_exit"]
    event = Mock(spec=ProcessExited, returncode=returncode)
    assert isinstance(on_exit(event, LaunchContext()), Shutdown)


def test_bridge_does_not_reconnect_during_shutdown():
    on_exit = _bridge_arguments()["on_exit"]
    context = LaunchContext()
    context._set_is_shutdown(True)
    assert on_exit(Mock(spec=ProcessExited, returncode=1), context) is None


@pytest.mark.parametrize(
    "failures,cancel_pending,expected_starts",
    [(2, False, 3), (10, False, 4), (10, True, 1)],
    ids=["recovers", "exhausts-retries", "shutdown-cancels-retry"],
)
def test_bridge_reconnect_process_lifecycle(
        tmp_path: Path, failures: int, cancel_pending: bool,
        expected_starts: int):
    attempts_path = tmp_path / "attempts"
    fake_bridge = """
import signal
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
attempts = int(path.read_text()) + 1 if path.exists() else 1
path.write_text(str(attempts))
if attempts <= int(sys.argv[2]):
    sys.exit(1)
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
while True:
    time.sleep(0.1)
"""
    arguments = _bridge_arguments()
    process = ExecuteProcess(
        cmd=[sys.executable, "-c", fake_bridge, str(attempts_path), str(failures)],
        respawn=arguments.get("respawn", False),
        respawn_delay=arguments.get("respawn_delay"),
        on_exit=arguments["on_exit"],
    )
    companion = ExecuteProcess(cmd=[
        sys.executable, "-c",
        "import signal, sys, time; "
        "signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); time.sleep(30)",
    ])
    starts = []
    exits = []
    companion_exits = []
    shutdowns = []
    timed_out = []

    def on_start(event, _context):
        starts.append(event.pid)
        if failures == 2 and len(starts) == expected_starts:
            return TimerAction(
                period=0.3, actions=[Shutdown(reason="reconnect test completed")])
        return None

    def on_exit(event, _context):
        exits.append(event.returncode)
        if cancel_pending:
            return TimerAction(
                period=0.1, actions=[Shutdown(reason="cancel pending reconnect")])
        return None

    def on_shutdown(_event, _context):
        shutdowns.append((len(starts), len(companion_exits)))

    service = LaunchService(noninteractive=True)
    service.context.register_event_handler(
        OnShutdown(on_shutdown=on_shutdown))
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnProcessStart(target_action=process, on_start=on_start)),
        RegisterEventHandler(OnProcessExit(target_action=process, on_exit=on_exit)),
        RegisterEventHandler(OnProcessExit(
            target_action=companion,
            on_exit=lambda event, _context: companion_exits.append(event.returncode))),
        TimerAction(period=7.0, actions=[
            OpaqueFunction(
                function=lambda _context: timed_out.append(True)),
            Shutdown(reason="reconnect test timed out"),
        ]),
        companion,
        process,
    ]))
    service.run()

    assert not timed_out
    assert len(starts) == expected_starts
    assert int(attempts_path.read_text()) == expected_starts
    assert exits == ([1, 1, 0] if failures == 2 else [1] * expected_starts)
    assert shutdowns == [(expected_starts, 0)]
    assert companion_exits == [0]

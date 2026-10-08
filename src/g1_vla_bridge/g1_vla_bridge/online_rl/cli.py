"""Task, Enter, chunk-bound score, then Enter or /home."""

import json
import os
import select
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from g1_vla_bridge.online_rl.session import score_reward
from g1_vla_bridge.vla_cli import VlaCli, input_action


def score_command(line, record):
    if record['state'] not in ('completed', 'interrupted', 'skipped'):
        raise ValueError('This chunk is not ready for scoring')
    reward = score_reward(line)
    if record['state'] != 'completed' and reward is not None:
        raise ValueError('Interrupted or unexecuted chunk: use /null')
    return dict(command='score', request_id=record['request_id'], id=record['id'], reward=reward)


def prompt_text(status, rl, submitted=None):
    if submitted:
        return f"feedback pending [{submitted['id']}]> "
    record = rl.get('record') or {}
    state = record.get('state')
    if state == 'completed':
        return (f"score [{record['id']}, v{record.get('policy_version')}, "
                f"{record['executed_steps']} steps] 1..5 (Enter=3), /null> ")
    if state in ('interrupted', 'skipped'):
        return f"unrated [{record['id']}, {state}] /null> "
    if state == 'uncertain':
        return f"verify execution [{record['id']}] /resolve N> "
    if state in ('prepared', 'received', 'executing', 'feedback_pending'):
        return f'{state}> '
    if rl.get('fault'):
        return 'RL fault (/status)> '
    if rl.get('phase') != 'collecting':
        return f"RL {rl.get('phase', 'connecting')}> "
    if not status.get('task'):
        return 'task> '
    return 'vla (Enter=execute, /home)> '


class OnlineRLCli(VlaCli):
    def __init__(self):
        super().__init__('online_rl_cli', '/online_rl_bridge', modes=False)
        self._rl = {}
        self._submitted = None
        self._last_submit = 0.
        self._feedback = self.create_publisher(String, '/online_rl_bridge/feedback', 10)
        self.create_subscription(String, '/online_rl_bridge/rl_status', self._on_rl_status, 10)

    def _on_rl_status(self, message):
        try:
            self._rl = json.loads(message.data)
        except ValueError:
            return
        record = self._rl.get('record') or {}
        if not self._submitted or record.get('request_id') != self._submitted['request_id']:
            return
        if self._submitted['command'] == 'resolve' and record.get('state') == 'interrupted':
            print('\nExecution count recorded; submit /null.')
        elif record.get('feedback_confirmed'):
            print(f"\nFeedback confirmed [{record['id']}]: {record['feedback']['reward']}")
        elif record.get('state') == 'expired':
            print('\nFeedback expired without confirmation; record retained.')
        else:
            return
        self._submitted = None

    def wait_ready(self, timeout=5.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._start.service_is_ready() and self._next.service_is_ready() and self._status and self._rl:
                return True
            rclpy.spin_once(self, timeout_sec=.1)
        return False

    def submit(self, command):
        self._submitted = command
        self._feedback.publish(String(data=json.dumps(command, allow_nan=False)))
        self._last_submit = time.monotonic()

    def retry_submission(self):
        if self._submitted and time.monotonic() - self._last_submit >= 1.:
            self.submit(self._submitted)

    def execute_one(self):
        if self._rl.get('pending') or self._submitted:
            print('Resolve the current chunk and confirm feedback first.')
            return
        if self._rl.get('phase') != 'collecting' or self._rl.get('fault'):
            print('RL is not collecting. Use /status.')
            return
        super().execute_one()

    def handle_line(self, line, displayed_record=None):
        action, value = input_action(line)
        if value in ('/stop', '/home', '/start', '/engage', '/estop'):
            self.call(getattr(self, '_' + value[1:]), value[1:])
            if value == '/estop':
                self.call(self._stop, 'stop')
            return True
        if value == '/quit':
            return False
        if value == '/status':
            print(json.dumps(self._rl, indent=2, ensure_ascii=False))
            return True
        if self._submitted:
            print('Waiting for confirmation; the existing feedback will be retried unchanged.')
            return True
        record = displayed_record or {}
        try:
            if value.startswith('/resolve'):
                parts = value.split()
                if len(parts) != 2 or record.get('state') != 'uncertain':
                    raise ValueError('After physical verification: /resolve <model-step count>')
                steps = int(parts[1])
                if not 0 <= steps <= record['planned_steps']:
                    raise ValueError('Model-step count outside the planned chunk')
                self.submit(dict(command='resolve', request_id=record['request_id'],
                                 id=record['id'], executed_steps=steps))
            elif record.get('state') in ('completed', 'interrupted', 'skipped'):
                self.submit(score_command(line, record))
            elif self._rl.get('pending'):
                print('Current chunk is pending. /stop, /home and /estop remain available.')
            elif action == 'next':
                self.execute_one()
            elif action == 'task':
                self.set_task(value)
            else:
                print(f'Unknown command: {value}')
        except (ValueError, KeyError) as error:
            print(error)
        return True

    def run(self):
        print('Enter a task, then Enter to execute. Score 1..5 (Enter=3); /null = unobserved.')
        print('/home /start /stop /engage /estop /status /quit; /home does not confirm arrival.')
        previous_prompt = None
        displayed_record = None
        pending_input = bytearray()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=.05)
            self.retry_submission()
            prompt = prompt_text(self._status, self._rl, self._submitted)
            if prompt != previous_prompt:
                if previous_prompt is not None:
                    print()
                print(prompt, end='', flush=True)
                displayed_record = dict(self._rl.get('record') or {})
                previous_prompt = prompt
            if b'\n' not in pending_input:
                if not select.select([sys.stdin], [], [], 0)[0]:
                    continue
                data = os.read(sys.stdin.fileno(), 4096)
                if not data:
                    break
                pending_input.extend(data)
                if b'\n' not in pending_input:
                    continue
            line, _, pending_input = pending_input.partition(b'\n')
            if not self.handle_line(line.decode('utf-8', errors='replace'), displayed_record):
                break
            previous_prompt = None


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    cli = OnlineRLCli()
    try:
        if not cli.wait_ready():
            raise RuntimeError('Online RL bridge unavailable; start online_rl.launch.py first')
        cli.run()
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        print(error, file=sys.stderr)
    finally:
        if cli._status.get('running') or cli._status.get('inference_active'):
            cli.call(cli._stop, 'stop on CLI exit')
        cli.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

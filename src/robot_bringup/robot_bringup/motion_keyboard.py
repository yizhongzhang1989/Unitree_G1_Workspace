"""组合启动运动任务，并在当前终端运行键盘控制台"""

import argparse
import os
import signal
import subprocess
import sys
import tempfile


def main(args=None):
    parser = argparse.ArgumentParser(description='启动速度任务、上肢任务与键盘控制台')
    parser.add_argument('--upper-task', choices=('float', 'vr', 'vla'), default='float')
    parser.add_argument('--lower-config', default='')
    parser.add_argument('--upper-config', default='')
    options, ros_arguments = parser.parse_known_args(args)
    if not sys.stdin.isatty():
        parser.error('键盘控制需要交互终端')
    from lower_velocity_tracking.teleop_keyboard import main as control

    arguments = ['lower_task:=velocity', f'upper_task:={options.upper_task}',
                 f'lower_config:={options.lower_config}', f'upper_config:={options.upper_config}']
    with tempfile.NamedTemporaryFile(prefix='g1-motion-keyboard-', suffix='.log', delete=False) as log:
        print(f'任务启动日志：{log.name}', flush=True)
        # 独立进程组只包含本入口启动的任务，硬件栈由其他终端管理
        process = subprocess.Popen(
            ['ros2', 'launch', 'robot_bringup', 'motion_tasks.launch.py', *arguments],
            start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
        try:
            control(args=ros_arguments[1:] if ros_arguments[:1] == ['--'] else ros_arguments,
                    should_exit=lambda: process.poll() is not None)
        finally:
            # 控制台先完成急停，再让 session 与任务正常收尾
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=30.0)
        if process.returncode:
            raise SystemExit(f'任务启动失败，详情见 {log.name}')


if __name__ == '__main__':
    main()

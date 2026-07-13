"""独立的命令行读取工具（应用层示例）

用于连线 / 通信 / 传感器本身的快速验证，无需搭建完整上位机程序

运行：
    python -m kwr57_sensor.cli --interface slcan --channel COM5
    python -m kwr57_sensor.cli --interface socketcan --channel can0 --rate-hz 500
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from .driver import KWR57Sensor


def _int_auto(value: str) -> int:
    """支持 0x15 或 21 两种写法。"""
    return int(value, 0)


def main() -> int:
    ap = argparse.ArgumentParser(description="KWR57 六轴力/力矩传感器 CAN 读取工具（独立运行）")
    ap.add_argument("--interface", required=True,
                    help="python-can 适配器类型，如 slcan / gs_usb / pcan / socketcan")
    ap.add_argument("--channel", required=True,
                    help="通道，如 COM5 / /dev/ttyACM0 / can0")
    ap.add_argument("--period-ms", type=int, default=1,
                    help="数据上传周期(ms)，默认 1 (~1000Hz，最高)")
    ap.add_argument("--rate-hz", type=int, default=1000,
                    help="传感器内部采样率 (100/200/400/500/600/1000)，默认 1000")
    ap.add_argument("--data-base-id", type=_int_auto, default=0x15,
                    help="传感器数据起始 CAN ID，默认 0x15；如修改过下位机 ID 请同步设置")
    ap.add_argument("--si", action="store_true",
                    help="按 kgf/kgf*m -> N/N*m 换算后显示")
    ap.add_argument("--print-hz", type=float, default=20.0,
                    help="控制台最大刷新率(Hz)，默认 20")
    args = ap.parse_args()

    try:
        sensor = KWR57Sensor.open(interface=args.interface, channel=args.channel,
                                  data_base_id=args.data_base_id)
    except Exception as exc:  # noqa: BLE001
        print(f"错误：无法打开 CAN 总线 ({args.interface}:{args.channel}): {exc}",
              file=sys.stderr)
        return 1

    def _shutdown(*_a):
        sensor.close()
        sys.stdout.write("\n")
        sys.stdout.flush()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    sensor.start_stream(period_ms=args.period_ms, rate_hz=args.rate_hz)

    min_period = 1.0 / args.print_hz if args.print_hz > 0 else 0.0
    last = 0.0
    n = 0
    t0 = time.monotonic()
    print(f"正在读取 {args.interface}:{args.channel}，上传周期 {args.period_ms}ms。"
          f"  data_base_id=0x{args.data_base_id:X}。按 Ctrl-C 停止。\n")

    try:
        while True:
            w = sensor.read_wrench(timeout=0.5)
            if w is None:
                continue
            if args.si:
                w = w.to_si()
            n += 1
            now = time.monotonic()
            if now - last < min_period:
                continue
            last = now
            hz = n / (now - t0) if now > t0 else 0.0
            sys.stdout.write(
                f"\rFx={w.fx:+9.3f} Fy={w.fy:+9.3f} Fz={w.fz:+9.3f}  |  "
                f"Mx={w.mx:+8.4f} My={w.my:+8.4f} Mz={w.mz:+8.4f}  "
                f"[{hz:6.1f} Hz, drop={sensor.dropped_sequences}, "
                f"bad={sensor.malformed_frames}, ignore={sensor.ignored_frames}]")
            sys.stdout.flush()
    except KeyboardInterrupt:
        _shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

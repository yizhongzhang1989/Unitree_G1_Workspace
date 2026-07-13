"""设置 / 复位 KWR57 的 CAN ID（非 ROS 命令行工具）。

同一条 CAN 总线上要接多个同型号 KWR57，必须给每个设备设不同的 CAN ID：
  - 接收(命令)ID：上位机向该设备下发命令用的 CAN ID（出厂 0x10）。
  - 发送(数据)基地址：设备上传数据的起始 CAN ID（出厂 0x15 -> 帧 0x15/0x16/0x17）。
两个未改 ID 的设备会发出完全相同的 CAN ID，无法区分且互相冲突。

⚠️ 重要操作提示
  1. **一次只接一个设备**：出厂设备都用 0x10/0x15，若同时接多个，改 ID 指令会
     同时作用于共享该命令 ID 的所有设备。请逐个连接、逐个设 ID。
  2. 修改会**持久化**。改完务必记录每个设备的新 ID，ROS/上位机需按新 ID 配置。
  3. 部分固件改 ID 后需**重新上电**才生效；本工具带 --verify 会尝试用新 ID 读数验证。

示例
  # 把当前设备（现命令 ID 0x10）改为 接收 0x11 / 发送基址 0x18，并验证
  python examples/set_id.py --interface canalystii --channel 0 \
      --host-id 0x11 --sensor-id 0x18 --verify

  # 第二个设备：接收 0x12 / 发送基址 0x1B
  python examples/set_id.py --interface canalystii --channel 0 \
      --host-id 0x12 --sensor-id 0x1B --verify

  # 恢复出厂 ID（接收 0x10 / 发送 0x15）
  python examples/set_id.py --interface canalystii --channel 0 --factory-reset --verify
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kwr57_sensor import KWR57Sensor  # noqa: E402
from kwr57_sensor import protocol  # noqa: E402


def _int_auto(value: str) -> int:
    """支持 0x11 或 17 两种写法。"""
    return int(value, 0)


def _verify(interface: str, channel: str, cmd_id: int, data_base_id: int,
            attempts: int = 3) -> bool:
    """用给定 ID 打开并尝试读数，确认设备已在新 ID 上工作。"""
    try:
        with KWR57Sensor.open(interface=interface, channel=channel,
                              cmd_id=cmd_id, data_base_id=data_base_id) as sensor:
            sensor.start_stream(period_ms=1, rate_hz=1000)
            for _ in range(attempts):
                w = sensor.read_wrench(timeout=0.8)
                if w is not None:
                    print(f"  验证成功：在 命令ID=0x{cmd_id:X} / 数据基址=0x{data_base_id:X} "
                          f"上读到数据 Fx={w.fx:+.3f} Fy={w.fy:+.3f} Fz={w.fz:+.3f}")
                    return True
    except Exception as exc:  # noqa: BLE001
        print(f"  验证时打开/读取失败：{exc}", file=sys.stderr)
        return False
    print("  验证失败：未在新 ID 上读到数据（可能需要给传感器重新上电后再试）。",
          file=sys.stderr)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(
        description="设置 / 复位 KWR57 传感器 CAN ID（谨慎，改动会持久化）")
    ap.add_argument("--interface", required=True,
                    help="python-can 适配器类型，如 canalystii / slcan / socketcan")
    ap.add_argument("--channel", required=True,
                    help="通道，如 0 / COM5 / can0")
    ap.add_argument("--current-cmd-id", type=_int_auto, default=protocol.CAN_ID_CMD,
                    help="目标设备**当前**的命令(接收)ID，改 ID 指令发往此 ID（默认 0x10）")
    ap.add_argument("--host-id", type=_int_auto,
                    help="新的接收(命令)ID，例如 0x11")
    ap.add_argument("--sensor-id", type=_int_auto,
                    help="新的发送(数据)基地址，例如 0x18（帧为 base/base+1/base+2）")
    ap.add_argument("--factory-reset", action="store_true",
                    help="恢复出厂 ID（接收 0x10 / 发送 0x15）")
    ap.add_argument("--verify", action="store_true",
                    help="改动后用新 ID 打开并读数验证")
    ap.add_argument("-y", "--yes", action="store_true",
                    help="跳过确认提示")
    args = ap.parse_args()

    if not args.factory_reset and (args.host_id is None or args.sensor_id is None):
        ap.error("请给出 --host-id 和 --sensor-id，或使用 --factory-reset")
    if args.factory_reset and (args.host_id is not None or args.sensor_id is not None):
        ap.error("--factory-reset 不能与 --host-id/--sensor-id 同时使用")

    # 冲突检查：新的数据三帧不能与新的命令 ID 相同
    if not args.factory_reset:
        new_ids = protocol.data_ids_from_base(args.sensor_id)
        if args.host_id in new_ids:
            ap.error(f"接收 ID 0x{args.host_id:X} 与数据三帧 "
                     f"{'/'.join(f'0x{c:X}' for c in new_ids)} 冲突，请另选")

    if args.factory_reset:
        print("即将【恢复出厂 ID】：接收 0x10 / 发送 0x15。")
    else:
        print(f"即将修改当前命令 ID 0x{args.current_cmd_id:X} 的设备为："
              f"接收 0x{args.host_id:X} / 发送基址 0x{args.sensor_id:X}"
              f"（数据帧 {'/'.join(f'0x{c:X}' for c in new_ids)}）。")
    print("请确认总线上**只连接了这一个**要修改的设备（出厂设备都用 0x10/0x15）。")

    if not args.yes:
        try:
            if input("继续？输入 y 确认：").strip().lower() not in ("y", "yes"):
                print("已取消。")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n已取消。")
            return 1

    try:
        if args.factory_reset:
            with KWR57Sensor.open(interface=args.interface, channel=args.channel) as s:
                s.factory_reset_id()
            print("已发送恢复出厂 ID 指令。")
            time.sleep(0.2)
            if args.verify:
                _verify(args.interface, args.channel,
                        protocol.CAN_ID_CMD, protocol.CAN_ID_DATA_FX_FY)
        else:
            with KWR57Sensor.open(interface=args.interface, channel=args.channel,
                                  cmd_id=args.current_cmd_id) as s:
                s.modify_id(host_id=args.host_id, sensor_id=args.sensor_id)
            print(f"已发送修改 ID 指令：接收 0x{args.host_id:X} / 发送基址 0x{args.sensor_id:X}。")
            time.sleep(0.2)
            if args.verify:
                _verify(args.interface, args.channel, args.host_id, args.sensor_id)
    except Exception as exc:  # noqa: BLE001
        print(f"错误：{exc}", file=sys.stderr)
        return 1

    print("完成。若验证失败，请给传感器重新上电后用新 ID 再验证。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

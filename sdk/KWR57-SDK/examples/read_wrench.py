"""最小示例：打开传感器、读取并打印若干组六轴数据。

使用前请根据自己的 USB-CAN 适配器修改 INTERFACE / CHANNEL。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kwr57_sensor import KWR57Sensor

# 根据实际适配器修改，例如：
#   CANalyst-II:           INTERFACE="canalystii", CHANNEL="0" 或 "1"
#   CANable/CANtact(slcan):  INTERFACE="slcan",     CHANNEL="COM5" 或 "/dev/ttyACM0"
#   创芯/候捷(gs_usb):        INTERFACE="gs_usb",    CHANNEL="0"
#   PEAK PCAN-USB:           INTERFACE="pcan",      CHANNEL="PCAN_USBBUS1"
#   Linux SocketCAN:         INTERFACE="socketcan", CHANNEL="can0"
INTERFACE = "canalystii"
CHANNEL = "0"   # CANalyst-II 有两个CAN通道，一个是0一个是1


def main() -> None:
    # with 语句保证退出时自动停止上传并关闭总线
    with KWR57Sensor.open(interface=INTERFACE, channel=CHANNEL) as sensor:
        # 1ms 周期上传（约 1000Hz）+ 1000Hz 内部采样，最高读取频率
        sensor.start_stream(period_ms=1, rate_hz=1000)

        for _ in range(200):
            w = sensor.read_wrench(timeout=0.5)
            if w is None:
                print("超时：未收到完整数据帧，请检查接线 / 比特率 / CAN ID "
                      f"(drop={sensor.dropped_sequences}, ignore={sensor.ignored_frames})")
                continue
            print(f"Fx={w.fx:+8.3f} Fy={w.fy:+8.3f} Fz={w.fz:+8.3f} | "
                  f"Mx={w.mx:+7.4f} My={w.my:+7.4f} Mz={w.mz:+7.4f}")


if __name__ == "__main__":
    main()

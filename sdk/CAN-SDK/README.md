# CAN-SDK
与 ROS 和具体设备协议无关的 CAN 基础库，统一封装：

- `python-can` 总线创建；
- CANalyst-II 的跨平台 `libusb` backend 配置；
- Linux USB 权限预检查；
- 面向单消费者设备 SDK 的 `CanTransport` 单帧收发与生命周期。

本包刻意**不提供多订阅或帧广播**。`recv()` 会消费底层队列中的下一帧；需要多设备或多进程共享物理 CAN 时，应让 `can_bridge_ros` 等上层总线管理器成为唯一接收者并分发帧。

本目录是普通 Python 包，位于工作区根目录的 `sdk/`，不在 ROS/colcon 默认扫描的 `src/` 下。
工作区内的 ROS2 节点通过 [`scripts/env.sh`](../../scripts/env.sh) 设置的 `PYTHONPATH` 直接导入源码；安装仅用于仓库外调用、虚拟环境或发布场景。

## 安装
安装是可选的。仅在工作区外使用时执行：

普通 `python-can` 后端：
```bash
python -m pip install -e ./sdk/CAN-SDK
```

CANalyst-II：
```bash
python -m pip install -e './sdk/CAN-SDK[canalystii]'
```

`python-can` 会自动支持大多数 USB-CAN 模块。个别适配器需额外驱动/后端：

| 适配器 | interface | channel 示例 | 备注 |
|---|---|---|---|
| CANable / CANtact (SLCAN) | `slcan` | `COM5`（Win）/`/dev/ttyACM0` | 跨平台，最常见 |
| CANalyst-II / CAN 分析仪 | `canalystii` | `0` 或 `1` | 不显示为 COM 口；需厂商驱动 + libusb 后端 |
| 创芯/候捷 (gs_usb) | `gs_usb` | `0` | 需安装 `libusb`/`pyusb` |
| PEAK PCAN-USB | `pcan` | `PCAN_USBBUS1` | 需 PEAK 驱动 |
| Kvaser | `kvaser` | `0` | 需 Kvaser 驱动 |
| Linux SocketCAN | `socketcan` | `can0` | 先 `sudo ip link set can0 up type can bitrate 1000000` |

## CANalyst-II 配置

CANalyst-II 通常不是串口设备，插入后不会显示为 COM 口。本库通过 `python-can`
的 [`canalystii`](https://pypi.org/project/canalystii/) 后端访问它。

### Windows
#### 安装驱动并确认设备(疑似不需要)
先安装 CANalyst-II 厂商提供的 Windows 驱动。设备管理器中可能显示为：

- `WinUSB Device`
- `USB-CAN`
- `CANalyst-II`
- 厂商自定义 USB 设备

可用 PowerShell 检查设备：

```powershell
Get-PnpDevice -PresentOnly |
    Where-Object { $_.InstanceId -like 'USB*' -or $_.FriendlyName -match 'CAN|WinUSB|USB-CAN|CANalyst' } |
    Select-Object Class, FriendlyName, Status, InstanceId
```

看到 `VID_04D8&PID_0053` 或类似 CANalyst-II/WinUSB 设备且状态为 `OK`，通常表示
操作系统已识别适配器。没有 COM 口是正常现象，不要将其配置为 `slcan`。

#### 安装 Python 后端
建议在虚拟环境中安装 CANalyst-II 可选依赖：

```powershell
python -m pip install -e ".\sdk\CAN-SDK[canalystii]"
```

该 extra 包含：

- `python-can`：统一 CAN API；
- `canalystii`：CANalyst-II 的 python-can 后端；
- `libusb-package`：在 Windows 上提供 `libusb-1.0.dll`。

CANalyst-II 有两路通道：

```python
INTERFACE = "canalystii"
CHANNEL = "0"  # CAN1；CAN2 使用 "1"
```

#### Windows 常见错误

| 报错或现象 | 处理方法 |
|---|---|
| `No module named 'canalystii'` | 安装 `CAN-SDK[canalystii]`，或单独安装 `canalystii` |
| `usb.core.NoBackendError: No backend available` | 安装 `libusb-package`；`can_sdk` 会为后端显式选择随包 DLL |
| 设备管理器没有 COM 口 | 正常；CANalyst-II 不是 SLCAN 串口设备 |
| 打开通道失败 | 检查厂商驱动和 USB 占用；接在 CAN2 时将通道改为 `"1"` |
| `Resource busy` | 确认没有其他进程或厂商工具占用同一个 USB 设备 |

### Linux
#### 安装依赖
```bash
sudo apt update
sudo apt install -y python3-pip libusb-1.0-0
python3 -m pip install -e './sdk/CAN-SDK[canalystii]'
```

#### 配置 USB 权限
CANalyst-II 常见 USB ID 为 `04d8:0053`。创建 udev 规则：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="0053", MODE="0666", GROUP="plugdev"' \
    | sudo tee /etc/udev/rules.d/99-canalystii.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

确认当前用户属于 `plugdev`：
```bash
id -nG | grep -w plugdev || echo 'not in plugdev'
```

若不在组内：
```bash
sudo usermod -aG plugdev "$USER"
```

重新登录并拔插设备，然后检查系统识别状态：
```bash
lsusb | grep -i -E '04d8|can|canalyst|chuangxin'
```

#### Linux 常见错误
- `Unable to load libusb backend`：确认已安装 `libusb-1.0-0` 与 `libusb-package`。
- `usb.core.USBError: [Errno 13] Access denied`：udev 规则或用户组尚未生效，重新登录并拔插设备。
- `Resource busy`：关闭其他直接打开该 USB 设备的进程；双通道应由同一个 Bus 使用 `channel="0,1"` 打开。

## 使用
```python
from can_sdk import CanTransport, open_bus

bus = open_bus("socketcan", "can0", 1_000_000)
# 调用方拥有 bus，使用完成后调用 bus.shutdown()。

with CanTransport("slcan", "COM5", 1_000_000) as transport:
    transport.send(0x10, b"\x8a\x00\x01")
    frame = transport.recv(timeout=0.1)
```

## 通用故障排查

| 现象 | 处理 |
|---|---|
| 打开总线失败 | 检查 `interface`、`channel`、比特率、适配器驱动和对应 python-can 后端 |
| 收不到任何帧 | 检查设备供电、CAN_H/CAN_L、两端终端电阻、比特率及设备是否已开始发送 |
| 帧间歇丢失 | 检查总线负载、终端匹配、USB 稳定性和接收循环是否及时消费队列 |
| 多个程序读到不同帧 | `recv()` 是单消费者语义；应由一个总线管理器独占接收并向上层分发 |

## 分层约束
- `can_sdk`：只负责适配器环境、总线打开和基础 I/O，不依赖 ROS、KWR57 或其他设备协议。
- `KWR57-SDK`：依赖本包完成直连模式 I/O，只保留 KWR57 协议和设备语义。
- `can_bridge_ros`：依赖本包打开物理总线，负责 ROS 消息转换和共享分发。

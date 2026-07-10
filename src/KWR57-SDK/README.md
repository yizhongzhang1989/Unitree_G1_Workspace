# KWR57 六轴力/力矩传感器 · CAN 通信 Python 驱动

坤维科技 **KWR57 系列**（应变式六轴力/力矩传感器，CAN 通信）的纯 Python 调用库。
通过一个 **USB 转 CAN 模块** 把 PC 与传感器相连，即可读取 `Fx/Fy/Fz/Mx/My/Mz`
六轴数据，并下发采样率、数据流、ID 修改等指令。

代码按 **协议 → 传输 → 驱动 → 应用** 四层组织，各层职责单一、可独立测试与替换。

## 1. 硬件连接

传感器为 4 线 + 屏蔽的多芯线缆（接线以实物标签为准，见手册 4.4 节）：

| 序号 | 芯线颜色 | 定义 | 接到 USB-CAN 模块 |
|---|---|---|---|
| 1 | 红 | 电源 + | 外部 9~24 VDC 电源正 |
| 2 | 黑 | 电源 − | 外部电源负 / 与模块共地 |
| 3 | 绿 | CAN H | CAN_H |
| 4 | 白 | CAN L | CAN_L |
| 5 | 屏蔽 | 屏蔽 | 机壳地 / 模块 GND |

> ⚠️ **电路板无反极性保护**：上电前务必确认电源正负极正确，反接会烧毁电路（手册第 3 节）。
> 传感器供电 **9~24 VDC**（KWR57A 为 12~48 VDC），由外部电源提供，通常不由 USB-CAN 模块供电。
> CAN 总线两端需要 **120 Ω 终端电阻**；多数 USB-CAN 模块可跳线/开关启用内置终端电阻。

连接拓扑：

```txt
┌──────────┐   USB    ┌──────────────┐   CAN_H/CAN_L  ┌──────────────┐   红/黑
│   PC     │────────▶│  USB-CAN 模块 │◀────────────▶│ KWR57 传感器 │ ◀────── 9~24VDC 电源
└──────────┘          └──────────────┘                └──────────────┘
```

## 2. 通信协议（手册 4.3 节）

**CAN 比特率固定 1 Mbps，标准帧（11 位 ID）。**

### 2.1 数据输出（传感器 → 上位机）

六个通道为 **IEEE754 单精度浮点**，每通道 4 字节，一个采样点分 **3 帧** 发送，用 CAN ID 区分：

| CAN ID | data[0:4] | data[4:8] |
|---|---|---|
| `0x15` | Fx | Fy |
| `0x16` | Fz | Mx |
| `0x17` | My | Mz |

> 实测 KWR57 数据帧中的 IEEE754 浮点采用 **小端** 字节序。

### 2.2 指令（上位机 → 传感器，默认发往 CAN ID `0x10`）

| 功能 | data 字节 | 说明 |
|---|---|---|
| 开始/连续上传 | `0x8A HH LL` | `HHLL` = 上传周期(ms)，高字节在前；例 `0x8A 00 10` → 16ms |
| 停止上传 | `0x8A 00 00` | 周期为 0 即停止 |
| 设置采样率 | `0x60 NN` | `NN`：01=100 02=200 03=400 04=500 05=600 06=1000 Hz（默认 500Hz） |
| 修改 ID | `0xDE AA 主hi 主lo 从hi 从lo 0D 0A` | 主=上位机(接收)ID，从=下位机(发送)ID |
| 恢复出厂 ID | `0xDE DE DE 0D 0A` | 出厂 接收 `0x10` / 发送 `0x15`；需在 CAN ID `0x000` 下发送 |

> **单位说明**：基本规格表标注“默认数据输出单位 kg, kgm”。本库默认原样输出传感器的浮点值；如需 N / N·m 需要换算（1 kgf ≈ 9.80665 N）。


## 3. 代码结构与分层

```
KWR57-SDK/                 ← 纯 Python SDK（非 ROS 包；位于 end_effector_ros 工作区）
├── kwr57_sensor/           import 名为 kwr57_sensor 的模块
│   ├── protocol.py    协议层：常量 / 指令构造 / 数据帧解码 / 三帧组装（纯逻辑，无 I/O）
│   ├── transport.py   传输层：封装 python-can，屏蔽不同 USB-CAN 适配器差异
│   ├── driver.py      驱动层：KWR57Sensor 高层 API（组合协议层 + 传输层）
│   ├── cli.py         应用层：命令行实时读取工具
│   └── __init__.py
├── examples/
│   ├── read_wrench.py  最小调用示例
│   ├── set_id.py       设置/复位设备 CAN ID（同总线挂多个设备前置步骤）
│   └── web_wrench.py   六轴 Web 可视化示例
├── setup.py / pyproject.toml   让 kwr57_sensor 可 pip 安装（供 ROS 节点导入）
├── requirements.txt
└── README.md
```

> **ROS 2 用户**：本 SDK 是纯 Python 库，可 `pip install -e .` 单独使用（非 ROS）。
> ROS 2 封装在同一工作区的 **`kwr57_ros`** 包（bridge 架构）：通用 `can_bridge` 独占总线并以
> `can_msgs/Frame` 收发，KWR57 只是一个**设备节点**（订阅总线帧、过滤自己的 CAN ID、
> 发 `geometry_msgs/WrenchStamped`）。安装/运行/多设备/demo 见 `../kwr57_ros/README.md` 与顶层 README。
> 注意：ROS 2 节点跑在 foxy 的系统 `python3`(3.8) 上，而非 conda `robot`。

- **协议层 `protocol.py`**：只做“字节 ↔ 语义”转换，不碰硬件，可脱离设备做单元测试。
  核心是 `WrenchAssembler`——把 `0x15/0x16/0x17` 三帧缓存并集齐后组装成一个 `Wrench`。
- **传输层 `transport.py`**：基于 [python-can](https://python-can.readthedocs.io)，
  只暴露 `send / recv / close`。更换适配器只需改 `interface/channel`，上层不动。
- **驱动层 `driver.py`**：`KWR57Sensor` 提供 `start_stream / stop_stream /
  set_sample_rate / read_wrench / modify_id / factory_reset_id`，并支持 `with` 自动关闭。
- **应用层 `cli.py` / `examples/`**：面向使用者的入口，演示如何调用驱动层；
  `web_wrench.py` 启动一个本地 HTTP 服务，在浏览器中显示 Fx/Fy/Fz/Mx/My/Mz
  六个条形以及合力/合力矩箭头，无需本地图形环境，适合 SSH 远程使用。

数据流：

```
CAN 帧 ──recv──▶ transport ──(id,data)──▶ WrenchAssembler ──集齐3帧──▶ Wrench ──▶ 应用
指令   ◀─send─── transport ◀──bytes────── protocol.build_*()  ◀─────── 驱动方法
```


## 4. 安装

```powershell
cd kwr57_can_sensor
pip install -r requirements.txt
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


## 5. CANalyst-II 从零配置（Windows）

CANalyst-II 这类“CAN 分析仪”通常不是串口设备，插上后不会出现在
“端口 (COM 和 LPT)”里，也不会有 `COM5` 这类端口号。本库通过
`python-can` 的 `canalystii` 后端访问它。

### 5.1 安装厂商驱动

先安装 CANalyst-II 自带的 Windows 驱动。安装后在设备管理器里它可能显示为：

- `WinUSB Device`
- `USB-CAN`
- `CANalyst-II`
- 厂商自定义 USB 设备

它不显示为 COM 口是正常现象。可用 PowerShell 查看当前 USB 设备：

```powershell
Get-PnpDevice -PresentOnly |
  Where-Object { $_.InstanceId -like 'USB*' -or $_.FriendlyName -match 'CAN|WinUSB|USB-CAN|CANalyst' } |
  Select-Object Class, FriendlyName, Status, InstanceId
```

若看到类似 `VID_04D8&PID_0053`、`WinUSB Device`、状态为 `OK` 的设备，
通常说明 CANalyst-II 已被 Windows 识别。

### 5.2 安装依赖

建议用 `.venv`：

```powershell
cd kwr57_can_sensor
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

CANalyst-II 需要这些 Python 依赖：

```text
python-can
canalystii
libusb-package
```

其中 `canalystii` 是 `python-can` 的 CANalyst-II 底层驱动包，
`libusb-package` 用来在 Windows 上提供 `libusb-1.0.dll`。

### 5.3 配置 interface / channel

CANalyst-II 有两路 CAN 通道：

```python
INTERFACE = "canalystii"
CHANNEL = "0"   # CAN1；如果接 CAN2，改为 "1"
```

运行示例：

```powershell
.\.venv\Scripts\python.exe .\examples\read_wrench.py
```

正常输出类似：

```text
Fx=  -0.470 Fy=  -0.230 Fz=  -0.085 | Mx=-0.0070 My=+0.0073 Mz=-0.0005
Fx=  -0.500 Fy=  -0.297 Fz=  -0.088 | Mx=-0.0077 My=+0.0078 Mz=+0.0001
```

### 5.4 常见 CANalyst-II 错误

| 报错 / 现象 | 处理方法 |
|---|---|
| `Cannot import module can.interfaces.canalystii ... No module named 'canalystii'` | 没装底层包，执行 `.\.venv\Scripts\python.exe -m pip install canalystii` |
| `usb.core.NoBackendError: No backend available` | PyUSB 找不到 libusb 后端，执行 `.\.venv\Scripts\python.exe -m pip install libusb-package`；本库已在 `CanTransport` 中为 `canalystii` 显式加载该 DLL |
| 设备管理器没有 COM 口 | 正常；CANalyst-II 不是 `slcan` 串口设备，不要使用 `INTERFACE="slcan"` / `CHANNEL="COM5"` |
| 一直输出“超时：未收到完整数据帧” | 检查 CAN_H/CAN_L、传感器供电、1Mbps 比特率、120Ω 终端电阻、是否接对 CAN1/CAN2 |
| 打开通道失败 | 如果接在 CAN2，把 `CHANNEL` 从 `"0"` 改成 `"1"` |

### 5.5 Linux 上使用 CANalyst-II


#### 1：安装系统依赖

```bash
sudo apt update
sudo apt install -y python3-pip libusb-1.0-0
```

#### 2：安装 Python 依赖

如果你用 conda/micromamba（例如 `robot`）：

```bash
source ~/.bashrc
conda activate robot
python -m pip install -r requirements.txt
```

如果你用 venv：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

#### 3：配置系统权限（必做）

CANalyst-II 常见 USB ID 是 `04d8:0053`。创建 udev 规则：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="0053", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-canalystii.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

确保当前用户在 `plugdev` 组：

```bash
id -nG | grep -w plugdev || echo 'not in plugdev'
```

若不在组内：

```bash
sudo usermod -aG plugdev $USER
```

然后重新登录，并拔插一次设备。

#### 4：验证

先看系统是否识别设备：

```bash
lsusb | grep -i -E '04d8|can|canalyst|chuangxin'
```

再跑示例：

```bash
python examples/read_wrench.py
```

Linux 参数与 Windows 一致：

```python
INTERFACE = "canalystii"
CHANNEL = "0"   # CAN1；如果接 CAN2，改为 "1"
```

#### 常见报错

- `RuntimeError: Unable to load libusb backend ... dll`：请更新到当前版本代码（已按系统自动选择 `.so/.dylib/.dll`）。
- `usb.core.USBError: [Errno 13] Access denied`：通常是 udev/用户组权限未生效，重做步骤 3 并重新登录。


## 6. 使用

### 6.1 命令行快速验证

```powershell
# CANalyst-II + Windows
python -m kwr57_sensor.cli --interface canalystii --channel 0

# CANable(slcan) + Windows COM5
python -m kwr57_sensor.cli --interface slcan --channel COM5

# 先把内部采样率设为 500Hz，再以 16ms 周期上传
python -m kwr57_sensor.cli --interface slcan --channel COM5 --rate-hz 500 --period-ms 16

# Linux SocketCAN
python -m kwr57_sensor.cli --interface socketcan --channel can0
```

输出示例：

```
Fx=  +0.123 Fy=  -0.045 Fz=  +2.310  |  Mx=+0.0012 My=-0.0034 Mz=+0.0007  [  20.0 Hz]
```

### 6.2 Web 可视化（浏览器查看，适合 SSH）

`examples/web_wrench.py` 会启动一个本地 HTTP 服务，在浏览器中实时显示六个轴的数值条形图，
并用箭头显示合力与合力矩的 XY 投影，左上角圆点显示 Z 轴分量大小与方向。
它只依赖 Python 标准库，不需要本地图形环境（Tkinter/X11），因此适合 SSH 远程使用。

```bash
# CANalyst-II
python examples/web_wrench.py --interface canalystii --channel 0

# CANable(slcan) + COM5
python examples/web_wrench.py --interface slcan --channel COM5

# 不连接硬件，预览界面
python examples/web_wrench.py --demo
```

启动后在浏览器打开（默认绑定 `127.0.0.1:8765`）：

```text
http://127.0.0.1:8765
```

通过 SSH 远程时，在本地机器做端口转发即可在本地浏览器查看：

```bash
ssh -L 8765:127.0.0.1:8765 user@server
```

如需在局域网内其它机器直接访问，可绑定到所有网卡（注意安全）：

```bash
python examples/web_wrench.py --demo --host 0.0.0.0 --port 8765
```

如果条形或箭头过早顶满，可按实际量程调整显示比例：

```bash
python examples/web_wrench.py --interface canalystii --channel 0 --force-scale 50 --torque-scale 2
```

### 6.3 在代码中调用

```python
from kwr57_sensor import KWR57Sensor

# 打开总线（按你的适配器修改 interface/channel）
with KWR57Sensor.open(interface="canalystii", channel="0") as sensor:
    sensor.start_stream(period_ms=1, rate_hz=1000)  # 1ms 周期 + 1000Hz 采样，最高频率
    for _ in range(200):
        w = sensor.read_wrench(timeout=0.5)   # 集齐三帧才返回
        if w:
            print(w.fx, w.fy, w.fz, w.mx, w.my, w.mz)
    # 退出 with 时自动 stop_stream + close
```

也可复用已有的 python-can 总线，自行构造传输层：

```python
from kwr57_sensor import KWR57Sensor, CanTransport

transport = CanTransport(interface="gs_usb", channel="0")
sensor = KWR57Sensor(transport)
sensor.start_stream(period_ms=1, rate_hz=1000)
...
sensor.close()
```

### 6.4 修改 / 恢复 CAN ID（谨慎）

```python
sensor.modify_id(host_id=0x20, sensor_id=0x25)  # 会持久化，改后需同步上位机配置
sensor.factory_reset_id()                        # 恢复出厂 0x10 / 0x15
```


## 7. 常见问题排查

| 现象 | 可能原因 |
|---|---|
| `read_wrench` 一直超时返回 None | 比特率不是 1Mbps；CAN_H/CAN_L 接反；缺终端电阻；传感器未上电或未发 `start_stream` |
| 能收到帧但数值巨大/NaN | 检查是否收到完整 0x15/0x16/0x17 三帧；若原始字节异常，确认传感器型号、CAN ID 和固件配置 |
| 只收到部分轴 | 只收到 3 帧中的一部分，检查总线负载/丢帧；确认 CAN ID 为 0x15/0x16/0x17 |
| 打开总线报错 | `interface/channel` 与实际适配器不符，或缺少对应后端驱动 |

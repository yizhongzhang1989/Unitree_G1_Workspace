# VR 头显遥操作 —— 启动流程
WebXR 桥接：头显浏览器读手柄/头部位姿 → WebSocket 直推给 `vr_teleop` 节点 → ROS 话题。
不需要 Unity，不需要装 APK，**也不再有独立的桥接进程**——采集页由 ROS 节点自己托管
（`g1_motion_control/vr_teleop.py` 内嵌 aiohttp）。

`vr_teleop` 是上层控制源，不做 IK：它负责 A/X 航向标定、squeeze 离合、坐标映射、
tracking 重定位抑制，最后以长度 20 发布到统一 `/motion_control/command`；和 VLA / 键盘
遵守同一个 2/4/7/14/20 分块契约，不另建 VR 专用命令话题。它只反向读取
`/motion_control/status.limited_pose`，用于离合接合时选一个可达、无编码器静差的锚点。

WebXR 通信正常也可能失去光学位置。采集页把 `emulatedPosition` 原样送到节点：一旦为真，
节点冻结目标但保留该侧离合。恢复时**按与上一帧真实跟踪位置的实际位移**判定：小于 0.1 m
就照常积分，超过才无跳变重锚。**不能一律重锚**——那样 `emulatedPosition` 逐帧抖时会按
比例吞掉手部运动，实测 1/2 抖动下手走 900 mm 而目标走 0 mm（完全冻住）。
squeeze 使用 0.5/0.4 的接合/释放迟滞，避免模拟量在阈值附近抖动。

末端命令还会被夹在 `limited_pose` 周围 `arm_lead_limit`(0.02 m) 的球里：够不着时目标
不再无界累积，所以手往回缩马上见动静，不必先空推一大段。别调到 0.03 以上：那会把
残差顶过策略层的 `ik_rescue_err`，逃生种子一直开着反而更跳。

本目录里只剩下这些：`index.html`（头显里打开的采集页）、`monitor.html`（监控面板）——
两个页随包安装到 `share/g1_motion_control/vr/`；加上 `adb_reverse_watch.sh` 守护
脚本和这份说明（这两个不安装，只在源码树里用）。签证书的脚本是个 ROS 命令：
`ros2 run g1_motion_control make_vr_cert`。

**以下命令全部在 devcontainer 内执行**。
容器是 `network_mode: host`，容器内的 `localhost` 就是宿主的 `localhost`，
所以 `adb reverse` 建在容器里等价于建在宿主上——不要在宿主再起一个 adb server，
两边共用 5037 端口会互相抢设备。

当前设备：

| | 地址 |
|---|---|
| 头显（PICO，model A9210） | `192.168.137.82:5555` |
| 机器人（本机在头显网段，wlan0） | `192.168.137.149` |

---

## 两条接入路径，**同时**开着

WebXR 只在**安全上下文**里可用（HTTPS 或 `localhost`），所以只有这两种进法。
节点同时监听两个端口，两条路服务的是同一份采集页，随便用哪条、也可以互为备份：

| | 地址 | 依赖 | 适合 |
|---|---|---|---|
| **A. HTTPS 直连**（推荐） | `https://192.168.137.149:8443` | 自签证书 | 长时间使用；不怕头显休眠 |
| **B. adb reverse** | `http://localhost:8000` | adb 连着 | 临时调试；懒得点证书警告 |

为什么不能只留一条：`adb reverse` 的转发规则会因为头显休眠、USB 重枚举、无线超时
而**静默失效**，服务端完全无感知，表现就是“网页打不开”；而 HTTPS 那条每次打开都要点
一下证书警告。两条都开着，哪条断了立刻能换。

---

## 0. 依赖

`adb`、`aiohttp` 已经写进 [.devcontainer/Dockerfile](../../../.devcontainer/Dockerfile)，
正常重建镜像后即可用（`aiohttp` 同时也在 `package.xml` 里声明为
`python3-aiohttp`，`rosdep install` 能装）。签证书还要 `python3-cryptography`。
若在旧镜像里发现缺失，临时补装：

```bash
sudo apt-get update && sudo apt-get install -y adb python3-aiohttp python3-cryptography
chmod +x adb_reverse_watch.sh
```

---

## A. HTTPS 直连（不需要 adb）

### A1. 签一张自签名证书（只需一次，有效期 825 天）

```bash
ros2 run g1_motion_control make_vr_cert
```

它会自动把**本机所有网卡的 IPv4** 写进证书 SAN，输出到 `~/.ros/g1_vr/`（私钥 0600），
正是 `vr_teleop.launch.py` 默认去找的位置——签完直接起节点就有 HTTPS，不用传参数。

换了网段、或者命令输出里没看到头显能访问的那个 IP，就手动指定后重签：

```bash
ip -4 -o addr show                                    # 查本机在头显网段的地址
ros2 run g1_motion_control make_vr_cert 192.168.137.149
```

> 证书里**必须包含头显实际输入的那个 IP**。不包含的话浏览器报的是
> `CERT_COMMON_NAME_INVALID`，和普通的“不受信任”不是一回事。

### A2. 起节点，头显里直接打开

```bash
ros2 launch g1_motion_control vr_teleop.launch.py
```

日志里会同时出现两行，两个口都开着：

```
WebXR 明文口已就绪：http://0.0.0.0:8000（配 adb reverse tcp:8000 tcp:8000 用）
WebXR HTTPS 口已就绪：https://<本机局域网IP>:8443（自签证书，头显里点「高级 → 继续前往」）
```

头显浏览器里输入 **`https://192.168.137.149:8443`**：

1. 跳“您的连接不是私密连接” → **高级（Advanced）** → **继续前往（Proceed）**。
   点过之后页面仍然是 `isSecureContext`，WebXR 可用。
2. 点 **Enter VR**。节点日志打 `头显已连接`。
3. 存个书签，下次直接开。

> Quest 浏览器每次**重启后**都要重新点一次“继续前往”。嫌烦就把 `cert.pem` 传进
> 头显装成用户 CA（设置 → 安全 → 从存储设备安装），之后就没警告了。

---

## B. adb reverse（备用）

### B1. 连头显

头显和机器人必须在**同一局域网**。先确认可达：

```bash
ping -c 2 192.168.137.82
```

无线连接：

```bash
adb connect 192.168.137.82:5555
adb devices          # 应看到 192.168.137.82:5555  device
```

连不上说明头显没开 tcpip 模式。**用 USB 把头显插到机器人一次**：

```bash
adb devices          # 先确认 USB 下能看到设备
adb tcpip 5555
adb shell ip route   # 记下 src 后面的 IP
```

拔线后重新 `adb connect`。

> `adb tcpip 5555` 头显重启后会失效。想彻底免掉插线：
> 头显 开发者选项 → 无线调试 → 用配对码 `adb pair <ip>:<配对端口>`，这个是持久的。

### B2. 建立端口转发

`adb reverse` 让头显的 `localhost:8000` 指向机器人的 8000 端口，这样就能用 http：

```bash
adb -s 192.168.137.82:5555 reverse tcp:8000 tcp:8000
adb -s 192.168.137.82:5555 reverse --list    # 应输出 host-N tcp:8000 tcp:8000

# reverse 守护，断连自动重建（走这条路就必开，见下方"已知坑"）
cd /workspace/src/g1_motion_control/vr && ./adb_reverse_watch.sh 192.168.137.82:5555
```

### B3. 在头显里拉起采集页

```bash
adb -s 192.168.137.82:5555 shell am start -a android.intent.action.VIEW -d "http://localhost:8000"
```

戴上头显，点页面里的 **Enter VR**。节点日志会打一行 `头显已连接`。

---

## 验证数据在流

```bash
curl -s http://localhost:8000/state              # 明文口
curl -sk https://192.168.137.149:8443/state      # TLS 口（-k 跳过自签证书校验）
```

`seq` 持续上涨、`session_active` 为 `true` 就对了。两个口拿到的是同一份数据。

> 服务默认绑 `0.0.0.0`，而它能触发 `engage`/`start`/`estop`。只走 adb 那条、
> 不开监控页时，加 `bind_host:=127.0.0.1` 把它收到环回口最安全（`adb reverse`
> 转的就是环回口，功能不受影响）。走 HTTPS 就必须听局域网，那就配
> `token:=<共享密钥>`，此后所有接口都要带 `?token=`。

---

## [节点](../g1_motion_control/vr_teleop.py)提供的接口

两个端口上都是这一套（`:8000` 明文、`:8443` TLS）：

| 方法 | 路径 | 用途 |
|---|---|---|
| GET | `/` | 头显里打开的采集页 |
| GET | `/monitor` | 监控面板（在有屏幕的机器上开） |
| GET | `/state` | 最新一帧，`curl` 自检用 |
| WS | `/ws/device` | 头显上行（采集页自己连，不用管） |
| WS | `/ws/subscribe` | 监控页下行 |
| POST | `/haptic` | 让手柄震动 |

监控页在笔记本上开 <http://localhost:8000/monitor> 即可。
它只是个订阅端，不需要 localhost 特权，机器人无头也没关系。采集页的 WS 会跟着
页面协议走（http → `ws`、https → `wss`），两条路径都不用改任何东西。

手柄到机器人的映射（摇杆、B/Y 状态机、squeeze 离合、trigger 夹爪）见 [../README.md](../README.md#vr-头显遥操可选代替键盘) 与 `vr_teleop.py` 的模块文档。

---

## 排查清单

两条路径先分清楚是哪一条坏了：

```bash
# 服务自身（两个口都应该 200）
curl -s  -o /dev/null -w '%{http_code}\n' http://localhost:8000/
curl -sk -o /dev/null -w '%{http_code}\n' https://192.168.137.149:8443/

# 证书里到底签了哪些地址（头显输的那个 IP 必须在里面）
openssl x509 -in ~/.ros/g1_vr/cert.pem -noout -text | grep -A1 'Subject Alternative Name'

# adb 那条
adb devices                                          # 设备在不在
adb -s 192.168.137.82:5555 reverse --list            # 转发规则在不在（最常见）
adb -s 192.168.137.82:5555 shell "curl -s -o /dev/null -w '%{http_code}' http://localhost:8000/"
```

| 现象 | 原因 |
|---|---|
| `seq` 不涨但 HTTP 200 | 页面连上了但没进 VR，戴上头显点 Enter VR |
| 头显报 `CERT_COMMON_NAME_INVALID` | 证书 SAN 里没有你输的那个 IP，`make_vr_cert <IP>` 重签 |
| 页面能开但 **Enter VR 按钮点不动** | 不是安全上下文。地址栏必须是 `https://` 或 `http://localhost`，`http://192.168.x.x` 不行 |
| 日志只有明文口那一行 | 证书不在。上一行会写「证书文件不在」，跑一次 `make_vr_cert` |
| `HTTPS 口 8443 起不来` | 端口被占。明文口不受影响，可以先用 adb 那条顶着 |

## 两个已知坑

**`adb reverse` 规则会掉。** 头显休眠、USB 重枚举、无线超时都会清空它，
服务端完全无感知，表现就是"打不开网页"。`adb_reverse_watch.sh` 就是防这个的。
**这也正是建议主用 HTTPS 那条的原因**——它根本不经过 adb。

**停服务时别只关终端。** 如果是从容器外用 `docker exec` 起的，
杀掉 `docker exec` 客户端不会把信号传进容器，进程会被 reparent 到 PID 0 继续跑。
在容器内 Ctrl+C 才是干净的；已经跑飞了就按 PID 杀：

```bash
pgrep -af "vr_teleop|adb_reverse_watch"
kill <PID>
```

残留的 `vr_teleop` 会一直占着 8000/8443，下一轮启动时节点日志会打
`WebXR 桥起不来`。用 `ss -lptn 'sport = :8000 or sport = :8443'` 能看到是谁占的。

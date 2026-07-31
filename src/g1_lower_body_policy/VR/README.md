# VR 头显遥操作 —— 启动流程

WebXR 桥接：头显浏览器读手柄/头部位姿 → WebSocket 推给 `server.py` → 机器人端消费。
不需要 Unity，不需要装 APK。

**以下命令全部在 devcontainer 内执行**（终端里 `cd /workspace/src/g1_lower_body_policy/VR`）。
容器是 `network_mode: host`，容器内的 `localhost` 就是宿主的 `localhost`，
所以 `adb reverse` 建在容器里等价于建在宿主上——不要在宿主再起一个 adb server，
两边共用 5037 端口会互相抢设备。

当前设备：

| | 地址 |
|---|---|
| 头显（PICO，model A9210） | `192.168.137.82:5555` |
| 机器人（本机在头显网段） | `192.168.137.149` |

---

## 0. 依赖

`adb` 和 `aiohttp` 已经写进 [.devcontainer/Dockerfile](../../../.devcontainer/Dockerfile)，
正常重建镜像后即可用。若在旧镜像里发现缺失，临时补装：

```bash
sudo apt-get update && sudo apt-get install -y adb
pip3 install -r requirements.txt
chmod +x adb_reverse_watch.sh
```

---

## 1. 连头显

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

---

## 2. 建立端口转发

WebXR 要求安全上下文（HTTPS 或 localhost）。`adb reverse` 让头显的 `localhost:8000`
指向机器人的 8000 端口，这样就能用 http 而不用折腾自签证书：

```bash
adb -s 192.168.137.82:5555 reverse tcp:8000 tcp:8000
adb -s 192.168.137.82:5555 reverse --list    # 应输出 host-N tcp:8000 tcp:8000
```

---

## 3. 启动（三个终端）

```bash
# 终端 1 —— 桥接服务，默认 0.0.0.0:8000
python3 server.py

# 终端 2 —— reverse 守护，断连自动重建（必开，见下方"已知坑"）
./adb_reverse_watch.sh 192.168.137.82:5555

# 终端 3 —— 在头显里拉起采集页
adb -s 192.168.137.82:5555 shell am start -a android.intent.action.VIEW -d "http://localhost:8000"
```

戴上头显，点页面里的 **Enter VR**。

验证数据在流：

```bash
curl -s http://localhost:8000/state
```

`seq` 持续上涨、`session_active` 为 `true` 就对了。

---

## 4. 机器人程序取数据

服务端接口：

| 方法 | 路径 | 用途 |
|---|---|---|
| GET | `/` | 头显里打开的采集页 |
| GET | `/monitor` | 监控面板（在有屏幕的机器上开） |
| GET | `/state` | 轮询拉最新一帧 |
| WS | `/ws/subscribe` | 实时订阅每一帧 |
| POST | `/haptic` | 让手柄震动 |

三选一：

```bash
# A. 订阅实时流（推荐）
python3 example_consumer.py --url ws://localhost:8000/ws/subscribe

# B. 轮询
curl http://localhost:8000/state

# C. 服务端定频 POST 到你的控制程序
python3 server.py --forward-url http://localhost:9000/vr --forward-hz 90
```

在 `example_consumer.py` 的 `# TODO` 处接机器人 SDK。
**用增量而不是绝对位姿**——按住扳机锁定参考原点，只下发相对位移。

监控页在笔记本上开 <http://192.168.137.149:8000/monitor> 即可。
它只是个订阅端，不需要 localhost 特权，机器人无头也没关系。

---

## 排查清单

按顺序查：

```bash
adb devices                                          # 设备在不在
adb -s 192.168.137.82:5555 reverse --list            # 转发规则在不在（最常见）
adb -s 192.168.137.82:5555 shell "curl -s -o /dev/null -w '%{http_code}' http://localhost:8000/"
curl -s http://localhost:8000/state                  # seq 有没有在涨
```

`seq` 不涨但 HTTP 200 → 页面连上了但没进 VR，戴上头显点 Enter VR。

## 两个已知坑

**`adb reverse` 规则会掉。** 头显休眠、USB 重枚举、无线超时都会清空它，
服务端完全无感知，表现就是"打不开网页"。终端 2 的守护脚本就是防这个的，别嫌麻烦。

**停服务时别只关终端。** 如果是从容器外用 `docker exec` 起的，
杀掉 `docker exec` 客户端不会把信号传进容器，进程会被 reparent 到 PID 0 继续跑。
在容器内 Ctrl+C 才是干净的；已经跑飞了就按 PID 杀：

```bash
pgrep -af "server.py|adb_reverse_watch"
kill <PID>
```

注意别用 `pkill -f "python3 server.py"`——`-f` 匹配完整命令行，
会把你正在执行 pkill 的那个 shell 自己也匹配掉。

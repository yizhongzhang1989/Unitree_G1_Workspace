# 本机 Jetson / ROS Humble 容器 GPU 配置记录

记录日期：2026-10-26。本次是在宿主实际检查、修改并验证后的记录，不是尚未执行的建议。
运行参数仍以 [docker-compose.yml](docker-compose.yml) 为唯一来源；
没有增加第二套启动配置，也没有修改模型、推理后端、相机或机器人控制逻辑。

2026-10-27 的独立 GPU ONNX Runtime 安装与模型推理实测见第 8 节；前七节保留为原始容器配置记录。

## 1. 实际环境与故障

| 项目 | 本机结果 |
|---|---|
| 硬件 / 架构 | NVIDIA Orin NX Developer Kit / aarch64 |
| 宿主 | Ubuntu 20.04.6，内核 `5.10.104-tegra` |
| L4T | `/etc/nv_tegra_release` 为 R35 revision 3.1，对应 JetPack 5.1.1 |
| CUDA | 宿主已安装 CUDA Toolkit 11.4；驱动 API 实测版本 `11040` |
| 宿主驱动 | `nvidia-l4t-cuda`、`nvidia-l4t-core` 等为 `35.3.1-20230319081403` |
| 容器组件 | `nvidia-container-runtime` 包 `3.9.0-1`，toolkit `1.11.0~rc.1-1`，`libnvidia-container1` `1.10.0-1` |
| Docker | 26.1.3；Compose 2.27.1 |
| 容器系统 | Ubuntu 22.04.5、ROS 2 Humble、Python 3.10.12 |
| 项目宿主路径 | `/home/unitree/Unitree_G1_Workspace`，bind mount 到容器 `/workspace` |

宿主 Docker 已注册 `nvidia` runtime，默认仍为 `runc`。NVIDIA runtime 配置为
`mode = "auto"`；该 Jetson 通过
`/etc/nvidia-container-runtime/host-files-for-container.d/l4t.csv`
注入 Tegra 厂商驱动库。清单包含 `libcuda.so.1.1`、对应符号链接和 `libnvrm_*` 等依赖。

旧容器 `2c7333345ef6` 虽然有 `privileged` 和 `/dev` 挂载，实际 runtime 却是
`runc`，也没有 `NVIDIA_*` 环境变量，因此没有注入驱动用户态库：

```text
OSError: libcuda.so.1: cannot open shared object file: No such file or directory
```

**根因是项目创建容器时没有选择 NVIDIA runtime，不是宿主缺少 CUDA 驱动。**
因此没有安装、升级任何 NVIDIA/CUDA 组件，没有修改宿主配置，没有重启 Docker 或宿主。

## 2. 只读检查与定位方法

以下是本次使用的主要检查入口。容器 ID 是本次记录，今后操作应重新检查，不要假定一直不变。
打印完整 `docker inspect` 或环境变量可能包含本机隐私，不要上传未经检查的输出。

```bash
# 宿主软件栈
cat /etc/os-release
uname -a
cat /etc/nv_tegra_release
dpkg-query -W 'nvidia-l4t-*' 'nvidia-container*' 'libnvidia-container*' 'cuda-*'
/sbin/ldconfig -p | grep 'libcuda.so'

# 已注册 runtime、现有容器
docker info --format '{{range $name, $value := .Runtimes}}{{$name}} {{end}} Default={{.DefaultRuntime}}'
docker ps -a
docker inspect <现有容器ID> --format \
  'Runtime={{.HostConfig.Runtime}} Image={{.Config.Image}} Mounts={{json .Mounts}} Labels={{json .Config.Labels}}'

# 项目改动：保留已有未提交内容
git -C /home/unitree/Unitree_G1_Workspace status --short
```

本次从 Mounts 和 Compose labels 确认：

- 项目名：`unitree_g1_workspace_devcontainer`；服务名：`dev`。
- 基础配置：本仓库 `.devcontainer/docker-compose.yml`。
- 现有历史 override：
  `/home/unitree/.vscode-remote-containers/data/docker-compose/g1-proxy-migration-20261020-088cc4c1547c.yml`。
- 实际镜像：`g1-humble:runtime-http-20261020-088cc4c1547c`，
  不是基础配置中声明的 `g1-humble:latest`。
- 重建前后镜像 ID 都是
  `sha256:09fc0d60885467ac1ce6a7cc2bfc56525431ff7dbedb9519ad4846ebb6b40aee`。

检查时项目已有两个标定配置的未提交修改，以及未跟踪的 `src/upper_insert/`；
均保留，未执行 Git 提交、回退或清理。

## 3. 最小持久化修改

仅在原有 `services.dev` 中合并以下三项，原有代理等环境变量保持不变：

```yaml
services:
  dev:
    runtime: nvidia
    environment:
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: compute,utility
```

这个片段是变更说明，**不是替代整份 Compose 文件的新配置**。
[devcontainer.json](devcontainer.json) 和 [dev.sh](dev.sh)
继续使用原 Compose 文件；[Dockerfile](Dockerfile) 未修改，仍安装 CPU 版 ONNX Runtime。

没有执行以下操作：

- 没有设置全局 `default-runtime: nvidia`。
- 没有覆盖 Docker daemon JSON、代理、其他 runtimes 或实时调度设置。
- 没有安装 x86/桌面显卡驱动、升级 JetPack 或刷机。
- 没有手工复制 `libcuda`、挂载整个宿主 `/usr/lib`、使用 stub 冒充驱动。
- 没有使用 `--gpus all` 替代 Jetson 上的显式 `runtime: nvidia`。

## 4. 数据保护、授权与重建

**本节的重建已经在用户明确授权后执行完成。不要为了验证而再次执行。**
今后任何重新创建都必须重新确认任务允许中断。

重建会终止开发容器中的所有进程、VS Code 远程连接和终端；仅 `restart` 不会应用新的
runtime。检查时曾出现 `colcon` 构建，不能因未看到控制节点就自行推定可以中断。
本次未调用运动、关节、接合、标定等机器人接口，也没有自动重新启动业务任务。

### 数据位置

| 内容 | 本次保护方式 |
|---|---|
| 工作区、源码、`build/`、`install/`、`log/` | 保留原宿主工作区 bind mount |
| 项目中的标定配置 | 重建前后校验 SHA-256，一致；保留用户未提交修改 |
| 用户 `.ros` | 原 `/home/unitree/.ros` bind mount，未删除或复制覆盖 |
| VS Code Server | 原 `/home/unitree/.g1-vscode-server` bind mount |
| `/vscode` | 保留原 `vscode` Docker volume |
| `camera_calib_data`、`motions_dataset`、`motions_experiments` | 位于现用历史镜像；保留同一镜像，未切换到 `latest` |
| 容器可写层 | 重建前创建无暂停的本地 Docker 回滚快照 |
| 最近变动的用户配置 | 另行备份并恢复 Git 配置、shell history、Copilot 状态和 CMakeTools 日志 |

宿主 `.ros/arm_gravity_compensation/` 当时不存在，不能把文档约定的目录当成已存在的
标定备份；实际检查了项目中的标定文件及真实挂载。容器中其他未挂载目录今后仍可能
产生数据，下一次重建应再次检查 `docker diff`，不能只依赖本次快照。

### 本次保留的回滚资源

- 本地镜像：`g1-humble:backup-before-nvidia-2c7333345ef6-20261026`。
- 快照 ID：`sha256:ab74f8b383d7ea6d85ed1f87aa98b8af8421904f430f6a18d042882291fe37a8`。
- 宿主备份目录：
  `/home/unitree/.copilot/session-state/b8b49699-4bd6-4f7b-9a7a-62462a3b1af7/files/jetson-gpu-before-nvidia-2c7333345ef6/`。

快照通过 `docker commit --pause=false` 在本地创建，没有暂停进程、上传镜像或进行 Git
提交。目录内保留旧容器 inspect、配置副本、标定校验值及用户状态备份。
快照不包含 bind mounts 或 volumes 中的数据，也不保存可继续执行的进程状态。
这些是明确保留的回滚资源，不是待清理的临时测试容器。

### 本次实际使用的 Compose 命令

以下为历史操作记录；重建命令仅可在获得中断许可后执行。沿用现有项目及两个配置文件，
没有另起第二套开发服务，没有额外消耗一份实时调度预算：

```bash
compose=(
  docker compose
  -p unitree_g1_workspace_devcontainer
  -f /home/unitree/Unitree_G1_Workspace/.devcontainer/docker-compose.yml
  -f /home/unitree/.vscode-remote-containers/data/docker-compose/g1-proxy-migration-20261020-088cc4c1547c.yml
)

# 先只解析，不启动
"${compose[@]}" config --quiet

# 本次已获授权并已执行：只重新创建 dev，沿用本地现用镜像
"${compose[@]}" up -d --no-deps --no-build --pull never --force-recreate dev
```

未执行 `docker compose down`、`prune`，未重启 Docker 或宿主。
重建后的容器 ID 为 `7594c8097356`；实际 runtime 为 `nvidia`，服务运行正常。

## 5. 实际验证结果

仓库保留可重复执行的 [CUDA Driver API 验证脚本](../scripts/check_cuda_driver.py)。
只依赖 Python 标准库，不需要编译器、PyCUDA、模型或大型工具链：

```bash
# 宿主执行；今后应先通过 inspect 核对实际容器 ID
docker inspect 7594c8097356 --format \
  'Runtime={{.HostConfig.Runtime}} Running={{.State.Running}}'
docker exec 7594c8097356 python3 /workspace/scripts/check_cuda_driver.py
```

实测输出中的关键结果：

```text
Loaded CUDA driver: /usr/lib/aarch64-linux-gnu/tegra/libcuda.so.1.1
PASS: cuInit returned CUDA_SUCCESS
CUDA driver API version: 11040
CUDA GPU count: 1
GPU 0: Orin
PASS: cuCtxCreate_v2 returned CUDA_SUCCESS
PASS: cuMemAlloc_v2 (4096 bytes) returned CUDA_SUCCESS
PASS: cuMemcpyHtoD_v2 returned CUDA_SUCCESS
PASS: cuMemcpyDtoH_v2 returned CUDA_SUCCESS
PASS: cuCtxSynchronize returned CUDA_SUCCESS
PASS: 4096-byte host/GPU round-trip data matches
PASS: cuMemFree_v2 returned CUDA_SUCCESS
PASS: cuCtxDestroy_v2 returned CUDA_SUCCESS
```

脚本也在修复前的容器执行过，正确打印缺少 `libcuda.so.1` 的错误并返回状态 1，
不存在静默回退到 CPU 的路径。

补充核验：

- 宿主与容器真实 `libcuda.so.1.1` 的 SHA-256 相同：
  `d573af38941ba1e70ac978848f76d36ed7f4e9b8f0cb3f3e869c35b4c2f9eb0c`。
- 容器 mountinfo 显示 Tegra 驱动以单文件只读挂载注入，不是整个系统库目录挂载。
- 容器仍使用自己的 glibc 2.35；宿主为 glibc 2.31，未无差别替换系统库。
- 基础及合并后的 Compose 均解析通过；重建前后对比镜像、原环境变量、挂载、
  网络/IPC、init、安全参数、ulimits 和实时预算，一致。
- `chrt -f 50 true` 成功，仅验证 SCHED_FIFO 权限，没有启动控制节点。
- `/opt/ros/humble/setup.bash` 和持久化工作区可访问。
- 本项目只有一个运行中的开发容器，没有遗留临时测试容器或临时验证文件。

保留的实时设置为：

```text
Docker daemon: cpu-rt-period=1000000, cpu-rt-runtime=950000
dev service:   cpu_rt_runtime=200000
ulimits:       rtprio=99, memlock=-1
```

宿主 `/etc/docker/daemon.json` 和 `/etc/nvidia-container-runtime/config.toml`
修改前后 SHA-256 一致；原 Docker systemd 代理配置未修改，默认 runtime 仍为 `runc`。
这是配置及权限验证，**没有实际运行 ROS 控制环来验证其时序性能**。

## 6. 回滚方法

回滚本身也会中断开发容器，必须先获得许可。不要回退整个工作树或覆盖用户标定文件。

1. 仅从原 Compose 中移除本次新增的 `runtime: nvidia`、
   `NVIDIA_VISIBLE_DEVICES` 和 `NVIDIA_DRIVER_CAPABILITIES`。
2. 用上一节的同一项目、两个原配置文件执行 `config --quiet`。
3. 获准中断后，仅重新创建 `dev` 服务，继续沿用当前镜像及全部原挂载。
4. 检查实际 runtime 恢复为原先的 `runc`；宿主配置无需恢复，Docker 无需重启。

若还需恢复重建前完整可写层，可在备份现状后让**原有历史 override** 的 `image`
指向上述本地回滚镜像，再沿用同项目重建；不创建新的平行配置或另一套服务。
持久化数据仍从原挂载读取，不在镜像快照中。备份目录还可以用于单独恢复用户配置，
无需为了取回一个文件而替换整个容器。

## 7. 未覆盖的 GPU 推理工作

ONNX Runtime 仍为 `1.23.2`，实测 providers 是：

```text
['AzureExecutionProvider', 'CPUExecutionProvider']
```

**本次验证到真实 CUDA 驱动、上下文与内存复制，不代表 GPU 模型推理已通过。**
没有启动模型、CUDA 计算 kernel、相机或机器人控制栈。

后续若需要 GPU 推理，应另行核对 JetPack 5 / CUDA 11.4、aarch64、Python 3.10、
ONNX Runtime CUDA EP、cuDNN/TensorRT 的版本与 ABI 兼容性，不能直接以普通 PC 的
GPU wheel 替换当前环境。如果这一步无法用合理的小改动兼容，可保留当前 Humble
容器，并另行评估与 L4T R35.3.1 匹配的独立 GPU 推理容器；本次未创建该容器。

## 8. 独立 GPU ONNX Runtime 实测（2026-10-27）

已下载、安装并执行真实 `policy.onnx` GPU 推理，不只是 CUDA 驱动或内存复制测试。
没有重建容器、升级 JetPack、修改宿主、替换系统 Python 包或切换机器人默认推理后端。

### 安装位置与版本

- 独立环境：`/workspace/.venv/onnx-gpu`，位于持久化工作区，受现有 `.venv/` 忽略规则保护。
- GPU Runtime：社区包
  [`onnxruntime-gpu-extended==1.18.1.11.8.post1`](https://pypi.org/project/onnxruntime-gpu-extended/1.18.1.11.8.post1/)，
  `cp310-cp310-manylinux2014_aarch64`，模块报告 ORT `1.18.1`。
  **不是 Microsoft 官方 aarch64 GPU wheel**；普通 PyPI `onnxruntime-gpu<1.19` 在本平台没有匹配分发。
- wheel SHA-256：
  `781ed3ae25b49f748f2c5f0957459e227594b978ac81e7e3423eef8fdfb01968`，
  已与 PyPI 发布元数据核对。下载通过现用镜像完成，没有禁用 TLS 验证。
- 原生产环境 `/usr/bin/python3` 仍使用 CPU ORT `1.23.2`，已复核未被覆盖。

从 NVIDIA 官方 JetPack 5 `r35.3` 仓库下载以下 arm64 包，校验仓库 SHA-256 后，
仅用 `dpkg-deb --extract` 解包到独立环境的 `nvidia/`，没有运行包安装脚本或修改系统库：

| 包 | 版本 |
|---|---|
| cuda-cudart-11-4 | 11.4.298-1 |
| libcublas-11-4 | 11.6.6.84-1 |
| libcudnn8 | 8.6.0.166-1+cuda11.4 |
| libcufft-11-4 | 10.6.0.202-1 |

未安装 TensorRT。`libcuda` 仍使用容器原来注入的宿主驱动，不使用 stub 或替代驱动。
下载缓存位于 `/home/unitree/.cache/unitree-gpu-ort/downloads/`。

### 使用方法

独立启动器只给本次 Python 子进程设置本地 CUDA 库路径，不影响其他终端、ROS 节点或系统加载器：

```bash
/workspace/.venv/onnx-gpu/bin/python-cuda -c \
  'import onnxruntime as ort; print(ort.__version__, ort.get_available_providers())'
```

输出为 `1.18.1` 和 `['CUDAExecutionProvider', 'CPUExecutionProvider']`。
仅出现 provider 名称不足以证明执行成功：创建 session 后还须断言 CUDA EP 已初始化，
并通过 profiling 检查实际算子归属，不能把自动 CPU 回退当成 GPU 成功。

复跑常驻模型、每拍新输入的对比：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /workspace/.venv/onnx-gpu/bin/python-cuda \
  /workspace/.venv/onnx-gpu/benchmark_policy.py

# 另一个独立进程，额外尝试 CUDA blocking-sync 调度
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /workspace/.venv/onnx-gpu/bin/python-cuda \
  /workspace/.venv/onnx-gpu/benchmark_policy.py --blocking-sync
```

验证数据及两轮测试输出位于独立环境的 `verification/`，不应提交生成物或将该环境作为
ROS 默认 Python 环境。当前 `InsertPolicy.load()` 仍明确选择 CPU；安装 GPU 包不代表任务已切换 GPU。

### 正确性与性能结果

- FP32，关闭 TF32，batch=1；模型初始化不计时，每帧使用新观测并延续 GRU 隐状态。
- profiling 的 100 帧中，两层 GRU、Gemm 等全部 15 个记录算子均在 CUDA EP 执行；
  不是 CPU 回退。
- 另与原生产 CPU ORT `1.23.2` 连续对照 200 帧：动作最大绝对误差约 `3.34e-6`，
  隐状态约 `4.52e-6`，通过 `rtol=1e-4, atol=1e-5`。
- 同一 ORT `1.18.1` 内对比 CPU EP/CUDA EP；每种配置连续热跑每次 3000 帧、
  50 Hz 每次 200 帧，两次反转测试顺序。IOBinding 版本额外将隐状态常驻 GPU，
  预分配输出，只把新观测输入和动作输出放在 CPU 侧，并验证了状态连续性。

50 Hz 下的输入到动作可读延迟，不包含拍间等待或策略观测组装：

| 调用方式 | 中位延迟范围 | P95 范围 | 按进程 CPU 时间折算的一核占用 |
|---|---:|---:|---:|
| CPU EP | 0.28–0.31 ms | 0.33–0.43 ms | 约 1.5–1.6% |
| CUDA EP，隐状态也往返 CPU | 1.46–1.56 ms | 1.87–2.09 ms | 约 7.9–8.0% |
| CUDA EP，IOBinding 隐状态常驻 GPU | 1.32–1.48 ms | 1.61–1.94 ms | 约 7.1–7.7% |

blocking-sync 在此实现中没有改善降 CPU 的结论。以上是未锁频的独立测试，
不是完整生产负载验收；也没有测试 TensorRT、FP16 或 CUDA Graph。
**当前 CUDA EP 路径对这个小 GRU 更慢且更耗 CPU，因此未切换默认后端。**
这不能推广成其他模型或其他 GPU 实现都没有加速收益。

## 9. 30 Hz 图像链路与原生内核

本容器 OpenCV 4.5.4 为 CPU 构建，不能仅改一个参数就调用 `cv2.cuda`。
当前任务通过已有 CUDA Driver API/PTX 做灰度/饱和度，ArUco 仍是 CPU OpenCV，
候选恢复另编译为 C++ Python 扩展；ONNX 仍按前述结论使用 CPU。

实际 Azure Kinect JPEG 为 1280×720、4:2:2，包含完整 Huffman 表。
已用现存 `libgstapp` 注册公开 appsrc/appsink 接口，跑通
`nvjpegdec → NVMM → nvvidconv → Y42B → full-range BGR`。
合成双码角点完全一致，真实图像相对 CPU JPEG 的像素平均误差约 `0.052`、最大 `2`。
不能直接用 limited-range I420→BGR 处理 JPEG full-range 图像；
本机 nvvidconv 的直接 RGB 路径还曾出现灰度范围错误，因此没有采用。

但该 NVIDIA 插件链路包含转换/拷贝后，中位约 `17.12 ms`、CPU `13.44 ms/帧`，
原 CPU JPEG 约 `9.41 / 9.40 ms`，并未降 CPU，所以没有接入默认解码路径。
没有安装此前缺失的 GStreamer 标准插件，也没有重编译/替换 OpenCV。

当前 [相机配置](../src/upper_insert/config/camera_config.json) 使用设备原生
`YUYV 1280×720@30`，保留 4:2:2，直接省掉 JPEG 解码。
单独采集 A/B 的 CPU 成本从 MJPEG 约 `12.85` 降到 YUYV `7.22 ms/帧`。
原生 YUV 需要两个 V4L2 缓冲，避免 BGR 转换期间无可用 DMA 缓冲；
用户态仍只缓存最新帧，视觉最多一帧在途。它不是全程 GPU 或零拷贝链路。

当前默认组合为原生 YUYV + CUDA 灰度/饱和度 + C++ 恢复 + C++ IK。
需先构建 `g1_motion_session` 与 `upper_insert`；本机 Pinocchio Python/C++ 均为 4.0.0，
开发头文件/动态库已存在，不需下载新依赖或重启容器。
详细吞吐、数值一致性、CPU-only 限制和验证命令见
[upper_insert](../src/upper_insert/README.md#c-与-gpu-图像流水线) 与
[原生 IK](../src/g1_motion_session/README.md#原生-ik-内核)。

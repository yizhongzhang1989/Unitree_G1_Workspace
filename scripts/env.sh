#!/usr/bin/env bash
# Unitree G1 工作区的 ROS 2 运行环境，唯一定义。一个文件两种用法：
#   - 被 source：只设环境变量。
#       source scripts/env.sh
#     容器里的 ~/.bashrc 也走这条——VS Code "Reopen in Container" 开的终端不经过
#     ENTRYPOINT，没有它就是个没有 ROS 的裸 shell。
#   - 被执行：设完环境再 exec 传进来的命令。
#       scripts/env.sh ros2 topic list
#     容器的 ENTRYPOINT 走这条，因为 `dev.sh <命令>` 这种一次性调用不经过 bash，
#     也就读不到 ~/.bashrc。
# 可重复 source，不产生输出。

_EE_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export G1_WORKSPACE="$_EE_WS"
export END_EFFECTOR_ROS_ROOT="$_EE_WS"

source /opt/ros/humble/setup.bash

if [ -f "$_EE_WS/install/setup.bash" ]; then
	source "$_EE_WS/install/setup.bash"
fi

# 根目录 sdk 下的纯 Python SDK 保持独立，不交给 colcon 构建，直接从源码目录导入。
case ":${PYTHONPATH:-}:" in
	*":$_EE_WS/sdk/CAN-SDK:"*) ;;
	*) export PYTHONPATH="$_EE_WS/sdk/CAN-SDK:$_EE_WS/sdk/KWR57-SDK:$_EE_WS/sdk/Gloria-M-SDK/src${PYTHONPATH:+:$PYTHONPATH}" ;;
esac

# 运行必须用 CycloneDDS：默认的 FastRTPS 会刷 std::bad_alloc。Humble 自带的 0.10.x
# 就是 Unitree 要求的版本，不需要官方那套源码编译的 cyclonedds_ws。
#
# 必须限定网卡：不指定时 Cyclone 会挑到 wlan0，表现为收不到 /lowstate。配置直接内联
# 在 CYCLONEDDS_URI 里（Cyclone 接受 XML 文本，不只是 file://），省掉一个配置文件；
# 换网卡时设 G1_DDS_INTERFACE 即可，不用编辑 XML。
#
# SocketReceiveBufferSize 是给大图像话题用的：1280x720 YUYV 一帧 1.84 MB，30 fps
# 就是 55 MB/s。Cyclone 切成 UDP 分片发，收包缓冲一满就丢片，丢一片整帧作废。实测
# 内核默认 rmem_max=208KB 时头部相机 30 fps 只收到 27.7（RcvbufErrors 10 秒涨 2427），
# 加到 8MB 后满帧且 RcvbufErrors 归零。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${G1_DDS_INTERFACE:-eth0}\" priority=\"default\" multicast=\"default\"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General><Internal><SocketReceiveBufferSize min=\"8MB\"/></Internal></Domain></CycloneDDS>}"

# 上面那 8MB 会被内核按 rmem_max 悄悄钳掉，钳掉后没有任何报错，只是默默丢帧。
# 容器是 network_mode: host，这个值就是宿主机的全局值，得在**宿主机**上持久化：
#   echo 'net.core.rmem_max=16777216' | sudo tee /etc/sysctl.d/60-ros2-dds.conf
#   sudo sysctl --system
if [ "$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)" -lt 8388608 ]; then
    echo "警告: net.core.rmem_max 过小，大图像话题会静默丢帧（见 scripts/env.sh）" >&2
fi

unset _EE_WS

# 被执行（而不是被 source）时，继续跑传进来的命令。
if ! (return 0 2>/dev/null); then
	exec "$@"
fi

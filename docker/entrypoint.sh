#!/usr/bin/env bash
# 容器入口：装配 ROS 2 Humble 环境后执行传入的命令。
set -e

source /opt/ros/humble/setup.bash

WS="${G1_WORKSPACE:-/home/unitree/Unitree_G1_Workspace}"
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

# sdk 下的纯 Python SDK 不交给 colcon，直接从源码目录导入。
if [ -d "$WS/sdk" ]; then
    export PYTHONPATH="$WS/sdk/CAN-SDK:$WS/sdk/KWR57-SDK:$WS/sdk/Gloria-M-SDK/src${PYTHONPATH:+:$PYTHONPATH}"
fi

# Unitree 机器人在 eth0 上，必须限定网卡，否则会在 wlan0 上发现不到 /lowstate。
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$WS/docker/cyclonedds.xml}"

exec "$@"

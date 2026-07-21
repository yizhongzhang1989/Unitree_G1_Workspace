#!/usr/bin/env bash
# source 本文件配置 Unitree G1 工作区的 ROS 2 运行环境（CycloneDDS）
#   source scripts/env.sh

# 工作区根目录（相对本脚本定位）
_EE_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export END_EFFECTOR_ROS_ROOT="$_EE_WS"

# 根目录 sdk 下的纯 Python SDK 保持独立，不交给 colcon 构建。ROS 节点直接从源码目录导入；
# 用户若已用 pip 安装 SDK，该设置仍确保当前工作区源码优先，便于联调。
_EE_SDK_PYTHONPATH="$_EE_WS/sdk/CAN-SDK:$_EE_WS/sdk/KWR57-SDK:$_EE_WS/sdk/Gloria-M-SDK/src"
export PYTHONPATH="$_EE_SDK_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"

source /opt/ros/foxy/setup.bash

_CYCLONEDDS_WS="${UNITREE_CYCLONEDDS_WS:-$HOME/cyclonedds_ws}"
if [ ! -f "$_CYCLONEDDS_WS/install/setup.bash" ]; then
	echo "未找到 Unitree CycloneDDS 环境：$_CYCLONEDDS_WS/install/setup.bash" >&2
	echo "请按 README 中的 Unitree ROS 2 官方步骤安装 CycloneDDS" >&2
	return 1 2>/dev/null || exit 1
fi
source "$_CYCLONEDDS_WS/install/setup.bash"

_PROJECT_SETUP="$_EE_WS/install/setup.bash"
if [ ! -f "$_PROJECT_SETUP" ]; then
	echo "未找到项目构建环境：$_PROJECT_SETUP" >&2
	echo "请先按 README 构建项目包" >&2
	return 1 2>/dev/null || exit 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-$_CYCLONEDDS_WS/cyclonedds.xml}"
export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source "$_PROJECT_SETUP"

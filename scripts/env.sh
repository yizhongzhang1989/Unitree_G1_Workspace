#!/usr/bin/env bash
# source 本文件配置 end_effector_ros 的 ROS 2 运行环境（CycloneDDS）。
#   source scripts/env.sh
# 默认 FastRTPS 在本机会刷 std::bad_alloc，故用 CycloneDDS。

# 工作区根目录（相对本脚本定位）
_EE_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export END_EFFECTOR_ROS_ROOT="$_EE_WS"

# 根目录 sdk 下的纯 Python SDK 保持独立，不交给 colcon 构建。ROS 节点直接从源码目录导入；
# 用户若已用 pip 安装 SDK，该设置仍确保当前工作区源码优先，便于联调。
_EE_SDK_PYTHONPATH="$_EE_WS/sdk/CAN-SDK:$_EE_WS/sdk/KWR57-SDK:$_EE_WS/sdk/Gloria-M-SDK/src"
export PYTHONPATH="$_EE_SDK_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"

source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
[ -f "$_EE_WS/install/setup.bash" ] && source "$_EE_WS/install/setup.bash"

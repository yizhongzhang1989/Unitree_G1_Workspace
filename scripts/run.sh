#!/usr/bin/env bash
# 一键启动末端执行器与左右相机。Ctrl-C 退出时清理所有节点、释放 CAN 设备。
#   bash scripts/run.sh            # 单总线（默认）
#   bash scripts/run.sh single
#   bash scripts/run.sh dual       # 双总线（每臂一条总线）
set -e

MODE="${1:-single}"
case "$MODE" in
  single) LAUNCH=single_bus.launch.py ;;
  dual)   LAUNCH=dual_bus.launch.py ;;
  *) echo "用法: run.sh [single|dual]"; exit 1 ;;
esac

# shellcheck source=/dev/null
source "$(cd "$(dirname "$0")" && pwd)/env.sh"

# 已安装节点可执行文件路径片段，用于兜底清理（避免误杀其它进程）
NODES='can_bridge_ros/lib/can_bridge_ros/bridge_node|kwr57_ros/lib/kwr57_ros/ft_sensor_node|gloria_ros/lib/gloria_ros/gripper_node|camera_node/lib/camera_node/camera_node'

cleanup() {
  trap - EXIT INT TERM
  # ros2 launch 会把节点放独立会话，父进程被杀会残留孤儿占着 CAN；按可执行路径清掉。
  pkill -INT -f "$NODES" 2>/dev/null || true
  sleep 0.6
  pkill -KILL -f "$NODES" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "启动 robot_bringup $LAUNCH （Ctrl-C 退出并清理）..."
ros2 launch robot_bringup "$LAUNCH"

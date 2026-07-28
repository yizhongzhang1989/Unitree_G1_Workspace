#!/usr/bin/env bash
# 在 G1 Humble 容器里跑命令；不带参数进交互 shell。
#   docker/run.sh                       # 交互 shell
#   docker/run.sh colcon build          # 一次性命令
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
IMAGE="${G1_IMAGE:-g1-humble:latest}"

# --privileged + --network host + --ipc host：USB/CAN 直通、与机器人同一网络栈、
# DDS 共享内存。rtprio/memlock 放开是为了让 ros2_control_node 能拿到 SCHED_FIFO
# 和 mlockall——这正是 Humble 相对 Foxy 的主要实时收益。
DOCKER_ARGS=(
    --rm
    --privileged
    --network host
    --ipc host
    --ulimit rtprio=99
    --ulimit memlock=-1
    -v /dev:/dev
    -v "$WS:$WS"
    -v "$HOME/.ros:$HOME/.ros"
    -e "G1_WORKSPACE=$WS"
    -w "$WS"
)

if [ -t 0 ]; then
    DOCKER_ARGS+=(-it)
fi

mkdir -p "$HOME/.ros"
exec docker run "${DOCKER_ARGS[@]}" "$IMAGE" "${@:-bash}"

#!/usr/bin/env bash
# G1 Humble 开发容器的入口脚本。
#   .devcontainer/dev.sh build [...]     构建镜像
#   .devcontainer/dev.sh                 交互 shell
#   .devcontainer/dev.sh <命令...>       一次性执行，退出即销毁
#
# 运行参数全在 docker-compose.yml 里，devcontainer.json 也引用同一份，不抄两遍。
# VS Code "Reopen in Container" 用不到本脚本；它是给普通 SSH 会话、开机自启和 CI 用的。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose -f "$HERE/docker-compose.yml")

if [ "${1:-}" = "build" ]; then
    shift
    # 本机直连不到 mirrors.ustc.edu.cn，构建期必须借宿主的 SOCKS 代理。这些是 docker
    # build 的预定义 ARG，只在构建期生效，不会写进镜像。
    # VS Code 的 "Reopen in Container" 不会带这些变量，所以镜像要先在这里做好。
    export G1_BUILD_PROXY="${G1_BUILD_PROXY:-${ALL_PROXY:-${all_proxy:-}}}"
    export G1_USER_NAME="$(id -un)"
    export G1_USER_UID="$(id -u)"
    export G1_USER_GID="$(id -g)"
    if [ -n "$G1_BUILD_PROXY" ]; then
        echo "使用代理构建：$G1_BUILD_PROXY" >&2
    else
        echo "未检测到 ALL_PROXY，直连构建（大概率拉不到 USTC）" >&2
    fi
    exec "${COMPOSE[@]}" build "$@"
fi

mkdir -p "$HOME/.ros"
exec "${COMPOSE[@]}" run --rm dev "${@:-bash}"

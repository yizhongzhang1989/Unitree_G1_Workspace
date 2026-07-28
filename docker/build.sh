#!/usr/bin/env bash
# 构建 G1 Humble 开发镜像。
#   docker/build.sh [额外的 docker build 参数...]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${G1_IMAGE:-g1-humble:latest}"

# 本机直连不到 mirrors.ustc.edu.cn，必须借宿主的 SOCKS 代理。--network host 让容器
# 里的 127.0.0.1:1080 就是宿主的代理端口。http_proxy 等是 docker build 的预定义
# ARG，只在构建期生效，不会写进镜像。
PROXY="${G1_BUILD_PROXY:-${ALL_PROXY:-${all_proxy:-}}}"

BUILD_ARGS=(
    --build-arg "USER_NAME=$(id -un)"
    --build-arg "USER_UID=$(id -u)"
    --build-arg "USER_GID=$(id -g)"
)
if [ -n "$PROXY" ]; then
    BUILD_ARGS+=(
        --build-arg "http_proxy=$PROXY"
        --build-arg "https_proxy=$PROXY"
        --build-arg "all_proxy=$PROXY"
        --build-arg "no_proxy=localhost,127.0.0.1"
    )
    echo "使用代理构建：$PROXY" >&2
else
    echo "未检测到 ALL_PROXY，直连构建（大概率拉不到 USTC）" >&2
fi

exec docker build \
    --network host \
    "${BUILD_ARGS[@]}" \
    -t "$IMAGE" \
    "$@" \
    "$HERE"

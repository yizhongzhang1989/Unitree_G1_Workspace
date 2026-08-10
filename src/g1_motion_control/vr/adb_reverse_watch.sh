#!/usr/bin/env bash
# 头显休眠 / USB 重枚举 / 无线断连都会清空 adb reverse 规则，这里持续检测并自动重建。
#
# 用法:
#   ./adb_reverse_watch.sh                        # USB 连接
#   ./adb_reverse_watch.sh 192.168.1.50:5555      # 无线连接（会自动重连）
#   ./adb_reverse_watch.sh 192.168.1.50:5555 8000 # 指定端口
set -u

DEVICE="${1:-}"
PORT="${2:-8000}"

adb_args=()
[ -n "$DEVICE" ] && adb_args=(-s "$DEVICE")

echo "watching adb reverse tcp:${PORT}${DEVICE:+ on $DEVICE} (Ctrl+C 退出)"
while true; do
    if ! adb "${adb_args[@]}" reverse --list 2>/dev/null | grep -q "tcp:${PORT}"; then
        # 无线场景下先尝试重连，USB 场景下这步跳过
        [ -n "$DEVICE" ] && adb connect "$DEVICE" >/dev/null 2>&1
        if adb "${adb_args[@]}" reverse "tcp:${PORT}" "tcp:${PORT}" >/dev/null 2>&1; then
            echo "$(date +%T) restored reverse tcp:${PORT}"
        fi
    fi
    sleep 2
done

#!/usr/bin/env bash
# 给 submodule 打本仓库自己的补丁。**幂等**：重复跑不会出错，也不会重复应用。
#
# 为什么需要：submodule 钉在上游的某个 commit 上，直接改工作树会让 `git status`
# 一直脏、`git submodule update` 会冲掉，而 fork 一份又要长期维护。补丁存在本仓库、
# 构建前打一次，两边都不欠。
#
#   scripts/apply_patches.sh            打补丁（已打过就跳过）
#   scripts/apply_patches.sh --check    只报告状态，不改动，未打全时返回 1
#   scripts/apply_patches.sh --revert   还原到上游原样
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCH_ROOT="$ROOT/patches"
MODE="${1:-apply}"

case "$MODE" in
    apply | --check | --revert) ;;
    *) echo "用法: $0 [--check|--revert]" >&2; exit 2 ;;
esac

pending=0
applied=0

for patch in "$PATCH_ROOT"/*/*.patch; do
    [[ -e "$patch" ]] || { echo "没有补丁文件（$PATCH_ROOT）"; exit 0; }
    pkg="$(basename "$(dirname "$patch")")"
    target="$ROOT/src/$pkg"
    name="$pkg/$(basename "$patch")"

    if [[ ! -d "$target" ]]; then
        echo "  跳过 $name：$target 不存在（submodule 没 checkout？）"
        continue
    fi

    # --reverse --check 能过 = 补丁已经在工作树里了
    if git -C "$target" apply --reverse --check "$patch" 2>/dev/null; then
        if [[ "$MODE" == "--revert" ]]; then
            git -C "$target" apply --reverse "$patch"
            echo "  已还原 $name"
        else
            echo "  已应用 $name"
            applied=$((applied + 1))
        fi
        continue
    fi

    if ! git -C "$target" apply --check "$patch" 2>/dev/null; then
        # 既打不上、也不是已打过的状态 —— 多半是上游动了这块代码
        echo "  ⚠️  $name 打不上：上游代码可能已变，需要重做补丁" >&2
        pending=$((pending + 1))
        continue
    fi

    if [[ "$MODE" == "--check" ]]; then
        echo "  未应用 $name"
        pending=$((pending + 1))
    elif [[ "$MODE" == "--revert" ]]; then
        echo "  本来就没打 $name"
    else
        git -C "$target" apply "$patch"
        echo "  已打上 $name"
        applied=$((applied + 1))
    fi
done

if [[ "$MODE" == "--check" && $pending -gt 0 ]]; then
    echo "有 $pending 个补丁没应用，跑 scripts/apply_patches.sh" >&2
    exit 1
fi
[[ $pending -gt 0 ]] && exit 1
exit 0

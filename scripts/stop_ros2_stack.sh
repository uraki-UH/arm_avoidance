#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STOP_TIMEOUT="${ROS2_STOP_TIMEOUT:-10}"
ROS_SERVICES=(gng_cpu rosbridge)
DRY_RUN=false

case "${1:-}" in
    "") ;;
    --dry-run) DRY_RUN=true ;;
    -h|--help)
        echo "Usage: ./scripts/stop_ros2_stack.sh [--dry-run]"
        exit 0
        ;;
    *)
        echo "不明なオプションです: $1" >&2
        exit 2
        ;;
esac

cd "$PROJECT_DIR"

if ! command -v docker >/dev/null 2>&1; then
    echo "dockerコマンドが見つかりません。" >&2
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "Dockerへ接続できません。Dockerの起動状態と権限を確認してください。" >&2
    exit 1
fi

mapfile -t running_services < <(docker compose ps --services --status running)
targets=()

for service in "${ROS_SERVICES[@]}"; do
    for running_service in "${running_services[@]}"; do
        if [[ "$service" == "$running_service" ]]; then
            targets+=("$service")
            break
        fi
    done
done

if (( ${#targets[@]} == 0 )); then
    echo "停止対象のROS 2コンテナはありません。"
    exit 0
fi

if $DRY_RUN; then
    echo "停止対象（dry-run）: ${targets[*]}"
    exit 0
fi

echo "ROS 2コンテナを停止します: ${targets[*]}"
docker compose stop -t "$STOP_TIMEOUT" "${targets[@]}"

remaining_services="$(docker compose ps --services --status running)"
for service in "${targets[@]}"; do
    if grep -qx "$service" <<< "$remaining_services"; then
        echo "停止できませんでした: $service" >&2
        exit 1
    fi
done

echo "ROS 2コンテナを停止しました。frontendは停止していません。"

#!/usr/bin/env bash
set -euo pipefail

# ホストとROSコンテナで共通の、ブラウザ非依存シミュレーション入口
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
simulator_dir="$workspace_dir/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    exec python3 "$simulator_dir/integrations/ros2/simulate.py" --help
fi
if [[ -f /.dockerenv || "${1:-}" == --local ]]; then
    if [[ "${1:-}" == --local ]]; then shift; fi
    exec bash "$simulator_dir/start_ros.sh" --simulate "$@"
fi
tty_arguments=(-T)
# 対話端末ではCtrl+Cのコンテナ側への伝達。自動検証ではTTY不要。
if [[ -t 0 && -t 1 ]]; then tty_arguments=(); fi
exec docker compose --project-directory "$workspace_dir" exec "${tty_arguments[@]}" gng_cpu \
    bash /ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/start_ros.sh --simulate "$@"

#!/usr/bin/env bash
set -euo pipefail

# 双腕シミュレータ専用プロファイルと、描画検証時のウィンドウ設定
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
model="${1:-long}"
if (( $# > 0 )); then shift; fi
case "$model" in
    standard|long) ;;
    *) printf '%s\n' '引数は standard または long を指定してください。' >&2; exit 2 ;;
esac
# Docker側の応答確認後にブラウザを起動。既存サービスは再利用
workspace_dir="$(cd -- "$script_dir/.." && pwd)"
if [[ "${enable_backend:-1}" == 1 ]]; then
    docker compose --project-directory "$workspace_dir" exec -T gng_cpu \
        bash /ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/start_ros.sh --ensure
fi
exec env viewer_gpu="${viewer_gpu:-auto}" \
    viewer_profile_dir="${viewer_profile_dir:-${XDG_CACHE_HOME:-$HOME/.cache}/topo-dual-arm-gpu-auto}" \
    bash "$script_dir/open_viewer_nvidia.sh" \
    "http://127.0.0.1:8877/?model=$model" \
    --ozone-platform=x11 --window-size=1440,1000 "$@"

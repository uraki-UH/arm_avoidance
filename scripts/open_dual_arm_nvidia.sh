#!/usr/bin/env bash
set -euo pipefail

# 双腕シミュレータ専用プロファイルと、描画検証時のウィンドウ設定
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
model=long
if (( $# > 0 )) && [[ "$1" != --* ]]; then model="$1"; shift; fi
case "$model" in
    standard|long) ;;
    *) printf '%s\n' '引数は standard または long を指定してください。' >&2; exit 2 ;;
esac
# Docker側の応答確認後にブラウザを起動。既存サービスは再利用
workspace_dir="$(cd -- "$script_dir/.." && pwd)"
viewer_url="http://127.0.0.1:8877/?model=$model"
if [[ "${1:-}" == --replay ]]; then
    shift
    if [[ "$model" != long ]]; then
        printf '%s\n' '回避実験の記録再生はlong用です。' >&2
        exit 2
    fi
    replay_case_dir=latest
    if (( $# > 0 )) && [[ "$1" != --* ]]; then replay_case_dir="$1"; shift; fi
    python3 "$workspace_dir/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/sphere_replay.py" \
        --case-dir "$replay_case_dir" >/dev/null
    viewer_url+='&replay=latest'
fi
if [[ "${enable_backend:-1}" == 1 ]]; then
    docker compose --project-directory "$workspace_dir" exec -T gng_cpu \
        bash /ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/start_ros.sh --ensure
fi
exec env viewer_gpu="${viewer_gpu:-auto}" \
    viewer_profile_prefix=topo-dual-arm \
    bash "$script_dir/open_viewer_nvidia.sh" \
    "$viewer_url" \
    --ozone-platform=x11 --window-size=1440,1000 "$@"

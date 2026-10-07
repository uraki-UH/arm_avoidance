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
exec env viewer_gpu="${viewer_gpu:-auto}" \
    viewer_profile_dir="${viewer_profile_dir:-${XDG_CACHE_HOME:-$HOME/.cache}/topo-dual-arm-gpu-auto}" \
    bash "$script_dir/open_viewer_nvidia.sh" \
    "http://127.0.0.1:8877/?model=$model" \
    --ozone-platform=x11 --window-size=1440,1000 "$@"

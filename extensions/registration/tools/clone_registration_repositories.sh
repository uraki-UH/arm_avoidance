#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "使用法: $0 取得先ディレクトリ" >&2
  exit 1
fi

command -v vcs >/dev/null || { echo "vcsコマンドが必要です（ROS開発環境のvcstool）。" >&2; exit 1; }
registration_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source_root="$registration_dir"
core_dir="$(cd -- "$registration_dir/../.." && pwd -P)"
if [[ -f "$core_dir/workspace.yaml" ]]; then source_root="$core_dir"; fi
target_dir="$(realpath -m -- "$1")"
if [[ "$target_dir" == / || "$target_dir" == "$source_root" || "$target_dir" == "$source_root/"* ]]; then
  echo "外部リポジトリの取得先は本体の外へ指定してください。" >&2
  exit 2
fi
mkdir -p -- "$target_dir"
# 固定commitの任意取得。既存checkoutの変更・強制上書きなし。
exec vcs import --recursive --skip-existing --input "$registration_dir/upstream.repos" "$target_dir"

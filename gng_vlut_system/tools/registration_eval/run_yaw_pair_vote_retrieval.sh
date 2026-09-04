#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "使用法: $0 dataset_dir query_dataset [除外template_id ...]" >&2
  exit 1
fi

dataset_dir="$1"
query_dataset="$2"
shift 2

registration_dir="$(cd "$(dirname "$0")" && pwd)"
output_path="/tmp/yaw_pair_vote_retrieval"

g++ -std=c++17 -O3 \
  "$registration_dir/yaw_pair_vote_retrieval.cpp" \
  -o "$output_path" \
  -lz

"$output_path" "$dataset_dir" "$query_dataset" "$@"

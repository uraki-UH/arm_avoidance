#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "使用法: $0 dataset_dir query_dataset [評価オプション...]" >&2
  exit 1
fi

dataset_dir="$1"
query_dataset="$2"
shift 2

registration_dir="$(cd "$(dirname "$0")" && pwd)"
output_path="/tmp/plane_cluster_fuzzy_retrieval"

g++ -std=c++17 -O3 \
  "$registration_dir/plane_cluster_fuzzy_retrieval.cpp" \
  -o "$output_path" \
  -lz

"$output_path" "$dataset_dir" "$query_dataset" "$@"

#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "使用法: $0 TEASER-plusplusディレクトリ dataset_dir query_dataset [除外template_id ...]" >&2
  exit 1
fi

teaser_dir="$1"
dataset_dir="$2"
query_dataset="$3"
shift 3

registration_dir="$(cd "$(dirname "$0")" && pwd)"
teaser_library_dir="$teaser_dir/build/teaser"
teaser_library_path="$teaser_library_dir/libteaser_registration.so"
output_path="/tmp/teaser_template_retrieval"

if [[ ! -f "$teaser_library_path" ]]; then
  echo "TEASER++を先にbuild_teaser.shでビルドしてください: $teaser_library_path" >&2
  exit 2
fi

g++ -std=c++17 -O3 \
  "$registration_dir/teaser_template_retrieval.cpp" \
  -o "$output_path" \
  -I/usr/include/eigen3 \
  -I"$teaser_dir/teaser/include" \
  -L"$teaser_library_dir" \
  -Wl,-rpath,"$teaser_library_dir" \
  -lteaser_registration \
  -lz \
  -fopenmp

"$output_path" "$dataset_dir" "$query_dataset" "$@"

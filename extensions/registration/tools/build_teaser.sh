#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "使用法: $0 TEASER-plusplusのディレクトリ" >&2
  exit 1
fi

teaser_dir="$1"
cmake -S "$teaser_dir" -B "$teaser_dir/build" -DTEASERPP_BUILD_TESTS=ON
cmake --build "$teaser_dir/build" -j"$(nproc)"
ctest --test-dir "$teaser_dir/build" --output-on-failure

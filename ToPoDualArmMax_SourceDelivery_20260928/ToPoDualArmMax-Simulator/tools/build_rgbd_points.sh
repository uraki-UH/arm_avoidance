#!/usr/bin/env bash
# ブラウザ用点群処理の再生成。Clangとwasm-ldを備えた環境向け。
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
compiler=${1:-clang++}
output=$(mktemp "${TMPDIR:-/tmp}/topo-rgbd-points.XXXXXX.wasm")
trap 'rm -f -- "$output"' EXIT
"$compiler" --target=wasm32 -O3 -nostdlib -fno-exceptions -fno-rtti -ffp-contract=off -fno-fast-math -mbulk-memory -msimd128 \
  -Wl,--no-entry -Wl,--export=process_points -Wl,--export=__heap_base -Wl,--export-memory \
  -Wl,--initial-memory=131072 -Wl,--max-memory=268435456 -Wl,-z,stack-size=16384 \
  "$root/native/rgbd_points.cpp" -o "$output"
install -m 644 -- "$output" "$root/app/rgbd-points.wasm"

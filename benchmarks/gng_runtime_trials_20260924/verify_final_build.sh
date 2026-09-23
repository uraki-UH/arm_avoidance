#!/usr/bin/env bash
set -euo pipefail
trial_artifacts=/ros2_ws/src/artifacts/gng_runtime_trials_20260924
trial_build=/tmp/gng_runtime_trials_build
# 測定済みライブラリを保持したまま、最終ソースとのバイト一致を確認。
for method in combined radix; do
  timeout 600 cmake --build "$trial_build/$method" -j 4 > "$trial_artifacts/$method/final_build.log" 2>&1
  timeout 120 ctest --test-dir "$trial_build/$method" --output-on-failure > "$trial_artifacts/$method/final_tests.log" 2>&1
  for library in "$trial_build/$method"/libgng_minimal_*.so; do
    cmp "$library" "$trial_artifacts/$method/${library##*/}"
  done
  echo "$method final libraries unchanged; tests passed"
done

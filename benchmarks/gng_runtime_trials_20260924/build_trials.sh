#!/usr/bin/env bash
set -euo pipefail
trial_source=/ros2_ws/src/ais_gng_cpu/experimental/gng_runtime_trials
trial_artifacts=/ros2_ws/src/artifacts/gng_runtime_trials_20260924
trial_build=/tmp/gng_runtime_trials_build
mkdir -p "$trial_build"
ulimit -c 0
# コピー元の未初期化ベクトルの再現。比較対象すべてには同じ修正を適用。
g++ -std=c++20 -O3 -DNDEBUG "$trial_source/test/vector_zero_test.cpp" \
  /ros2_ws/src/ais_gng_cpu/experimental/gng_minimal_comparison/src/utils/vec3f.cpp \
  -I "$trial_source/include" -o "$trial_build/vector_zero_original"
set +e
timeout 10 "$trial_build/vector_zero_original" > "$trial_artifacts/vector_zero_original.log" 2>&1
original_status=$?
set -e
echo "$original_status" > "$trial_artifacts/vector_zero_original.status"
test "$original_status" -ne 0
for method in baseline hint heap lto fused combined; do
  case "$method" in
    baseline) free_mode=0; enable_lto=OFF; enable_fused=OFF; variants="tree_raw;tree_voxel;grid_raw;grid_voxel" ;;
    hint) free_mode=1; enable_lto=OFF; enable_fused=OFF; variants=tree_raw ;;
    heap) free_mode=2; enable_lto=OFF; enable_fused=OFF; variants=tree_raw ;;
    lto) free_mode=0; enable_lto=ON; enable_fused=OFF; variants="tree_raw;tree_voxel" ;;
    fused) free_mode=0; enable_lto=OFF; enable_fused=ON; variants=tree_voxel ;;
    combined) free_mode=1; enable_lto=ON; enable_fused=ON; variants="tree_raw;tree_voxel;grid_raw;grid_voxel" ;;
  esac
  mkdir -p "$trial_artifacts/$method"
  timeout 120 cmake -S "$trial_source" -B "$trial_build/$method" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DGNG_FREE_NODE_MODE="$free_mode" -DGNG_ENABLE_LTO="$enable_lto" -DGNG_RADIX_VOXELS=OFF \
    -DGNG_FUSE_VOXEL_REDUCTION="$enable_fused" -DGNG_BUILD_VARIANTS="$variants" \
    > "$trial_artifacts/$method/configure.log" 2>&1
  timeout 600 cmake --build "$trial_build/$method" -j 4 > "$trial_artifacts/$method/build.log" 2>&1
  timeout 120 ctest --test-dir "$trial_build/$method" --output-on-failure \
    > "$trial_artifacts/$method/tests.log" 2>&1
  cp "$trial_build/$method"/libgng_minimal_*.so "$trial_build/$method/CMakeCache.txt" \
    "$trial_build/$method/compile_commands.json" "$trial_artifacts/$method/"
  echo "$method build and tests complete"
done

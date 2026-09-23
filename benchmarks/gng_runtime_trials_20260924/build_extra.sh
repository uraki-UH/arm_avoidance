#!/usr/bin/env bash
set -euo pipefail
trial_source=/ros2_ws/src/ais_gng_cpu/experimental/gng_runtime_trials
trial_artifacts=/ros2_ws/src/artifacts/gng_runtime_trials_20260924
trial_build=/tmp/gng_runtime_trials_build
for method in radix native; do
  case "$method" in
    radix) enable_radix=ON; enable_native=OFF; variants="tree_voxel;grid_voxel" ;;
    native) enable_radix=OFF; enable_native=ON; variants="tree_raw;tree_voxel;grid_raw;grid_voxel" ;;
  esac
  mkdir -p "$trial_artifacts/$method"
  timeout 120 cmake -S "$trial_source" -B "$trial_build/$method" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DGNG_FREE_NODE_MODE=1 -DGNG_ENABLE_LTO=ON -DGNG_FUSE_VOXEL_REDUCTION=ON \
    -DGNG_RADIX_VOXELS="$enable_radix" -DGNG_NATIVE_CPU="$enable_native" \
    -DGNG_BUILD_VARIANTS="$variants" > "$trial_artifacts/$method/configure.log" 2>&1
  timeout 600 cmake --build "$trial_build/$method" -j 4 > "$trial_artifacts/$method/build.log" 2>&1
  timeout 120 ctest --test-dir "$trial_build/$method" --output-on-failure > "$trial_artifacts/$method/tests.log" 2>&1
  cp "$trial_build/$method"/libgng_minimal_*.so "$trial_build/$method/CMakeCache.txt" \
    "$trial_build/$method/compile_commands.json" "$trial_artifacts/$method/"
  echo "$method build and tests complete"
done

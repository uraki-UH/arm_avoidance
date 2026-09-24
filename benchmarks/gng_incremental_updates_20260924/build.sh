#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_incremental_updates_20260924
trial_build=/tmp/gng_incremental_updates_build
for method in "$@"; do
  mkdir -p "$trial_artifacts/$method"
  timeout 120 cmake -S "$trial_artifacts/${method}_deterministic" -B "$trial_build/$method" \
    -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_AUTHENTICATION=OFF -DGNG_ENABLE_FRAME_LOG=ON \
    -DGNG_BUILD_BENCHMARKS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > "$trial_artifacts/$method/configure.log" 2>&1
  timeout 900 cmake --build "$trial_build/$method" -j 3 > "$trial_artifacts/$method/build.log" 2>&1
  timeout 240 ctest --test-dir "$trial_build/$method" --no-tests=error --output-on-failure > "$trial_artifacts/$method/tests.log" 2>&1
  timeout 45 "$trial_build/$method/gng_training_event_api_test" > "$trial_artifacts/$method/training_events.log" 2>&1
  timeout 45 "$trial_build/$method/gng_map_delta_api_test" > "$trial_artifacts/$method/map_delta.log" 2>&1
  cp "$trial_build/$method/libgng_cpu.so" "$trial_build/$method/CMakeCache.txt" \
     "$trial_build/$method/compile_commands.json" "$trial_artifacts/$method/"
  echo "$method Release build and tests passed"
done

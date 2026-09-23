#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_source=/ros2_ws/src/ais_gng_cpu/experimental/gng_radix_trials
trial_artifacts=/ros2_ws/src/artifacts/gng_radix_multiseed_20260924
trial_build=/tmp/gng_radix_multiseed_build
mkdir -p "$trial_artifacts"
touch "$trial_artifacts/COLCON_IGNORE"
for method in boost radix; do
  if [ "$method" = radix ]; then enable_radix=ON; else enable_radix=OFF; fi
  mkdir -p "$trial_artifacts/$method"
  timeout 90 cmake -S "$trial_source" -B "$trial_build/$method" \
    -DCMAKE_BUILD_TYPE=Release -DGNG_BUILD_BENCHMARKS=ON -DGNG_ENABLE_AUTHENTICATION=OFF \
    -DGNG_ENABLE_FRAME_LOG=ON -DGNG_DETERMINISTIC_BENCHMARK=ON -DGNG_RADIX_VOXELS="$enable_radix" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > "$trial_artifacts/$method/configure.log" 2>&1
  timeout 600 cmake --build "$trial_build/$method" -j 4 > "$trial_artifacts/$method/build.log" 2>&1
  timeout 180 ctest --test-dir "$trial_build/$method" --output-on-failure > "$trial_artifacts/$method/tests.log" 2>&1
  timeout 30 "$trial_build/$method/gng_training_event_api_test" > "$trial_artifacts/$method/training_events.log" 2>&1
  timeout 30 "$trial_build/$method/gng_map_delta_api_test" > "$trial_artifacts/$method/map_delta.log" 2>&1
  cp "$trial_build/$method/libgng_cpu.so" "$trial_build/$method/CMakeCache.txt" \
     "$trial_build/$method/compile_commands.json" "$trial_artifacts/$method/"
  echo "$method Release build and tests passed"
done

#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_artifacts=/ros2_ws/src/artifacts/gng_unused_mapping_20260924
trial_build=/tmp/gng_unused_mapping_build_20260924
for method in before after; do
  mkdir -p "$trial_artifacts/$method"
  timeout 120 cmake -S "$trial_artifacts/${method}_deterministic" -B "$trial_build/$method" \
    -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_AUTHENTICATION=OFF -DGNG_ENABLE_FRAME_LOG=ON \
    -DGNG_BUILD_BENCHMARKS=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > "$trial_artifacts/$method/configure.log" 2>&1
  timeout 300 cmake --build "$trial_build/$method" -j 3 > "$trial_artifacts/$method/build.log" 2>&1
  cp "$trial_build/$method/libgng_cpu.so" "$trial_build/$method/CMakeCache.txt" \
     "$trial_build/$method/compile_commands.json" "$trial_artifacts/$method/"
  echo "$method Release build complete"
done
timeout 120 cmake -S /ros2_ws/src/ais_gng_cpu/src/gng_cpu -B /ros2_ws/build/gng_cpu \
  -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_AUTHENTICATION=OFF -DGNG_ENABLE_FRAME_LOG=ON \
  -DGNG_BUILD_BENCHMARKS=ON > "$trial_artifacts/production_configure.log" 2>&1
timeout 600 cmake --build /ros2_ws/build/gng_cpu -j 3 > "$trial_artifacts/production_build.log" 2>&1
timeout 240 ctest --test-dir /ros2_ws/build/gng_cpu --no-tests=error --output-on-failure > "$trial_artifacts/production_tests.log" 2>&1
for test_name in gng_training_event_api_test gng_map_delta_api_test; do
  timeout 45 "/ros2_ws/build/gng_cpu/$test_name" > "$trial_artifacts/$test_name.log" 2>&1
done
echo 'production Release build and tests complete'

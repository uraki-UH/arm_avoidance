#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_followup_efficiency_20260924
test -f "$trial_artifacts/adoption.json"
timeout 120 cmake -S "$trial_root/ais_gng_cpu/src/gng_cpu" -B /ros2_ws/build/gng_cpu \
  -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_AUTHENTICATION=OFF -DGNG_ENABLE_FRAME_LOG=ON \
  -DGNG_BUILD_BENCHMARKS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  > "$trial_artifacts/production_configure.log" 2>&1
timeout 900 cmake --build /ros2_ws/build/gng_cpu -j 3 > "$trial_artifacts/production_build.log" 2>&1
timeout 240 ctest --test-dir /ros2_ws/build/gng_cpu --no-tests=error --output-on-failure \
  > "$trial_artifacts/production_tests.log" 2>&1
timeout 45 /ros2_ws/build/gng_cpu/gng_training_event_api_test > "$trial_artifacts/production_training_events.log" 2>&1
timeout 45 /ros2_ws/build/gng_cpu/gng_map_delta_api_test > "$trial_artifacts/production_map_delta.log" 2>&1
python3 "$trial_root/benchmarks/gng_followup_efficiency_20260924/install.py"
timeout --signal=INT --kill-after=10s 240 taskset -c 0 \
  python3 "$trial_root/benchmarks/gng_followup_efficiency_20260924/benchmark.py" \
  --library /ros2_ws/install/gng_cpu/lib/libgng_cpu.so \
  --config "$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml" \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --output "$trial_artifacts/installed_smoke.json" --frames 30 --warmup 5 --voxel 0.1 --features \
  > "$trial_artifacts/installed_smoke.log" 2>&1
timeout --signal=INT --kill-after=20s 1200 bash -ic cb > "$trial_artifacts/cb.log" 2>&1
timeout 180 ctest --test-dir /ros2_ws/build/gng_wasm_core -R gng_wasm_core_cpu_kernel_test \
  --no-tests=error --output-on-failure > "$trial_artifacts/wasm_production_tests.log" 2>&1
echo 'production Release build, tests, atomic install, smoke, cb and wasm test passed'

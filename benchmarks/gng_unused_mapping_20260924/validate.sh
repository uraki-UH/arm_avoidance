#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_unused_mapping_20260924
timeout 240 cmake --build /ros2_ws/build/gng_wasm_core -j 3 > "$trial_artifacts/wasm_build.log" 2>&1
timeout 60 ctest --test-dir /ros2_ws/build/gng_wasm_core -R gng_wasm_core_cpu_kernel_test \
  --no-tests=error --output-on-failure > "$trial_artifacts/wasm_tests.log" 2>&1
python3 "$trial_root/benchmarks/gng_unused_mapping_20260924/install.py"
timeout --signal=INT --kill-after=10s 240 taskset -c 0 \
  python3 "$trial_root/benchmarks/gng_followup_efficiency_20260924/benchmark.py" \
  --library /ros2_ws/install/gng_cpu/lib/libgng_cpu.so \
  --config "$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml" \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --output "$trial_artifacts/installed_smoke.json" --frames 30 --warmup 5 --voxel 0.1 --features \
  > "$trial_artifacts/installed_smoke.log" 2>&1
echo 'WASM native test and installed runtime smoke passed'

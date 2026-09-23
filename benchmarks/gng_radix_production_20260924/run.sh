#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_radix_production_20260924
trial_benchmark=$trial_root/benchmarks/gng_radix_production_20260924/benchmark.py
trial_config=$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml
trial_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
for trial in 1 2 3; do
  if [ "$trial" = 1 ]; then frames=300; else frames=100; fi
  if [ "$trial" = 2 ]; then methods='radix boost'; else methods='boost radix'; fi
  for voxel in 0.1 0.5; do
    for method in $methods; do
      timeout 240 taskset -c 0 python3 "$trial_benchmark" \
        --library "$trial_artifacts/$method/libgng_cpu.so" --config "$trial_config" --bag "$trial_bag" \
        --output "$trial_artifacts/$method/voxel_${voxel}_${trial}.json" \
        --frames "$frames" --warmup 50 --voxel "$voxel" \
        > "$trial_artifacts/$method/voxel_${voxel}_${trial}.log" 2>&1
      echo "$method voxel=$voxel trial=$trial frames=$frames complete"
    done
  done
done
for method in boost radix; do
  for voxel in 0 0.1 0.5; do
    timeout 180 taskset -c 0 python3 "$trial_benchmark" \
      --library "$trial_artifacts/$method/libgng_cpu.so" --config "$trial_config" --bag "$trial_bag" \
      --output "$trial_artifacts/$method/features_${voxel}.json" --frames 30 --warmup 5 --voxel "$voxel" --features \
      > "$trial_artifacts/$method/features_${voxel}.log" 2>&1
  done
  echo "$method features and disabled-voxel tests complete"
done

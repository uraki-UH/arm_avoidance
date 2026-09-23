#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_normal_efficiency_20260924
for trial in 1 2 3; do
  if [ "$trial" = 1 ]; then frames=300; else frames=100; fi
  if [ "$trial" = 2 ]; then methods='reuse before'; else methods='before reuse'; fi
  for voxel in 0.1 0.5; do
    for method in $methods; do
      timeout --signal=INT --kill-after=10s 240 taskset -c 0 python3 "$trial_root/benchmarks/gng_normal_efficiency_20260924/benchmark.py" \
        --library "$trial_artifacts/$method/libgng_cpu.so" \
        --config "$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml" \
        --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
        --output "$trial_artifacts/$method/voxel_${voxel}_${trial}.json" --frames "$frames" --warmup 50 --voxel "$voxel" \
        > "$trial_artifacts/$method/voxel_${voxel}_${trial}.log" 2>&1
      echo "$method voxel=$voxel trial=$trial complete"
    done
  done
done
for method in before reuse; do
  for voxel in 0 0.1 0.5; do
    timeout --signal=INT --kill-after=10s 180 taskset -c 0 python3 "$trial_root/benchmarks/gng_normal_efficiency_20260924/benchmark.py" \
      --library "$trial_artifacts/$method/libgng_cpu.so" \
      --config "$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml" \
      --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
      --output "$trial_artifacts/$method/features_${voxel}.json" --frames 30 --warmup 5 --voxel "$voxel" --features \
      > "$trial_artifacts/$method/features_${voxel}.log" 2>&1
  done
  echo "$method features complete"
done

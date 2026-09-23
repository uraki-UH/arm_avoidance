#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_normal_efficiency_20260924
for voxel in 0.1 0.5; do
  for method in before after; do
    timeout --signal=INT --kill-after=10s 180 taskset -c 0 python3 "$trial_root/benchmarks/gng_normal_efficiency_20260924/benchmark.py" \
      --library "$trial_artifacts/$method/libgng_cpu.so" \
      --config "$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml" \
      --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
      --output "$trial_artifacts/$method/probe_${voxel}.json" --frames 80 --warmup 20 --voxel "$voxel" \
      > "$trial_artifacts/$method/probe_${voxel}.log" 2>&1
    echo "$method voxel=$voxel probe complete"
  done
done

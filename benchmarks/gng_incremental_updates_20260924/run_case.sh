#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_incremental_updates_20260924
method=$1
voxel=$2
frames=$3
warmup=$4
name=$5
shift 5
timeout --signal=INT --kill-after=10s 600 taskset -c 0 \
  python3 "$trial_root/benchmarks/gng_followup_efficiency_20260924/benchmark.py" \
  --library "$trial_artifacts/$method/libgng_cpu.so" \
  --config "$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml" \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --output "$trial_artifacts/$method/$name.json" --frames "$frames" --warmup "$warmup" --voxel "$voxel" "$@" \
  > "$trial_artifacts/$method/$name.log" 2>&1
echo "$method $name complete"

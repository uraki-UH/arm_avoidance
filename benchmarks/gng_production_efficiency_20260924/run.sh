#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_production_efficiency_20260924
trial_benchmark=$trial_root/benchmarks/gng_production_efficiency_20260924/benchmark.py
trial_config=$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml
trial_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
for trial in 1 2 3; do
  if [ "$trial" = 1 ]; then frames=300; else frames=100; fi
  if [ "$trial" = 2 ]; then methods='after before'; else methods='before after'; fi
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
for method in before after; do
  timeout 180 taskset -c 0 python3 "$trial_benchmark" \
    --library "$trial_artifacts/$method/libgng_cpu.so" --config "$trial_config" --bag "$trial_bag" \
    --output "$trial_artifacts/$method/raw.json" --frames 100 --warmup 50 --voxel 0 \
    > "$trial_artifacts/$method/raw.log" 2>&1
  for voxel in 0 0.1 0.5; do
    if [ "$voxel" = 0.1 ]; then case_name=smoke; else case_name=features_$voxel; fi
    timeout 180 taskset -c 0 python3 "$trial_benchmark" \
      --library "$trial_artifacts/$method/libgng_cpu.so" --config "$trial_config" --bag "$trial_bag" \
      --output "$trial_artifacts/$method/$case_name.json" --frames 30 --warmup 5 --voxel "$voxel" --features \
      > "$trial_artifacts/$method/$case_name.log" 2>&1
  done
  echo "$method raw and additional features complete"
done

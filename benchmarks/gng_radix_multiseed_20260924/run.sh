#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_radix_multiseed_20260924
trial_benchmark=$trial_root/benchmarks/gng_radix_multiseed_20260924/benchmark.py
trial_config=$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml
trial_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
trial_num=0
for seed in 11 101 1009 10007 104729 20260924; do
  trial_num=$((trial_num + 1))
  if (( trial_num % 2 )); then methods='boost radix'; else methods='radix boost'; fi
  for voxel in 0.1 0.5; do
    for method in $methods; do
      timeout --signal=INT --kill-after=10s 240 taskset -c 0 python3 "$trial_benchmark" \
        --library "$trial_artifacts/$method/libgng_cpu.so" --config "$trial_config" --bag "$trial_bag" \
        --output "$trial_artifacts/$method/voxel_${voxel}_seed_${seed}.json" \
        --frames 200 --warmup 50 --voxel "$voxel" --seed "$seed" --observe \
        > "$trial_artifacts/$method/voxel_${voxel}_seed_${seed}.log" 2>&1
      echo "$method voxel=$voxel seed=$seed frames=200 complete"
    done
  done
done
# 観測ON/OFFと同一セル内代表点変更の、学習への独立性確認。
for method in boost radix; do
  for voxel in 0.1 0.5; do
    for mode in off last; do
      extra=()
      if [ "$mode" = last ]; then extra=(--observe --last-representative); fi
      timeout --signal=INT --kill-after=10s 180 taskset -c 0 python3 "$trial_benchmark" \
        --library "$trial_artifacts/$method/libgng_cpu.so" --config "$trial_config" --bag "$trial_bag" \
        --output "$trial_artifacts/$method/observation_${voxel}_${mode}.json" \
        --frames 30 --warmup 5 --voxel "$voxel" --seed 20260924 "${extra[@]}" \
        > "$trial_artifacts/$method/observation_${voxel}_${mode}.log" 2>&1
      echo "$method voxel=$voxel observation=$mode complete"
    done
  done
done

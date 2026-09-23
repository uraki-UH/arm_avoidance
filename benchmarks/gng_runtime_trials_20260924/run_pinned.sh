#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_artifacts=/ros2_ws/src/artifacts/gng_runtime_trials_20260924
trial_configs=/ros2_ws/src/benchmarks/gng_runtime_trials_20260924
trial_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_runtime_trials/benchmark.py
trial_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
# 許可CPU内の先頭を使用する単一コア固定の確認。既存プロセスのaffinity変更なし。
trial_cpu=$(python3 -c 'import os; print(min(os.sched_getaffinity(0)))')
echo "$trial_cpu" > "$trial_artifacts/pinned_cpu.txt"
for trial in 1 2 3; do
  case "$trial" in
    1) cases=(baseline/tree_raw combined/tree_raw baseline/tree_voxel combined/tree_voxel radix/tree_voxel) ;;
    2) cases=(radix/tree_voxel combined/tree_voxel baseline/tree_voxel combined/tree_raw baseline/tree_raw) ;;
    3) cases=(combined/tree_raw baseline/tree_raw combined/tree_voxel radix/tree_voxel baseline/tree_voxel) ;;
  esac
  for item in "${cases[@]}"; do
    method="${item%/*}"; variant="${item#*/}"
    OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 taskset -c "$trial_cpu" python3 "$trial_script" \
      --library "$trial_artifacts/$method/libgng_minimal_$variant.so" \
      --config "$trial_configs/at128_voxel_0_1.yaml" --bag "$trial_bag" \
      --output "$trial_artifacts/$method/${variant}_pinned_${trial}.json" \
      > "$trial_artifacts/$method/${variant}_pinned_${trial}.stdout" 2>&1
    python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(sys.argv[1], round(d["summary"]["exec_ms"]["mean"],3))' \
      "$trial_artifacts/$method/${variant}_pinned_${trial}.json"
  done
done

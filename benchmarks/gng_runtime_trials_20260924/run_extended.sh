#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_artifacts=/ros2_ws/src/artifacts/gng_runtime_trials_20260924
trial_configs=/ros2_ws/src/benchmarks/gng_runtime_trials_20260924
trial_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_runtime_trials/benchmark.py
trial_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
# 同じ300個の連続入力。方式ごとに基準版と合成版を交互に計測。
for variant in tree_raw grid_raw tree_voxel_0_1 grid_voxel_0_1 tree_voxel_0_5 grid_voxel_0_5; do
  case "$variant" in
    *_0_1) library="${variant%_0_1}"; size=0_1 ;;
    *_0_5) library="${variant%_0_5}"; size=0_5 ;;
    *) library="$variant"; size=0_1 ;;
  esac
  case "$variant" in
    tree*) methods=(baseline combined) ;;
    grid*) methods=(combined baseline) ;;
  esac
  if [[ "$variant" == *voxel* ]]; then methods+=(radix); fi
  for method in "${methods[@]}"; do
    OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 180 python3 "$trial_script" \
      --library "$trial_artifacts/$method/libgng_minimal_$library.so" \
      --config "$trial_configs/at128_voxel_$size.yaml" --bag "$trial_bag" \
      --frames 300 --warmup 50 --bag-frames 300 --quality-every 25 \
      --output "$trial_artifacts/$method/${variant}_extended.json" \
      > "$trial_artifacts/$method/${variant}_extended.stdout" 2>&1
    python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(sys.argv[1], round(d["summary"]["exec_ms"]["mean"],3))' \
      "$trial_artifacts/$method/${variant}_extended.json"
  done
done

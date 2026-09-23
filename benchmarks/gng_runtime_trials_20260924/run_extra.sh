#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_artifacts=/ros2_ws/src/artifacts/gng_runtime_trials_20260924
trial_configs=/ros2_ws/src/benchmarks/gng_runtime_trials_20260924
trial_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_runtime_trials/benchmark.py
trial_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
for trial in 1 2 3; do
  case "$trial" in
    1) methods=(native radix combined) ;;
    2) methods=(radix combined native) ;;
    3) methods=(combined native radix) ;;
  esac
  for method in "${methods[@]}"; do
    case "$method" in
      radix) variants=(tree_voxel_0_1 grid_voxel_0_1 tree_voxel_0_5 grid_voxel_0_5) ;;
      *) variants=(tree_raw grid_raw tree_voxel_0_1 grid_voxel_0_1 tree_voxel_0_5 grid_voxel_0_5) ;;
    esac
    for variant in "${variants[@]}"; do
      case "$variant" in
        *_0_1) library="${variant%_0_1}"; size=0_1 ;;
        *_0_5) library="${variant%_0_5}"; size=0_5 ;;
        *) library="$variant"; size=0_1 ;;
      esac
      OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$trial_script" \
        --library "$trial_artifacts/$method/libgng_minimal_$library.so" \
        --config "$trial_configs/at128_voxel_$size.yaml" --bag "$trial_bag" \
        --output "$trial_artifacts/$method/${variant}_extra_${trial}.json" \
        > "$trial_artifacts/$method/${variant}_extra_${trial}.stdout" 2>&1
      python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(sys.argv[1], round(d["summary"]["exec_ms"]["mean"],3))' \
        "$trial_artifacts/$method/${variant}_extra_${trial}.json"
    done
  done
done

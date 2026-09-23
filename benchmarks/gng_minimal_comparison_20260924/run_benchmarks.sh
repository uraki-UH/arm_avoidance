#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
comparison_dir=/ros2_ws/src/artifacts/gng_minimal_comparison_20260924
config_dir=/ros2_ws/src/benchmarks/gng_minimal_comparison_20260924
benchmark_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_minimal_comparison/benchmark.py
benchmark_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
# 順番による偏りを抑える6条件の順次計測。並列計測なし。
for trial in 1 2 3; do
  case "$trial" in
    1) variants=(tree_raw grid_raw tree_voxel_0_1 grid_voxel_0_1 tree_voxel_0_5 grid_voxel_0_5) ;;
    2) variants=(grid_voxel_0_1 tree_voxel_0_1 grid_raw tree_raw grid_voxel_0_5 tree_voxel_0_5) ;;
    3) variants=(tree_voxel_0_5 grid_voxel_0_5 tree_raw grid_raw tree_voxel_0_1 grid_voxel_0_1) ;;
  esac
  for variant in "${variants[@]}"; do
    case "$variant" in
      *_0_1) library="${variant%_0_1}"; size=0_1 ;;
      *_0_5) library="${variant%_0_5}"; size=0_5 ;;
      *) library="$variant"; size=0_1 ;;
    esac
    OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$benchmark_script" \
      --library "$comparison_dir/libgng_minimal_$library.so" \
      --config "$config_dir/at128_voxel_$size.yaml" --bag "$benchmark_bag" \
      --output "$comparison_dir/${variant}_${trial}.json" \
      > "$comparison_dir/${variant}_${trial}.stdout" 2>&1
    tail -1 "$comparison_dir/${variant}_${trial}.stdout"
  done
done

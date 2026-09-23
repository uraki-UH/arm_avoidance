#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
comparison_dir=/ros2_ws/src/artifacts/gng_spatial_nearest_20260923
benchmark_config=/ros2_ws/src/benchmarks/gng_spatial_nearest_20260923/at128_snapshot.yaml
mkdir -p "$comparison_dir"
benchmark_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree/benchmark.py
benchmark_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
for trial in 1 2 3; do
  for variant in grid spatial; do
    OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$benchmark_script" \
      --library "$comparison_dir/libgng_${variant}.so" \
      --config "$benchmark_config" \
      --bag "$benchmark_bag" \
      --output "$comparison_dir/${variant}_${trial}.json" \
      > "$comparison_dir/${variant}_${trial}.stdout" 2>&1
    tail -1 "$comparison_dir/${variant}_${trial}.stdout"
  done
done
OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$benchmark_script" \
  --library /ros2_ws/src/artifacts/gng_spatial_tree_20260923/libgng_spatial.so \
  --config "$benchmark_config" \
  --bag "$benchmark_bag" \
  --output "$comparison_dir/old_range.json" \
  > "$comparison_dir/old_range.stdout" 2>&1
tail -1 "$comparison_dir/old_range.stdout"

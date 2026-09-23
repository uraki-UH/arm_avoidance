#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
comparison_dir=/ros2_ws/src/artifacts/gng_bsp3d_sampled_20260923
config_dir=/ros2_ws/src/benchmarks/gng_bsp3d_sampled_20260923
for variant in pure sampled; do
  for unit in 0_001 1_0; do
    OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 \
      /ros2_ws/src/ais_gng_cpu/experimental/gng_bsp3d_sampled/benchmark.py \
      --library "$comparison_dir/libgng_bsp3d_${variant}.so" \
      --config "$config_dir/at128_node_grid_${unit}.yaml" \
      --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
      --output "$comparison_dir/grid_invariance_${variant}_${unit}.json" \
      > "$comparison_dir/grid_invariance_${variant}_${unit}.stdout" 2>&1
    tail -1 "$comparison_dir/grid_invariance_${variant}_${unit}.stdout"
  done
done

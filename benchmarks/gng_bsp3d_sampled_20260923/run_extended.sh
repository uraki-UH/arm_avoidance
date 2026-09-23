#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
comparison_dir=/ros2_ws/src/artifacts/gng_bsp3d_sampled_20260923
config_dir=/ros2_ws/src/benchmarks/gng_bsp3d_sampled_20260923
benchmark_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_bsp3d_sampled/benchmark.py
benchmark_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
for variant in grid sampled; do
  if [ "$variant" = grid ]; then
    library=/ros2_ws/src/artifacts/gng_bsp3d_20260923/libgng_grid.so
  else
    library="$comparison_dir/libgng_bsp3d_sampled.so"
  fi
  OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 180 python3 "$benchmark_script" \
    --library "$library" --config "$config_dir/at128_voxel_0_1.yaml" --bag "$benchmark_bag" \
    --frames 300 --warmup 50 --bag-frames 300 --quality-every 25 \
    --output "$comparison_dir/extended_${variant}.json" > "$comparison_dir/extended_${variant}.stdout" 2>&1
  tail -1 "$comparison_dir/extended_${variant}.stdout"
done

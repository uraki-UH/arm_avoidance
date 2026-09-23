#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
comparison_dir=/ros2_ws/src/artifacts/gng_bsp3d_20260923
benchmark_config=/ros2_ws/src/benchmarks/gng_bsp3d_20260923/at128_snapshot.yaml
mkdir -p "$comparison_dir"
benchmark_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree/benchmark.py
benchmark_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
for trial in 1 2 3; do
  case "$trial" in
    1) variants=(grid spatial bsp3d) ;;
    2) variants=(spatial bsp3d grid) ;;
    3) variants=(bsp3d grid spatial) ;;
  esac
  for variant in "${variants[@]}"; do
    OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$benchmark_script" \
      --library "$comparison_dir/libgng_${variant}.so" \
      --config "$benchmark_config" \
      --bag "$benchmark_bag" \
      --output "$comparison_dir/${variant}_${trial}.json" \
      > "$comparison_dir/${variant}_${trial}.stdout" 2>&1
    tail -1 "$comparison_dir/${variant}_${trial}.stdout"
  done
done

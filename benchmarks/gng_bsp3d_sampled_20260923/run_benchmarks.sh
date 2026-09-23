#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
comparison_dir=/ros2_ws/src/artifacts/gng_bsp3d_sampled_20260923
config_dir=/ros2_ws/src/benchmarks/gng_bsp3d_sampled_20260923
benchmark_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_bsp3d_sampled/benchmark.py
benchmark_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
num_trials=${1:-3}
for trial in $(seq 1 "$num_trials"); do
  case "$trial" in
    1) variants=(grid bsp3d pure sampled) ;;
    2) variants=(sampled pure bsp3d grid) ;;
    *) variants=(pure grid sampled bsp3d) ;;
  esac
  for voxel in 0_1 0_5; do
    for variant in "${variants[@]}"; do
      case "$variant" in
        grid|bsp3d) library=/ros2_ws/src/artifacts/gng_bsp3d_20260923/libgng_${variant}.so ;;
        *) library="$comparison_dir/libgng_bsp3d_${variant}.so" ;;
      esac
      OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$benchmark_script" \
        --library "$library" --config "$config_dir/at128_voxel_${voxel}.yaml" \
        --bag "$benchmark_bag" --output "$comparison_dir/${variant}_${voxel}_${trial}.json" \
        > "$comparison_dir/${variant}_${voxel}_${trial}.stdout" 2>&1
      tail -1 "$comparison_dir/${variant}_${voxel}_${trial}.stdout"
    done
  done
done

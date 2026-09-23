#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
comparison_dir=/ros2_ws/src/artifacts/gng_bsp3d_minimal_20260923
config_dir=/ros2_ws/src/benchmarks/gng_bsp3d_minimal_20260923
benchmark_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_bsp3d_minimal/benchmark.py
benchmark_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
for trial in 1 2 3; do
  case "$trial" in
    1) variants=(grid sampled minimal) ;;
    2) variants=(minimal grid sampled) ;;
    3) variants=(sampled minimal grid) ;;
  esac
  for variant in "${variants[@]}"; do
    case "$variant" in
      grid) library=/ros2_ws/src/artifacts/gng_bsp3d_20260923/libgng_grid.so ;;
      sampled) library=/ros2_ws/src/artifacts/gng_bsp3d_sampled_20260923/libgng_bsp3d_sampled.so ;;
      minimal) library="$comparison_dir/libgng_bsp3d_minimal.so" ;;
    esac
    OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$benchmark_script" \
      --library "$library" --config "$config_dir/at128_voxel_0_1.yaml" --bag "$benchmark_bag" \
      --output "$comparison_dir/${variant}_${trial}.json" \
      > "$comparison_dir/${variant}_${trial}.stdout" 2>&1
    tail -1 "$comparison_dir/${variant}_${trial}.stdout"
  done
done
# 未使用のvoxel設定による出力差がないことの確認。
OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$benchmark_script" \
  --library "$comparison_dir/libgng_bsp3d_minimal.so" --config "$config_dir/at128_voxel_0_5.yaml" \
  --bag "$benchmark_bag" --output "$comparison_dir/minimal_0_5.json" > "$comparison_dir/minimal_0_5.stdout" 2>&1
# 繰返しなしの連続入力による最小版の確認。
OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 180 python3 "$benchmark_script" \
  --library "$comparison_dir/libgng_bsp3d_minimal.so" --config "$config_dir/at128_voxel_0_1.yaml" \
  --bag "$benchmark_bag" --frames 300 --warmup 50 --bag-frames 300 --quality-every 25 \
  --output "$comparison_dir/extended_minimal.json" > "$comparison_dir/extended_minimal.stdout" 2>&1
tail -1 "$comparison_dir/extended_minimal.stdout"

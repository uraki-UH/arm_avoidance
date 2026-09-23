#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
profile_dir=/ros2_ws/src/artifacts/gng_bsp3d_profile_20260923
mkdir -p "$profile_dir"
benchmark_script=/ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree/benchmark.py
benchmark_bag=/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
benchmark_config=/ros2_ws/src/benchmarks/gng_bsp3d_20260923/at128_snapshot.yaml
for mode in coarse detail control; do
  case "$mode" in
    coarse) detail=0; library=/tmp/gng_bsp3d_profile_build/libgng_bsp3d.so ;;
    detail) detail=1; library=/tmp/gng_bsp3d_profile_build/libgng_bsp3d.so ;;
    control) detail=0; library=/ros2_ws/src/artifacts/gng_bsp3d_20260923/libgng_bsp3d.so ;;
  esac
  GNG_PROFILE_DETAIL="$detail" OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 120 python3 "$benchmark_script" \
    --library "$library" --config "$benchmark_config" --bag "$benchmark_bag" \
    --output "$profile_dir/${mode}.json" \
    > "$profile_dir/${mode}.stdout" 2> "$profile_dir/${mode}.profile"
  tail -1 "$profile_dir/${mode}.stdout"
done

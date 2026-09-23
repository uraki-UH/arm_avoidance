#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
mkdir -p /ros2_ws/src/artifacts/gng_bsp3d_profile_20260923
GNG_PROFILE_DETAIL=1 GNG_PROFILE_SAMPLE_STRIDE=64 OPENBLAS_NUM_THREADS=1 \
  timeout -s INT -k 5 120 python3 \
  /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree/benchmark.py \
  --library /tmp/gng_bsp3d_sample_profile_build/libgng_bsp3d.so \
  --config /ros2_ws/src/benchmarks/gng_bsp3d_20260923/at128_snapshot.yaml \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --output /ros2_ws/src/artifacts/gng_bsp3d_profile_20260923/sampled.json \
  > /ros2_ws/src/artifacts/gng_bsp3d_profile_20260923/sampled.stdout \
  2> /ros2_ws/src/artifacts/gng_bsp3d_profile_20260923/sampled.profile

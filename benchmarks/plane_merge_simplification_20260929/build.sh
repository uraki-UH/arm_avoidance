#!/usr/bin/env bash
set -euo pipefail

# 保存済みの変更前ヘッダーを優先した、同一最適化条件の比較用ビルド。
cd /ros2_ws/src
trial_root=artifacts/plane_merge_simplification_20260929
cp benchmarks/plane_merge_simplification_20260929/plane_parameters.txt "$trial_root/plane_parameters.txt"
trial_flags=(-std=c++17 -O3 -DNDEBUG -Iais_gng_cpu/src/ais_gng/include -I/usr/include/eigen3 -I/ros2_ws/install/ais_gng_msgs/include/ais_gng_msgs)
for package in geometry_msgs std_msgs builtin_interfaces rosidl_runtime_cpp rosidl_runtime_c rosidl_typesupport_interface rcutils; do
  trial_flags+=(-I"/opt/ros/humble/include/$package")
done
for mode in before after; do
  trial_source=ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp
  trial_mode_flags=()
  if [[ "$mode" == before ]]; then
    trial_source=$trial_root/before.cpp
    trial_mode_flags=(-I"$trial_root/before_include" -Dplane_merge_benchmark_before)
  fi
  g++ "${trial_mode_flags[@]}" "${trial_flags[@]}" "$trial_source" \
    benchmarks/plane_merge_simplification_20260929/replay.cpp -o "$trial_root/replay_$mode"
done

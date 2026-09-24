#!/usr/bin/env bash
set -euo pipefail
cd /ros2_ws/src
trial_root=artifacts/plane_consistency_20260924
trial_mode=${1:-after}
trial_include=ais_gng_cpu/src/ais_gng/include
trial_source=ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp
if [[ "$trial_mode" == before || "$trial_mode" == scale_before || "$trial_mode" == contact_before ]]; then
  trial_include=$trial_root/$trial_mode/include
  trial_source=$trial_root/$trial_mode/plane_cluster_incremental.cpp
fi
# 追加オプションのABIを揃えた変更前アルゴリズムの比較。実装だけを保存版へ切替。
if [[ "$trial_mode" == fragment_before || "$trial_mode" == direction_before || "$trial_mode" == absorption_before ]]; then
  trial_source=$trial_root/$trial_mode/plane_cluster_incremental.cpp
fi
mkdir -p "$trial_root/$trial_mode"
trial_flags=(-std=c++17 -O3 -DNDEBUG -I"$trial_include" -I/usr/include/eigen3 -I/ros2_ws/install/ais_gng_msgs/include/ais_gng_msgs)
for package in geometry_msgs std_msgs builtin_interfaces rosidl_runtime_cpp rosidl_runtime_c rosidl_typesupport_interface rcutils; do
  trial_flags+=(-I"/opt/ros/humble/include/$package")
done
g++ "${trial_flags[@]}" -c "$trial_source" -o "$trial_root/$trial_mode/plane.o"
g++ "${trial_flags[@]}" ais_gng_cpu/src/ais_gng/test/test_plane_cluster_incremental.cpp "$trial_root/$trial_mode/plane.o" -lgtest_main -lgtest -pthread -o "$trial_root/$trial_mode/tests"
g++ "${trial_flags[@]}" ais_gng_cpu/src/ais_gng/test/benchmark_plane_cluster_incremental.cpp "$trial_root/$trial_mode/plane.o" -o "$trial_root/$trial_mode/synthetic"
g++ "${trial_flags[@]}" benchmarks/plane_consistency_20260924/replay.cpp "$trial_root/$trial_mode/plane.o" -o "$trial_root/$trial_mode/replay"

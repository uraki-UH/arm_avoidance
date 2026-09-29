#!/usr/bin/env bash
set -euo pipefail
cd /ros2_ws/src
trial_root=artifacts/voxel_plane_lookup_20260928/inheritance_perf
trial_flags=(-std=c++17 -O3 -DNDEBUG -Iais_gng_cpu/src/ais_gng/include -I/usr/include/eigen3 -I/ros2_ws/install/ais_gng_msgs/include/ais_gng_msgs)
for package in geometry_msgs std_msgs builtin_interfaces rosidl_runtime_cpp rosidl_runtime_c rosidl_typesupport_interface rcutils; do
  trial_flags+=(-I"/opt/ros/humble/include/$package")
done
for mode in before after; do
  trial_source=$trial_root/before.cpp
  if [[ "$mode" == after ]]; then
    trial_source=ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp
  fi
  g++ "${trial_flags[@]}" "$trial_source" benchmarks/voxel_plane_lookup_20260928/replay_inheritance.cpp -o "$trial_root/replay_$mode"
done

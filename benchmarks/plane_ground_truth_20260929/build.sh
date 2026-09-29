#!/usr/bin/env bash
set -euo pipefail

# 同じGNG本体と、保存済み変更前・現行平面実装による単体比較用ビルド。
trial_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
trial_output=${1:-"$trial_root/artifacts/plane_ground_truth_20260929"}
trial_install=${PLANE_GT_INSTALL_ROOT:-/ros2_ws/install}
trial_ros=${PLANE_GT_ROS_ROOT:-/opt/ros/humble}
trial_gng="$trial_root/ais_gng_cpu/src/gng_cpu"
mkdir -p "$trial_output"
trial_flags=(-std=c++20 -O3 -DNDEBUG -DGNG_VERSION=0
  -I"$trial_gng/src" -I"$trial_gng/include"
  -I"$trial_root/ais_gng_cpu/src/ais_gng/include"
  -I/usr/include/eigen3 -I"$trial_install/ais_gng_msgs/include/ais_gng_msgs")
for package in geometry_msgs std_msgs builtin_interfaces rosidl_runtime_cpp rosidl_runtime_c rosidl_typesupport_interface rcutils; do
  trial_flags+=(-I"$trial_ros/include/$package")
done
timeout 180 g++ "${trial_flags[@]}" \
  "$trial_root/benchmarks/plane_ground_truth_20260929/learn.cpp" \
  "$trial_gng/src/cpu/cugng.cpp" "$trial_gng/src/cpu/labelling.cpp" \
  "$trial_gng/src/cpu/voxel_grid.cpp" "$trial_gng/src/utils/node.cpp" \
  "$trial_gng/src/utils/param.cpp" "$trial_gng/src/utils/vec3f.cpp" \
  "$trial_gng/src/utils/utils.cpp" -o "$trial_output/learn"
for mode in before after; do
  trial_source="$trial_root/ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp"
  trial_mode_flags=()
  if [[ "$mode" == before ]]; then
    trial_source="$trial_root/artifacts/plane_merge_simplification_20260929/before.cpp"
    trial_mode_flags=(-I"$trial_root/artifacts/plane_merge_simplification_20260929/before_include")
  fi
  timeout 180 g++ "${trial_mode_flags[@]}" "${trial_flags[@]}" "$trial_source" \
    "$trial_root/benchmarks/plane_ground_truth_20260929/planes.cpp" -o "$trial_output/planes_$mode"
done

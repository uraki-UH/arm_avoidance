#!/usr/bin/env bash
set -euo pipefail
trial_mode=${1:-before}
trial_root=/ros2_ws/src/artifacts/surface_priority_20260929
trial_bench=/ros2_ws/src/benchmarks/surface_priority_20260929
if [[ "$trial_mode" == before ]]; then
  trial_include="$trial_root/before/include"
  trial_source="$trial_root/before/src"
elif [[ "$trial_mode" == after ]]; then
  trial_include=/ros2_ws/src/ais_gng_cpu/src/ais_gng/include
  trial_source=/ros2_ws/src/ais_gng_cpu/src/ais_gng/src/topological_plane
else
  exit 2
fi
mkdir -p "$trial_root/$trial_mode/bin"
trial_flags=(-std=c++17 -O3 -DNDEBUG -I"$trial_include" -I/usr/include/eigen3 -I/ros2_ws/install/ais_gng_msgs/include/ais_gng_msgs)
if [[ "$trial_mode" == after ]]; then trial_flags+=(-DSURFACE_PRIORITY); fi
for trial_package in geometry_msgs std_msgs builtin_interfaces rosidl_runtime_cpp rosidl_runtime_c rosidl_typesupport_interface rcutils; do
  trial_flags+=(-I"/opt/ros/humble/include/$trial_package")
done
trial_objects=()
for trial_unit in patch_curvature surface_model surface_model_support surface_model_tracking surface_model_local; do
  trial_objects+=("$trial_root/$trial_mode/bin/$trial_unit.o")
  if [[ ${2:-} != --measure-only ]]; then
    timeout 120 g++ "${trial_flags[@]}" -c "$trial_source/$trial_unit.cpp" -o "$trial_root/$trial_mode/bin/$trial_unit.o"
  fi
done
timeout 120 g++ "${trial_flags[@]}" "$trial_bench/measure.cpp" "${trial_objects[@]}" -o "$trial_root/$trial_mode/bin/measure"

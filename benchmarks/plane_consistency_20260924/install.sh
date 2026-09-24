#!/usr/bin/env bash
set -euo pipefail
trial_root=/ros2_ws/src/artifacts/plane_consistency_20260924
trial_files=(libplane_cluster_incremental.so libais_gng_component_cpu.so plane_cluster_incremental_node)
trial_backup=${1:-runtime_before}
case "$trial_backup" in runtime_before|scale_runtime_before|contact_runtime_before|fragment_runtime_before|direction_runtime_before|absorption_runtime_before|simple_runtime_before) ;; *) exit 2 ;; esac
test ! -e "$trial_root/$trial_backup"
mkdir "$trial_root/$trial_backup"
for file in "${trial_files[@]}"; do
  test -f "$trial_root/ros_build/$file"
  cp -p "/ros2_ws/build/ais_gng/$file" "$trial_root/$trial_backup/$file"
  cp -p "$trial_root/ros_build/$file" "/ros2_ws/build/ais_gng/$file.plane_consistency_new"
done
# 既存プロセスのマッピングを保持するinode単位の差し替え。
for file in "${trial_files[@]}"; do
  mv "/ros2_ws/build/ais_gng/$file.plane_consistency_new" "/ros2_ws/build/ais_gng/$file"
  cmp "$trial_root/ros_build/$file" "/ros2_ws/build/ais_gng/$file"
done

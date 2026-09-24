#!/usr/bin/env bash
set -euo pipefail
trial_script=/ros2_ws/src/benchmarks/gng_incremental_updates_20260924/run_case.sh
methods=("$@")
for trial in 1 2 3; do
  frames=100
  if [ "$trial" = 1 ]; then frames=300; fi
  for voxel in 0.1 0.5; do
    for (( idx=0; idx<${#methods[@]}; idx++ )); do
      selected=$idx
      if [ "$trial" = 2 ]; then selected=$(( ${#methods[@]} - idx - 1 )); fi
      method=${methods[$selected]}
      bash "$trial_script" "$method" "$voxel" "$frames" 50 "voxel_${voxel}_$trial"
    done
  done
done
for method in "${methods[@]}"; do
  for voxel in 0 0.1 0.5; do
    bash "$trial_script" "$method" "$voxel" 30 5 "features_$voxel" --features
  done
done

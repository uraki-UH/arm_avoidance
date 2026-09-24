#!/usr/bin/env bash
set -euo pipefail
trial_script=/ros2_ws/src/benchmarks/gng_followup_efficiency_20260924/run_case.sh
for voxel in 0.1 0.5; do
  for method in "$@"; do
    bash "$trial_script" "$method" "$voxel" 100 50 "probe_$voxel"
  done
done

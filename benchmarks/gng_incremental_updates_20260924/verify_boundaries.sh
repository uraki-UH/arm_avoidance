#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_artifacts=/ros2_ws/src/artifacts/gng_incremental_updates_20260924
for method in sync orphan edges combined pooled; do
  timeout 180 cmake --build "/tmp/gng_incremental_updates_build/$method" --target incremental_updates_test -j 3 > "$trial_artifacts/$method/boundary_build.log" 2>&1
  timeout 60 "/tmp/gng_incremental_updates_build/$method/incremental_updates_test" > "$trial_artifacts/$method/boundary_test.log" 2>&1
  echo "$method boundary comparison passed"
done
bash /ros2_ws/src/benchmarks/gng_incremental_updates_20260924/sanitize.sh pooled

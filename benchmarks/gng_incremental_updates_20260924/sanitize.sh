#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_artifacts=/ros2_ws/src/artifacts/gng_incremental_updates_20260924
trial_method=${1:-combined}
trial_build=/tmp/gng_incremental_updates_sanitize_$trial_method
timeout 120 cmake -S "$trial_artifacts/${trial_method}_source" -B "$trial_build" \
  -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_AUTHENTICATION=OFF -DGNG_BUILD_BENCHMARKS=ON \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -g -DNDEBUG -fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined' > "$trial_artifacts/sanitize_${trial_method}_configure.log" 2>&1
timeout 600 cmake --build "$trial_build" --target incremental_updates_test -j 3 > "$trial_artifacts/sanitize_${trial_method}_build.log" 2>&1
timeout 120 "$trial_build/incremental_updates_test" > "$trial_artifacts/sanitize_${trial_method}_test.log" 2>&1
echo 'ASan and UBSan comparison passed'

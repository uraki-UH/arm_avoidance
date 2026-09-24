#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
cd /ros2_ws/src
trial_source=artifacts/gng_followup_efficiency_20260924/edges_source
trial_output=artifacts/gng_followup_efficiency_20260924/edges
trial_build=/tmp/gng_followup_edges_build
timeout 120 cmake -S "$trial_source" -B "$trial_build" -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_AUTHENTICATION=OFF -DGNG_ENABLE_FRAME_LOG=ON -DGNG_BUILD_BENCHMARKS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > "$trial_output/configure.log" 2>&1
timeout 600 cmake --build "$trial_build" -j 2 > "$trial_output/build.log" 2>&1
timeout 180 ctest --test-dir "$trial_build" --no-tests=error --output-on-failure > "$trial_output/tests.log" 2>&1
timeout 30 "$trial_build/gng_training_event_api_test" > "$trial_output/training_events.log" 2>&1
timeout 30 "$trial_build/gng_map_delta_api_test" > "$trial_output/map_delta.log" 2>&1
timeout 30 "$trial_build/sparse_edges_test" > "$trial_output/sparse_edges.log" 2>&1
cp "$trial_build/libgng_cpu.so" "$trial_build/CMakeCache.txt" "$trial_build/compile_commands.json" "$trial_output/"
# 実際の次数上限・ID再利用・寿命境界を含むメモリ検査。
timeout 180 g++ -std=c++20 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -DGNG_VERSION=0     -I "$trial_source/src" -I "$trial_source/include"     "$trial_source/test/sparse_edges_test.cpp" "$trial_source/test/edge_dense_reference/cugng.cpp"     "$trial_source/src/cpu/cugng.cpp" "$trial_source/src/utils/node.cpp" "$trial_source/src/utils/param.cpp"     "$trial_source/src/utils/vec3f.cpp" "$trial_source/src/utils/utils.cpp"     -o "$trial_build/sparse_edges_sanitizer" > "$trial_output/sanitizer_build.log" 2>&1
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 timeout 120 "$trial_build/sparse_edges_sanitizer"     > "$trial_output/sanitizer.log" 2>&1
# 共通CPU実装を利用するwasmラッパーの、既存C++17ネイティブ試験。
timeout 180 g++ -std=c++17 -O3 -DGNG_VERSION=0     -I gng_web_tools/wasm/include -I "$trial_source/include" -I "$trial_source/src/cpu" -I "$trial_source/src/utils"     gng_web_tools/wasm/src/gng_kernel.cpp gng_web_tools/wasm/src/wasm_exports.cpp     gng_web_tools/wasm/test/gng_wasm_core_cpu_kernel_test.cpp     "$trial_source/src/cpu/cugng.cpp" "$trial_source/src/utils/node.cpp" "$trial_source/src/utils/param.cpp"     "$trial_source/src/utils/vec3f.cpp" "$trial_source/src/utils/utils.cpp"     -o "$trial_build/sparse_edges_wasm_native" > "$trial_output/wasm_native_build.log" 2>&1
timeout 30 "$trial_build/sparse_edges_wasm_native" > "$trial_output/wasm_native.log" 2>&1
cat "$trial_output/sparse_edges.log" "$trial_output/sanitizer.log" "$trial_output/wasm_native.log"

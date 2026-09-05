#!/usr/bin/env bash
set -eo pipefail
output_dir=${1:?出力ディレクトリが必要です}
# コンテナ内の独立ビルドと計測。既存install/buildへの変更なし。
source /opt/ros/humble/setup.bash
set -u
workspace_dir=/ros2_ws/src
# 固定乱数は比較用コピーだけへの適用。本体ソースと通常ビルドからの分離。
python3 - "$workspace_dir" <<'PY_COPY'
from pathlib import Path
import shutil
import sys
workspace = Path(sys.argv[1])
source = Path("/tmp/node_support_current_source")
shutil.copytree(workspace / "ais_gng_cpu/src/gng_cpu", source, dirs_exist_ok=True)
for directory in (Path("/tmp/node_support_baseline/ais_gng_cpu/src/gng_cpu"), source):
    path = directory / "src/cpu/cugng.cpp"
    code = path.read_text().replace("mt19937 mt(rnd());", "mt19937 mt(42U + frame_number);")
    path.write_text(code)
PY_COPY
baseline_dir=/tmp/node_support_baseline/ais_gng_cpu/src/gng_cpu
cmake -S "$baseline_dir" -B /tmp/node_support_baseline_build \
    -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_FRAME_LOG=OFF
cmake --build /tmp/node_support_baseline_build -j4
cmake -S /tmp/node_support_current_source -B /tmp/node_support_current_build \
    -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_FRAME_LOG=OFF -DGNG_BUILD_BENCHMARKS=ON
cmake --build /tmp/node_support_current_build -j4
for mode in baseline current; do
    build_dir=/tmp/node_support_current_build
    flags=()
    if [[ "$mode" == baseline ]]; then
        build_dir=/tmp/node_support_baseline_build
        flags+=(-Denable_baseline_build "$workspace_dir/ais_gng_cpu/src/gng_cpu/src/utils/vec3f.cpp")
    fi
    g++ -std=c++17 -O3 -DNDEBUG \
        -I"$workspace_dir/ais_gng_cpu/src/ais_gng/include" \
        -I"$workspace_dir/ais_gng_cpu/src/gng_cpu/include" -I/usr/include/eigen3 \
        "${flags[@]}" "$workspace_dir/ais_gng_cpu/src/ais_gng/test/benchmark_node_support.cpp" \
        -L"$build_dir" -Wl,-rpath,"$build_dir" -lgng_cpu \
        -o "/tmp/benchmark_node_support_$mode"
done
LD_LIBRARY_PATH="/tmp/node_support_current_build:${LD_LIBRARY_PATH:-}" /tmp/node_support_current_build/gng_training_event_api_test

# 順序効果を抑える交互実行。ウォームアップ除外は集計側。
for repeat in 0 1 2 3 4; do
    modes=(baseline support)
    if (( repeat % 2 )); then modes=(support baseline); fi
    for mode in "${modes[@]}"; do
        executable=/tmp/benchmark_node_support_current
        library_dir=/tmp/node_support_current_build
        if [[ "$mode" == baseline ]]; then
            executable=/tmp/benchmark_node_support_baseline
            library_dir=/tmp/node_support_baseline_build
        fi
        LD_LIBRARY_PATH="$library_dir:${LD_LIBRARY_PATH:-}" "$executable" "$output_dir/input.bin" "$output_dir/${mode}_${repeat}.csv" "$mode"
    done
done
for mode in capture2 cumulative first; do
    LD_LIBRARY_PATH="/tmp/node_support_current_build:${LD_LIBRARY_PATH:-}" /tmp/benchmark_node_support_current "$output_dir/input.bin" "$output_dir/${mode}.csv" "$mode"
done
# 記録I/Oの計測は通常ベンチマークから分離。
LD_LIBRARY_PATH="/tmp/node_support_current_build:${LD_LIBRARY_PATH:-}" /tmp/benchmark_node_support_current "$output_dir/input.bin" "$output_dir/diagnostic.csv" support "$output_dir/diagnostic"

#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
trial_root=/ros2_ws/src
trial_artifacts=$trial_root/artifacts/gng_normal_efficiency_20260924
# 一致検証が全件成功した集計の存在確認。
test -f "$trial_artifacts/summary.json"
timeout 120 cmake -S "$trial_root/ais_gng_cpu/src/gng_cpu" -B /ros2_ws/build/gng_cpu \
  -DCMAKE_BUILD_TYPE=Release -DGNG_ENABLE_AUTHENTICATION=OFF -DGNG_ENABLE_FRAME_LOG=ON \
  -DGNG_BUILD_BENCHMARKS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  > "$trial_artifacts/production_configure.log" 2>&1
timeout 600 cmake --build /ros2_ws/build/gng_cpu -j 4 > "$trial_artifacts/production_build.log" 2>&1
timeout 180 ctest --test-dir /ros2_ws/build/gng_cpu --no-tests=error --output-on-failure > "$trial_artifacts/production_tests.log" 2>&1
timeout 30 /ros2_ws/build/gng_cpu/gng_training_event_api_test > "$trial_artifacts/production_training_events.log" 2>&1
timeout 30 /ros2_ws/build/gng_cpu/gng_map_delta_api_test > "$trial_artifacts/production_map_delta.log" 2>&1
python3 - <<'PY'
from pathlib import Path
import hashlib
import json
import os
import shutil

output = Path('/ros2_ws/src/artifacts/gng_normal_efficiency_20260924')
summary = json.loads((output / 'summary.json').read_text())
assert summary['compared_frames'] == 1410
for case in summary['comparisons']:
    assert all(num == case['frames'] for num in case['matches'].values())
source = Path('/ros2_ws/build/gng_cpu/libgng_cpu.so')
installed = Path('/ros2_ws/install/gng_cpu/lib/libgng_cpu.so')
assert not installed.is_symlink()
before_hash = hashlib.sha256(installed.read_bytes()).hexdigest()
assert before_hash == (output / 'production_before.txt').read_text().split()[0], 'installed library changed externally'
shutil.copy2(installed, output / 'production_before.so')
# 稼働中プロセスの参照inodeを保つ、同一ディレクトリ内の原子的な差替え。
staged = installed.with_name('.libgng_cpu_normal_20260924.so')
assert not staged.exists()
shutil.copy2(source, staged)
before_inode = installed.stat().st_ino
os.replace(staged, installed)
after_hash = hashlib.sha256(installed.read_bytes()).hexdigest()
assert after_hash == hashlib.sha256(source.read_bytes()).hexdigest()
assert installed.stat().st_ino != before_inode
(output / 'install.json').write_text(json.dumps(dict(before_sha256=before_hash, after_sha256=after_hash,
    before_inode=before_inode, after_inode=installed.stat().st_ino, installed=str(installed)), indent=2) + '\n')
print('production library atomically installed; existing loaded library inode retained')
PY
timeout 180 taskset -c 0 python3 "$trial_root/benchmarks/gng_normal_efficiency_20260924/benchmark.py" \
  --library /ros2_ws/install/gng_cpu/lib/libgng_cpu.so \
  --config "$trial_root/benchmarks/gng_runtime_trials_20260924/at128_voxel_0_1.yaml" \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --output "$trial_artifacts/installed_smoke.json" --frames 30 --warmup 5 --voxel 0.1 --features \
  > "$trial_artifacts/installed_smoke.log" 2>&1
echo 'normal production build, tests, atomic install and real-input smoke passed'

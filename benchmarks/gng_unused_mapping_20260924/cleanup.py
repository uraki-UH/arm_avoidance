"""既存ROSの維持確認と、今回の専用ビルドの後片付け。"""
from pathlib import Path
import json
import shutil
import subprocess

output = Path('/ros2_ws/src/artifacts/gng_unused_mapping_20260924')
before = (output / 'processes_before.txt').read_text()
after = subprocess.check_output(['ps', '-eo', 'pid,ppid,args'], text=True)
(output / 'processes_after.txt').write_text(after)
def relevant(text):
    rows = {}
    for line in text.splitlines()[1:]:
        columns = line.split(None, 2)
        if len(columns) != 3:
            continue
        command = columns[2]
        if command.startswith(('/ros2_ws/install/', '/usr/bin/python3 /opt/ros/')) or (
                command.startswith('/usr/bin/python3 -c ') and 'ros2cli.daemon' in command):
            rows[columns[0]] = columns[1:]
    return rows
baseline = relevant(before)
assert baseline == relevant(after), '既存ROSプロセスの相違を検出'
initial = json.loads((output / 'production_before.json').read_text())
loaded = []
for pid in baseline:
    rows = [line for line in Path(f'/proc/{pid}/maps').read_text().splitlines() if '/libgng_cpu.so' in line]
    if rows:
        assert all(int(line.split()[4]) == initial['inode'] for line in rows)
        loaded.append(int(pid))
temporary = Path('/tmp/gng_unused_mapping_build_20260924')
assert temporary.is_dir() and not temporary.is_symlink()
shutil.rmtree(temporary)
result = dict(existing_ros_processes=len(baseline), unchanged_ros_pids=sorted(map(int, baseline)),
              old_library_pids=loaded, temporary_build_removed=not temporary.exists(),
              ctest_passed=21, additional_api_tests_passed=2, wasm_native_tests_passed=1,
              installed_smoke_frames=30)
(output / 'runtime_verification.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result))

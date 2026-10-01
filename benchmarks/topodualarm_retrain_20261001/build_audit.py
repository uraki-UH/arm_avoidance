"""既存Releaseビルドの設定を再利用した監査専用ツールの構築。"""
from pathlib import Path
import shlex
import subprocess
import sys

root = Path('/ros2_ws/build/gng_vlut_system')
output = Path(sys.argv[1]) if len(sys.argv) >= 2 else Path('/ros2_ws/src/artifacts/topodualarm_retrain_20261001/audit')
source = Path(sys.argv[2]) if len(sys.argv) >= 3 else Path(__file__).with_name('audit.cpp')
flags = {}
for line in (root / 'CMakeFiles/test_reachability_collision.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        name, value = line.split(' = ', 1)
        flags[name] = shlex.split(value)
command = ['/usr/bin/c++', *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
           str(source), '-o', str(output)]
links = shlex.split((root / 'CMakeFiles/test_reachability_collision.dir/link.txt').read_text())
start = next(idx for idx, value in enumerate(links) if value.startswith('-Wl,-rpath'))
for value in links[start:]:
    if value.startswith('gtest/'):
        continue
    command.append(str(root / value) if value.startswith('src/') else value)
command.extend(['-Wl,--start-group', *[str(root / f'src/lib{name}.a') for name in
    ('safety_engine', 'collision_lib', 'robot_model_lib', 'kinematics_lib', 'common_lib')],
    '-Wl,--end-group', '-lfcl', '-lccd', '-loctomap', '-loctomath'])
output.with_suffix('.build_command.txt').write_text(shlex.join(command) + '\n')
subprocess.run(command, check=True, cwd=root, timeout=180)

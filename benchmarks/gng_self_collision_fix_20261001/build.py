"""既存 ROS ビルドのコンパイル設定を使う独立ツールの有限ビルド。"""
import argparse
from pathlib import Path
import shlex
import subprocess

root = Path('/ros2_ws/build/gng_vlut_system')
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
source = Path(__file__).resolve().with_name('safe_graph.cpp')
args.output.parent.mkdir(parents=True, exist_ok=True)
flags = {}
for line in (root / 'CMakeFiles/test_reachability_collision.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        name, value = line.split(' = ', 1)
        flags[name] = shlex.split(value)
command = ['/usr/bin/c++', *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'],
           *flags['CXX_FLAGS'], '-Wall', '-Wextra', '-pthread', str(source),
           '-o', str(args.output)]
links = shlex.split((root / 'CMakeFiles/test_reachability_collision.dir/link.txt').read_text())
start = next(idx for idx, value in enumerate(links) if value.startswith('-Wl,-rpath'))
for value in links[start:]:
    if value.startswith('gtest/'):
        continue
    if value.startswith('src/'):
        value = str(root / value)
    command.append(value)
command.extend(['-Wl,--start-group', str(root / 'src/libsafety_engine.a'),
                str(root / 'src/libcollision_lib.a'), str(root / 'src/librobot_model_lib.a'),
                str(root / 'src/libkinematics_lib.a'), str(root / 'src/libcommon_lib.a'),
                '-Wl,--end-group', '-lfcl', '-lccd', '-loctomap', '-loctomath'])
args.output.with_suffix('.build_command.txt').write_text(shlex.join(command) + '\n')
subprocess.run(command, check=True, cwd=root, timeout=180)

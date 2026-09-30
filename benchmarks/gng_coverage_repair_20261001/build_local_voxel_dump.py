"""既存ROSビルド設定を利用する、低優先度・単一コンパイラの有限ビルド。"""
import json
from pathlib import Path
import shlex
import subprocess
import time

build_root = Path('/ros2_ws/build/gng_vlut_system')
folder = Path(__file__).resolve().parent
flags = {}
for line in (build_root / 'CMakeFiles/test_reachability_collision.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        name, value = line.split(' = ', 1)
        flags[name] = shlex.split(value)
command = ['nice', '-n', '19', '/usr/bin/c++', *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'],
           *flags['CXX_FLAGS'], '-O1', '-Wall', '-Wextra', str(folder / 'dump_local_voxels.cpp'),
           '-o', str(folder / 'dump_local_voxels')]
links = shlex.split((build_root / 'CMakeFiles/test_reachability_collision.dir/link.txt').read_text())
start = next(idx for idx, value in enumerate(links) if value.startswith('-Wl,-rpath'))
for value in links[start:]:
    if value.startswith('gtest/'):
        continue
    if value.startswith('src/'):
        value = str(build_root / value)
    command.append(value)
command.extend(['-Wl,--start-group', str(build_root / 'src/libsafety_engine.a'),
                str(build_root / 'src/libcollision_lib.a'), str(build_root / 'src/librobot_model_lib.a'),
                str(build_root / 'src/libkinematics_lib.a'), str(build_root / 'src/libcommon_lib.a'),
                '-Wl,--end-group', '-lfcl', '-lccd', '-loctomap', '-loctomath'])
started = time.monotonic()
result = subprocess.run(command, cwd=build_root, timeout=170)
(folder / 'local_voxel_build.json').write_text(json.dumps({
    'argv': command, 'returncode': result.returncode, 'elapsed_sec': time.monotonic() - started}, indent=2) + '\n')
result.check_returncode()

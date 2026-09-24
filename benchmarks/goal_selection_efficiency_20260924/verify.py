"""選択参照表の回帰試験に対するASan・UBSan検証。"""
from pathlib import Path
import os
import shlex
import subprocess
root=Path('/ros2_ws/src')
out=root/'artifacts/goal_selection_efficiency_20260924'
work=Path('/tmp/goal_selection_efficiency_20260924')
flags_path=Path('/ros2_ws/build/gng_vlut_system/CMakeFiles/test_grasp_candidate_reachability.dir/flags.make')
includes=shlex.split(next(x.split(' = ',1)[1] for x in flags_path.read_text().splitlines() if x.startswith('CXX_INCLUDES = ')))
command=['c++','-std=c++17','-O1','-g0','-fsanitize=address,undefined','-fno-omit-frame-pointer',*includes,'-I'+str(root/'bsp3d/include'),'-I'+str(root/'gng_vlut_system/src'),str(root/'gng_vlut_system/test/test_grasp_candidate_reachability.cpp'),'/ros2_ws/build/gng_vlut_system/gtest/libgtest_main.a','/ros2_ws/build/gng_vlut_system/gtest/libgtest.a','-pthread','-o',str(work/'selection_sanitized')]
(out/'sanitizer_command.txt').write_text(shlex.join(command)+'\n')
with (out/'sanitizer_build.log').open('w') as log:
    result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=240)
if result.returncode: raise RuntimeError((out/'sanitizer_build.log').read_text()[-4000:])
env=dict(os.environ,ASAN_OPTIONS='detect_leaks=1',UBSAN_OPTIONS='halt_on_error=1')
result=subprocess.run([str(work/'selection_sanitized')],capture_output=True,text=True,timeout=90,env=env)
(out/'sanitizer_test.log').write_text(result.stdout+result.stderr)
print(result.stdout[-1200:]+result.stderr,flush=True)
if result.returncode: raise RuntimeError('Sanitizer regression failed')

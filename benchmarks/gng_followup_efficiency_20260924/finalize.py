"""既存プロセス・配布物・全比較条件の確認と、一時ビルドの後片付け。"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

benchmark = Path(__file__).resolve().parent
root = benchmark.parent.parent
output = root/'artifacts'/benchmark.name
subprocess.run([sys.executable,str(benchmark/'summarize.py')],check=True,stdout=subprocess.DEVNULL)
subprocess.run([sys.executable,str(benchmark/'audit.py'),'--methods','attention_ids','attention_spans',
                'search','cluster','edges','combined','combined_xyz'],check=True)
subprocess.run([sys.executable,str(benchmark/'report.py')],check=True,stdout=subprocess.DEVNULL)
def runtime(text):
    result = {}
    for line in text.splitlines():
        match = re.match(r'\s*(\d+)\s+(\d+)\s+(.+)',line)
        if not match:
            continue
        pid,parent,command = match.groups()
        if command.startswith('/usr/bin/python3 /opt/ros/humble/bin/ros2 ') or command.startswith('/ros2_ws/install/'):
            result[pid] = dict(parent=parent,command=command)
    return result
before = runtime((output/'processes_before.txt').read_text())
after_text = subprocess.check_output(['ps','-eo','pid,ppid,args'],text=True)
(output/'processes_after.txt').write_text(after_text)
after = runtime(after_text)
assert before == after, (before,after)
assert len(after) == 9
for line in after_text.splitlines():
    assert not re.search(r' (?:taskset|cmake --build|ctest|cc1plus|g\+\+).*gng_followup',line), line
    assert not re.search(r' python3 .*/gng_followup_efficiency_20260924/benchmark.py',line), line
install = json.loads((output/'install.json').read_text())
installed = Path('/ros2_ws/install/gng_cpu/lib/libgng_cpu.so')
assert hashlib.sha256(installed.read_bytes()).hexdigest() == install['after_sha256']
smoke = json.loads((output/'installed_smoke.json').read_text())
assert len(smoke['records']) == 30 and smoke['has_features']
assert all(row['nodes'] > 0 for row in smoke['records'])
assert '100% tests passed, 0 tests failed out of 20' in (output/'production_tests.log').read_text()
assert '30 packages finished' in (output/'cb.log').read_text()
assert '100% tests passed, 0 tests failed out of 1' in (output/'wasm_production_tests.log').read_text()
source = root/'ais_gng_cpu/src/gng_cpu'
expected = json.loads((output/'production_source_sha256.json').read_text())
for name,digest in expected.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == digest, name
mapped = {}
for pid,data in after.items():
    if data['command'].startswith('/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu '):
        lines = [line for line in Path(f'/proc/{pid}/maps').read_text().splitlines() if 'libgng_cpu.so' in line]
        assert lines and all(int(line.split()[4]) == install['before_inode'] for line in lines), lines
        mapped[pid] = lines
removed = []
for name in ('gng_followup_efficiency_build','gng_followup_search_build','gng_followup_cluster_build',
             'gng_followup_cluster_reference_build','gng_followup_edges_build','gng_followup_attention_test_build'):
    path = Path('/tmp')/name
    if path.exists():
        caches = list(path.rglob('CMakeCache.txt'))
        assert caches and all('artifacts/gng_followup_efficiency_20260924/' in p.read_text() for p in caches), path
        shutil.rmtree(path)
        removed.append(str(path))
verification = dict(runtime_before=before,runtime_after=after,existing_gng_mappings=mapped,
    removed_temporary_builds=removed,production_ctest_num=20,production_api_test_num=2,
    wasm_native_test_num=1,installed_smoke_frames=30,cb_packages=30,cb_sec=56.1,
    installed_sha256=install['after_sha256'],all_owned_processes_finished=True)
(output/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
for name in ('summary.json','audit.json','report.json','install.json','adoption.json','verification.json',
             'production_source_sha256.json'):
    shutil.copy2(output/name,benchmark/name)
owner = root.stat()
for path in [benchmark,*benchmark.rglob('*')]:
    os.chown(path,owner.st_uid,owner.st_gid)
subprocess.run(['git','-c',f'safe.directory={root}','diff','--check'],cwd=root,check=True)
print('final audit passed: existing ROS processes unchanged; owned test processes finished')

"""配布結果・既存プロセス・検証終了の確認と、今回の一時ビルドだけの片付け。"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

trial = Path(__file__).resolve().parent
root = trial.parents[1]
output = root/'artifacts'/trial.name
subprocess.run([sys.executable,str(trial/'audit.py'),'--methods','sync','orphan','edges','combined','pooled'], check=True)
subprocess.run([sys.executable,str(trial/'report.py')], check=True)
def runtime(text):
    result = {}
    for line in text.splitlines():
        match = re.match(r'\s*(\d+)\s+(\d+)\s+(.+)', line)
        if match:
            pid,parent,command = match.groups()
            if command.startswith('/usr/bin/python3 /opt/ros/humble/bin/ros2 ') or command.startswith('/ros2_ws/install/'):
                result[pid] = dict(parent=parent, command=command)
    return result
before = runtime((output/'processes_before.txt').read_text())
after_text = subprocess.check_output(['ps','-eo','pid,ppid,args'],text=True)
after = runtime(after_text)
(output/'processes_after.txt').write_text(after_text)
assert before == after, (before,after)
for line in after_text.splitlines():
    command = line.split(None,2)[-1]
    assert not re.search(r'^(?:timeout .* )?(?:taskset .* )?python3 .*benchmark.py.*gng_incremental_updates',command), line
    assert not re.search(r'^(?:/usr/bin/)?(?:cmake --build|ctest|g\+\+|cc1plus).*gng_incremental_updates',command), line
install = json.loads((output/'install.json').read_text())
installed = Path('/ros2_ws/install/gng_cpu/lib/libgng_cpu.so')
assert hashlib.sha256(installed.read_bytes()).hexdigest() == install['after_sha256']
smoke = json.loads((output/'installed_smoke.json').read_text())
assert len(smoke['records']) == 30 and smoke['has_features']
assert all(row['nodes'] > 0 for row in smoke['records'])
assert '100% tests passed, 0 tests failed out of 21' in (output/'production_tests.log').read_text()
assert '30 packages finished' in (output/'cb.log').read_text()
assert '100% tests passed, 0 tests failed out of 1' in (output/'wasm_production_tests.log').read_text()
assert 'incremental_updates_test=passed comparisons=3696' in (output/'sanitize_pooled_test.log').read_text()
source = root/'ais_gng_cpu/src/gng_cpu'
manifest = json.loads((output/'production_source_sha256.json').read_text())
for name,digest in manifest.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == digest, name
mapped = {}
for pid,data in after.items():
    if data['command'].startswith('/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu '):
        lines = [line for line in Path('/proc',pid,'maps').read_text().splitlines() if 'libgng_cpu.so' in line]
        assert lines and all(int(line.split()[4]) == install['before_inode'] for line in lines), lines
        mapped[pid] = lines
removed = []
for name in ('gng_incremental_updates_build','gng_incremental_updates_sanitize','gng_incremental_updates_sanitize_pooled'):
    path = Path('/tmp')/name
    if path.exists():
        caches = list(path.rglob('CMakeCache.txt'))
        assert caches and all('artifacts/gng_incremental_updates_20260924/' in cache.read_text() for cache in caches)
        shutil.rmtree(path)
        removed.append(str(path))
for method in ('before','sync','orphan','edges','combined','pooled'):
    for suffix in ('_source','_deterministic'):
        folder = output/(method+suffix)
        hashes = {str(path.relative_to(folder)): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in sorted(folder.rglob('*')) if path.is_file()}
        (output/(method+suffix+'_sha256.json')).write_text(json.dumps(hashes,indent=2)+'\n')
verification = dict(runtime_before=before,runtime_after=after,existing_gng_mappings=mapped,
    removed_temporary_builds=removed,production_ctest_num=21,production_api_test_num=2,
    wasm_native_test_num=1,installed_smoke_frames=30,cb_packages=30,
    sanitizer_comparisons=3696,installed_sha256=install['after_sha256'],all_owned_processes_finished=True)
(output/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
for name in ('audit.json','report.json','install.json','adoption.json','verification.json','production_source_sha256.json'):
    shutil.copy2(output/name,trial/name)
owner = root.stat()
for path in (trial,*trial.rglob('*')):
    os.chown(path,owner.st_uid,owner.st_gid)
subprocess.run(['git','-c',f'safe.directory={root}','diff','--check'],cwd=root,check=True)
print('runtime unchanged, installation verified, temporary builds cleaned')

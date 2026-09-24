"""公開シンボルと変更前ライブラリを確認した、実行中inodeを保つ原子的差替え。"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

root = Path('/ros2_ws/src')
output = root/'artifacts/gng_followup_efficiency_20260924'
adoption = json.loads((output/'adoption.json').read_text())
audit = json.loads((output/'audit.json').read_text())
assert adoption['method'] in audit['methods']
source = Path('/ros2_ws/build/gng_cpu/libgng_cpu.so')
installed = Path('/ros2_ws/install/gng_cpu/lib/libgng_cpu.so')
before = json.loads((output/'production_before.json').read_text())
assert hashlib.sha256(installed.read_bytes()).hexdigest() == before['sha256'], '外部でのライブラリ更新を検出'
assert not installed.is_symlink()
def symbols(path):
    text = subprocess.check_output(['nm','-D','--defined-only',str(path)], text=True)
    return sorted(line.split()[-1] for line in text.splitlines() if line.split())
assert symbols(installed) == symbols(source), '公開シンボルの変化を検出'
assert not any('efficiency' in name or 'attention_phase' in name for name in symbols(source))
shutil.copy2(installed, output/'production_before.so')
staged = installed.with_name('.libgng_cpu_followup_20260924.so')
assert not staged.exists()
shutil.copy2(source, staged)
before_inode = installed.stat().st_ino
os.replace(staged, installed)
after_hash = hashlib.sha256(installed.read_bytes()).hexdigest()
assert after_hash == hashlib.sha256(source.read_bytes()).hexdigest()
assert installed.stat().st_ino != before_inode
(output/'install.json').write_text(json.dumps(dict(method=adoption['method'],before_sha256=before['sha256'],
    after_sha256=after_hash,before_inode=before_inode,after_inode=installed.stat().st_ino,
    exported_symbols=symbols(installed)),indent=2)+'\n')
print('production library atomically installed; existing loaded inode retained')

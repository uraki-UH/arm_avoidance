"""検証済み本番ライブラリの原子的な反映。実行中inodeの維持。"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess

root = Path('/ros2_ws/src')
output = root / 'artifacts/gng_unused_mapping_20260924'
report = json.loads((output / 'report.json').read_text())
assert report['compared_frames'] == 840
source = root / 'ais_gng_cpu/src/gng_cpu'
manifest = json.loads((output / 'after_source_sha256.json').read_text())
assert all(hashlib.sha256((source / name).read_bytes()).hexdigest() == digest
           for name, digest in manifest.items()), '比較後のソース変更を検出'
installed = Path('/ros2_ws/install/gng_cpu/lib/libgng_cpu.so')
built = Path('/ros2_ws/build/gng_cpu/libgng_cpu.so')
before = json.loads((output / 'production_before.json').read_text())
assert not installed.is_symlink()
assert hashlib.sha256(installed.read_bytes()).hexdigest() == before['sha256'], '配布版の更新を検出'
def symbols(path):
    text = subprocess.check_output(['nm', '-D', '--defined-only', str(path)], text=True)
    return sorted(line.split()[-1] for line in text.splitlines() if line.split())
assert symbols(installed) == symbols(built), '公開シンボルの変化を検出'
assert not any('efficiency' in name for name in symbols(built))
staged = installed.with_name('.libgng_cpu_unused_mapping_20260924.so')
assert not staged.exists()
shutil.copy2(built, staged)
os.replace(staged, installed)
assert installed.stat().st_ino != before['inode']
after_hash = hashlib.sha256(installed.read_bytes()).hexdigest()
assert after_hash == hashlib.sha256(built.read_bytes()).hexdigest()
result = dict(before=before, after_sha256=after_hash, after_inode=installed.stat().st_ino,
              exported_symbols=symbols(installed))
(output / 'install.json').write_text(json.dumps(result, indent=2) + '\n')
print('production library atomically installed')

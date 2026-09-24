"""既存プロセスとインストール済みライブラリの検証前記録。"""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

root = Path('/ros2_ws/src/artifacts/gng_incremental_updates_20260924')
processes = subprocess.check_output(['ps', '-eo', 'pid,ppid,args'], text=True)
rows = [line for line in processes.splitlines() if any(word in line for word in
        ('/opt/ros/', '/ros2_ws/install/', '_ros2_daemon'))]
(root / 'processes_before.txt').write_text('\n'.join(rows) + '\n')
library = Path('/ros2_ws/install/gng_cpu/lib/libgng_cpu.so')
(root / 'production_before.json').write_text(json.dumps({
    'sha256': hashlib.sha256(library.read_bytes()).hexdigest(), 'inode': library.stat().st_ino}, indent=2) + '\n')
shutil.copy2(library, root / 'production_before.so')
print(f'existing runtime processes: {len(rows)}')

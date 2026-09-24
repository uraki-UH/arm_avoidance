"""固定コミットからの独立比較コピーと、検証前状態の保存。"""
from pathlib import Path
import hashlib
import io
import tarfile
import json
import subprocess

root = Path(__file__).resolve().parents[2]
trial = Path(__file__).resolve().parent
output = root / 'artifacts' / trial.name
output.mkdir(parents=True, exist_ok=False)
(output / 'COLCON_IGNORE').touch()
revision_path = trial / 'base_revision.txt'
revision = revision_path.read_text().strip()
prefix = 'ais_gng_cpu/src/gng_cpu/'
archive = subprocess.check_output(['git', '-c', f'safe.directory={root}', 'archive', revision, prefix.rstrip('/')], cwd=root)
for method in ('before', 'sync', 'orphan', 'edges', 'combined'):
    source = output / (method + '_source')
    source.mkdir()
    with tarfile.open(fileobj=io.BytesIO(archive)) as entries:
        for entry in entries:
            if not entry.isfile():
                continue
            relative = Path(entry.name[len(prefix):])
            assert entry.name.startswith(prefix) and not relative.is_absolute() and '..' not in relative.parts
            target = source / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(entries.extractfile(entry).read())
manifest = {str(path.relative_to(output / 'before_source')): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted((output / 'before_source').rglob('*')) if path.is_file()}
(output / 'before_sha256.json').write_text(json.dumps(manifest, indent=2) + '\n')
processes = subprocess.check_output(['ps', '-eo', 'pid,ppid,args'], text=True)
(output / 'host_processes_before.txt').write_text('\n'.join(line for line in processes.splitlines()
    if any(word in line for word in ('/opt/ros/', '/ros2_ws/install/', '_ros2_daemon'))) + '\n')
(output / 'containers_before.txt').write_text(subprocess.check_output(
    ['docker', 'ps', '--format', '{{.ID}} {{.Names}}'], text=True))
print('comparison sources and host baseline saved')

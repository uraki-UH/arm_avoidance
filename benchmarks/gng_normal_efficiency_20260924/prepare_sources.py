"""記録した基準コミットからの比較ソース展開。既存保存先の上書き禁止。"""
import io
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

root = Path('/ros2_ws/src')
benchmark = Path(__file__).resolve().parent
output = root/'artifacts/gng_normal_efficiency_20260924'
output.mkdir(parents=True, exist_ok=True)
(output/'COLCON_IGNORE').touch()
revision = (benchmark/'base_revision.txt').read_text().strip()
source = output/'before_source'
assert not source.exists(), 'comparison source already exists'
source.mkdir()
prefix = 'ais_gng_cpu/src/gng_cpu/'
archive = subprocess.check_output(['git', 'archive', revision, prefix.rstrip('/')], cwd=root)
with tarfile.open(fileobj=io.BytesIO(archive)) as entries:
    for entry in entries:
        if not entry.isfile():
            continue
        assert entry.name.startswith(prefix)
        relative = Path(entry.name[len(prefix):])
        assert not relative.is_absolute() and '..' not in relative.parts
        target = source/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(entries.extractfile(entry).read())
for method, variant in [('after', 'packed'), ('reuse', 'reuse')]:
    target = output/(method+'_source')
    shutil.copytree(source, target)
    subprocess.run([sys.executable, str(benchmark/'optimize.py'), str(target), '--variant', variant], check=True)

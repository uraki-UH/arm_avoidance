"""固定コミットからの独立比較ソース展開。既存保存先の上書き禁止。"""
import argparse
import hashlib
import io
import json
import subprocess
import sys
import tarfile
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--methods', nargs='+', default=['before', 'attention_ids', 'attention_spans'])
args = parser.parse_args()
benchmark = Path(__file__).resolve().parent
root = benchmark.parent.parent
output = root / 'artifacts' / benchmark.name
output.mkdir(parents=True, exist_ok=True)
(output / 'COLCON_IGNORE').touch()
revision = (benchmark / 'base_revision.txt').read_text().strip()
prefix = 'ais_gng_cpu/src/gng_cpu/'
archive = subprocess.check_output(['git', '-c', f'safe.directory={root}', 'archive', revision, prefix.rstrip('/')], cwd=root)
for method in args.methods:
    source = output / (method + '_source')
    assert not source.exists(), source
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
    if method.startswith('attention_'):
        subprocess.run([sys.executable, str(benchmark/'optimize_attention.py'), str(source),
                        '--variant', method.removeprefix('attention_')], check=True)
    elif method in ('search', 'cluster', 'edges'):
        subprocess.run([sys.executable, str(benchmark/f'optimize_{method}.py'), str(source)], check=True)
    manifest = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(source.rglob('*')) if p.is_file()}
    (output / (method + '_original_sha256.json')).write_text(json.dumps(manifest, indent=2)+'\n')

"""全条件照合済みの候補を、変更前ソース確認後に本番へ適用する手順。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--method', choices=('combined', 'combined_xyz'), required=True)
args = parser.parse_args()
benchmark = Path(__file__).resolve().parent
root = benchmark.parent.parent
output = root/'artifacts'/benchmark.name
source = root/'ais_gng_cpu/src/gng_cpu'
reference = output/'before_source'
audit = json.loads((output/'audit.json').read_text())
assert args.method in audit['methods'], '合成版の全条件照合が必要'
for path in reference.rglob('*'):
    if path.is_file():
        assert path.read_bytes() == (source/path.relative_to(reference)).read_bytes(), path
scripts = []
if args.method == 'combined':
    scripts.append(['optimize_attention.py', '--variant', 'ids'])
scripts += [[f'optimize_{name}.py'] for name in ('search','cluster','edges')]
for script in scripts:
    subprocess.run([sys.executable, str(benchmark/script[0]), str(source), *script[1:]], check=True)
subprocess.run([sys.executable, str(benchmark/'register_attention_test.py'), str(source),
                '--variant', 'ids' if args.method == 'combined' else 'before'], check=True)
selected = output/(args.method+'_source')
for directory in ('src','include'):
    for path in (selected/directory).rglob('*'):
        if path.is_file():
            assert path.read_bytes() == (source/path.relative_to(selected)).read_bytes(), path
owner = root.stat()
for path in source.rglob('*'):
    if not (reference/path.relative_to(source)).exists():
        os.chown(path, owner.st_uid, owner.st_gid)
manifest = {str(path.relative_to(source)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(source.rglob('*')) if path.is_file()}
(output/'production_source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
(output/'adoption.json').write_text(json.dumps(dict(method=args.method, source=str(source)),indent=2)+'\n')
print('validated source applied:', args.method)

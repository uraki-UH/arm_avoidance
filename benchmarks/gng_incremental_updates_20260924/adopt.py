"""全条件照合済み候補の、事前ソース確認付き適用。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
parser = argparse.ArgumentParser()
parser.add_argument('--method', choices=('sync','orphan','edges','combined','pooled'), required=True)
args = parser.parse_args()
trial = Path(__file__).resolve().parent
root = trial.parents[1]
output = root/'artifacts'/trial.name
source = root/'ais_gng_cpu/src/gng_cpu'
before = output/'before_source'
audit = json.loads((output/'audit.json').read_text())
assert args.method in audit['methods']
for path in before.rglob('*'):
    if path.is_file():
        assert path.read_bytes() == (source/path.relative_to(before)).read_bytes(), path
subprocess.run([sys.executable, str(trial/'optimize.py'), str(source), '--method', 'combined' if args.method == 'pooled' else args.method], check=True)
if args.method == 'pooled':
    subprocess.run([sys.executable, str(trial/'optimize_pool.py'), str(source)], check=True)
subprocess.run([sys.executable, str(trial/'register_test.py'), str(source)], check=True)
selected = output/(args.method+'_source')
for path in selected.rglob('*'):
    if path.is_file():
        assert path.read_bytes() == (source/path.relative_to(selected)).read_bytes(), path
owner = root.stat()
os.chown(source/'test/incremental_updates_test.cpp', owner.st_uid, owner.st_gid)
manifest = {str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(source.rglob('*')) if path.is_file()}
(output/'production_source_sha256.json').write_text(json.dumps(manifest, indent=2)+'\n')
(output/'adoption.json').write_text(json.dumps({'method': args.method}, indent=2)+'\n')
print('validated source applied:', args.method)

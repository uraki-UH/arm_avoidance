"""既存の同値性・性能検証手順の再利用。生成スクリプトはartifacts内に限定。"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess

root = Path('/ros2_ws/src')
output = root / 'artifacts/gng_search_readability_20260924'
bench = root / 'benchmarks/gng_search_readability_20260924'
driver = output / 'driver'
parser = argparse.ArgumentParser()
parser.add_argument('phase', choices=('build', 'measure', 'finish', 'cleanup'))
args = parser.parse_args()

def run(name, timeout_sec):
    command = ['bash' if name.endswith('.sh') else 'python3', str(driver / name)]
    subprocess.run(command, check=True, timeout=timeout_sec)

if args.phase == 'build':
    driver.mkdir(exist_ok=False)
    previous = root / 'benchmarks/gng_unused_mapping_20260924'
    manifest = {}
    for name in ('prepare.py', 'build.sh', 'run.sh', 'report.py', 'install.py', 'validate.sh', 'cleanup.py'):
        source = (previous / name).read_text()
        manifest[name] = hashlib.sha256(source.encode()).hexdigest()
        text = source.replace('gng_unused_mapping', 'gng_search_readability')
        text = text.replace('benchmarks/gng_search_readability_20260924/',
                            'artifacts/gng_search_readability_20260924/driver/')
        if name == 'cleanup.py':
            text = text.replace("initial = json.loads((output / 'production_before.json').read_text())",
                "initial_maps = json.loads((output / 'loaded_maps_before.json').read_text())")
            text = text.replace("assert all(int(line.split()[4]) == initial['inode'] for line in rows)",
                "assert [row.split()[:5] for row in rows] == [row.split()[:5] for row in initial_maps[pid]]")
        if name == 'report.py':
            text = text.replace('root = Path(__file__).resolve().parents[2]', "root = Path('/ros2_ws/src')")
        (driver / name).write_text(text)
    (bench / 'reused_scripts_sha256.json').write_text(json.dumps(manifest, indent=2) + '\n')
    installed = Path('/ros2_ws/install/gng_cpu/lib/libgng_cpu.so')
    shutil.copy2(installed, output / 'production_before.so')
    (output / 'production_before.json').write_text(json.dumps(dict(
        sha256=hashlib.sha256(installed.read_bytes()).hexdigest(), inode=installed.stat().st_ino,
        is_symlink=installed.is_symlink())) + '\n')
    loaded_maps = {}
    for line in (output / 'processes_before.txt').read_text().splitlines()[1:]:
        columns = line.split(None, 2)
        if len(columns) == 3 and columns[2].startswith(('/ros2_ws/install/', '/usr/bin/python3 /opt/ros/')):
            pid = columns[0]
            loaded_maps[pid] = [row for row in Path(f'/proc/{pid}/maps').read_text().splitlines()
                                if '/libgng_cpu.so' in row]
    (output / 'loaded_maps_before.json').write_text(json.dumps(loaded_maps, indent=2) + '\n')
    run('prepare.py', 60)
    run('build.sh', 1200)
elif args.phase == 'measure':
    run('run.sh', 1800)
    run('report.py', 60)
    shutil.copy2(output / 'report.json', bench / 'report.json')
else:
    if args.phase == 'finish':
        run('validate.sh', 600)
    run('cleanup.py', 60)
    for name in ('before_source_sha256.json', 'after_source_sha256.json', 'install.json', 'runtime_verification.json'):
        shutil.copy2(output / name, bench / name)
print(args.phase + ' complete', flush=True)

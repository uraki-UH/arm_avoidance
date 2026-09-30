"""開始時プロセスの維持と、所有試験の残留の最終確認。停止操作なし。"""
import argparse
import json
import os
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('folder', type=Path)
args = parser.parse_args()
baseline = json.loads((args.folder / 'runtime_baseline.json').read_text())
identity = json.loads((args.folder / 'runtime_baseline_identity.json').read_text())
previous = {item['pid']: item for item in identity['processes']}
def process_info(pid):
    path = Path('/proc') / str(pid)
    fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
    argv = (path / 'cmdline').read_bytes().split(b'\0')
    argv = [value.decode(errors='replace') for value in argv if value]
    return {'pid': pid, 'ppid': int(fields[1]), 'start_ticks': int(fields[19]), 'argv': argv,
            'command': ' '.join(argv)}
ancestors = set()
pid = os.getpid()
while pid > 0 and pid not in ancestors:
    ancestors.add(pid)
    pid = process_info(pid)['ppid']
current = {}
for path in Path('/proc').iterdir():
    if not path.name.isdigit():
        continue
    try:
        item = process_info(int(path.name))
        current[item['pid']] = item
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        pass
changed = []
for item in baseline['processes']:
    actual = current.get(item['pid'])
    if actual is None or actual['command'] != item['command'].strip() or actual['start_ticks'] != previous[item['pid']]['start_ticks']:
        changed.append({'before': item, 'after': actual})
markers = ('gng_coverage_repair_20261001', 'coverage_max_2cm', 'coverage_long_2cm')
owned = []
new_ros = []
for pid, item in current.items():
    if pid in ancestors or pid in previous:
        continue
    if any(marker in item['command'] for marker in markers):
        owned.append(item)
    argv = item['argv']
    if any(value == '/opt/ros/humble/bin/ros2' or value.startswith('/ros2_ws/install/') for value in argv[:2]):
        new_ros.append(item)
containers = subprocess.check_output(['docker', 'ps', '--format', '{{.ID}} {{.Names}}'], text=True).splitlines()
result = {'num_baseline_processes': len(previous), 'num_preserved_processes': len(previous) - len(changed),
          'changed_baseline_processes': changed, 'remaining_owned_processes': owned, 'new_unowned_ros_processes': new_ros,
          'scope': '所有試験プロセスの終了と開始時プロセスの維持。並行作業の新規ROSは停止対象外。',
          'containers_before': baseline['containers'], 'containers_after': containers,
          'has_passed': not changed and not owned and sorted(containers) == sorted(baseline['containers'])}
(args.folder / 'runtime_final.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
print(json.dumps(result, ensure_ascii=False))
assert result['has_passed']

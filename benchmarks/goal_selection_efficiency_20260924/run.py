"""同一CPUの同一プロセス比較と試行ごとのログ保存。"""
from pathlib import Path
import json
import subprocess

root = Path('/ros2_ws/src')
out = root / 'artifacts/goal_selection_efficiency_20260924'
fixtures = root / 'artifacts/bsp3d_migration_20260924'
rows = []
initial = []
for trial in range(3):
    for fixture in (['points', 'planar', 'robot'] if trial % 2 == 0 else ['robot', 'planar', 'points']):
        cmd = ['taskset', '-c', '0', '/tmp/goal_selection_efficiency_20260924/benchmark', str((out if fixture=='robot' else fixtures)/(fixture+'.txt')), str(trial)]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
        (out/f'{fixture}_{trial}.log').write_text(result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError(result.stdout[-1000:]+result.stderr)
        for line in result.stdout.splitlines():
            if line.startswith(('result ', 'initial ')):
                fields = dict(part.split('=',1) for part in line.split()[1:])
                target = rows if line.startswith('result ') else initial
                target.append(dict(fixture=fixture, trial=trial, **fields))
        print(fixture, trial, result.stdout.splitlines()[-1], flush=True)
        (out/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
        (out/'initial.json').write_text(json.dumps(initial,indent=2)+'\n')

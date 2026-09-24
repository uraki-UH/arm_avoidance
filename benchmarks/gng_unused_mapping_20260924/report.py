"""フレーム出力照合と、各試行平均の中央値による比較。"""
from pathlib import Path
import hashlib
import json
import statistics

root = Path(__file__).resolve().parents[2]
output = root / 'artifacts/gng_unused_mapping_20260924'
cases = [f'v{voxel}_r{trial}' for trial in (1, 2, 3) for voxel in (0.1, 0.5)]
cases += ['voxel_disabled'] + [f'features_{voxel}' for voxel in (0, 0.1, 0.5)]
compared_num = 0
compared_fields = set()
results = {}
for name in cases:
    before = json.loads((output / 'before' / (name + '.json')).read_text())
    after = json.loads((output / 'after' / (name + '.json')).read_text())
    assert len(before['records']) == len(after['records'])
    assert before['accepted'] == after['accepted']
    for first, second in zip(before['records'], after['records']):
        assert set(first) == set(second)
        for field in first:
            if field.endswith('_ms'):
                continue
            assert first[field] == second[field], (name, first['frame'], field)
            compared_fields.add(field)
        compared_num += 1
    results[name] = dict(frames=len(before['records']), warmup=before['warmup'],
                         before=before['mean'], after=after['mean'])
median = {}
for voxel in (0.1, 0.5):
    trials = [results[f'v{voxel}_r{trial}'] for trial in (1, 2, 3)]
    median[str(voxel)] = {
        method: {field: statistics.median(trial[method][field] for trial in trials)
                 for field in trials[0][method]}
        for method in ('before', 'after')}
report = dict(compared_frames=compared_num, total_frames=compared_num * 2,
              compared_fields=sorted(compared_fields), cases=results, median=median)
serialized = json.dumps(report, indent=2) + '\n'
(output / 'report.json').write_text(serialized)
Path(__file__).with_name('report.json').write_text(serialized)
for method in ('before', 'after'):
    source = output / (method + '_source')
    manifest = json.loads((output / (method + '_source_sha256.json')).read_text())
    assert all(hashlib.sha256((source / name).read_bytes()).hexdigest() == digest
               for name, digest in manifest.items())
print(json.dumps(dict(compared_frames=compared_num, median=median), indent=2))

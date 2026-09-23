from pathlib import Path
import json
import statistics

root = Path('/ros2_ws/src/artifacts/gng_production_efficiency_20260924')
comparisons = []
for path in sorted((root / 'before').glob('*.json')):
    if path.name == 'compile_commands.json':
        continue
    after_path = root / 'after' / path.name
    if not after_path.exists():
        raise RuntimeError(f'missing after result: {after_path}')
    before = json.loads(path.read_text())
    after = json.loads(after_path.read_text())
    assert len(before['records']) == len(after['records'])
    keys = [key for key in before['records'][0] if key.endswith('_sha256') or key.endswith('_num')
            or key in ('nodes', 'edges', 'clusters')]
    matches = {key: sum(x[key] == y[key] for x, y in zip(before['records'], after['records'])) for key in keys}
    assert all(value == len(before['records']) for value in matches.values()), (path.name, matches)
    assert all(row['learning_num'] == 4000 for row in before['records'] + after['records'])
    comparisons.append(dict(case=path.stem, frames=len(before['records']), matches=matches,
                            before_mean=before['mean'], after_mean=after['mean']))
results = {}
for voxel in (0.1, 0.5):
    trials = []
    for trial in (1, 2, 3):
        before = json.loads((root / 'before' / f'voxel_{voxel}_{trial}.json').read_text())
        after = json.loads((root / 'after' / f'voxel_{voxel}_{trial}.json').read_text())
        # 全試行の同一フレーム区間による、負荷変動の反復確認。
        start_idx, end_idx = 50, 100
        entry = dict(trial=trial)
        for method, data in (('before', before), ('after', after)):
            entry[method] = {name: statistics.mean(row[name] for row in data['records'][start_idx:end_idx])
                             for name in data['mean']}
        trials.append(entry)
    medians = {method: statistics.median(trial[method]['total_ms'] for trial in trials)
               for method in ('before', 'after')}
    before = json.loads((root / 'before' / f'voxel_{voxel}_1.json').read_text())
    after = json.loads((root / 'after' / f'voxel_{voxel}_1.json').read_text())
    results[str(voxel)] = dict(trials=trials, median_total_ms=medians,
        median_reduction_percent=100 * (1 - medians['after'] / medians['before']),
        long_run={method: data['mean'] for method, data in (('before', before), ('after', after))},
        first_frame_ms={method: data['records'][0]['total_ms'] for method, data in (('before', before), ('after', after))})
summary = dict(comparison_frames=sum(item['frames'] for item in comparisons),
               executed_frames=2*sum(item['frames'] for item in comparisons),
               comparisons=comparisons, timings=results)
(root / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))

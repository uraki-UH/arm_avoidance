from pathlib import Path
import json
import statistics

import numpy as np

root = Path('/ros2_ws/src/artifacts/gng_radix_production_20260924')
comparisons = []
for path in sorted((root / 'boost').glob('*.json')):
    if path.name == 'compile_commands.json':
        continue
    boost = json.loads(path.read_text())
    radix = json.loads((root / 'radix' / path.name).read_text())
    assert len(boost['records']) == len(radix['records'])
    required = ('input_num', 'kept_input_num', 'voxel_num', 'learning_num',
                'cell_membership_sha256', 'cell_sequence_sha256', 'ranges_sha256')
    for before, after in zip(boost['records'], radix['records']):
        assert all(before[key] == after[key] for key in required), (path.name, before['frame'])
        assert before['learning_num'] == after['learning_num'] == 4000
    keys = [key for key in boost['records'][0] if key.endswith('_sha256')]
    matches = {key: sum(before.get(key) == after.get(key) for before, after in zip(boost['records'], radix['records'])
                        if key in before) for key in keys}
    totals = {key: sum(key in row for row in boost['records']) for key in keys}
    first_difference = {key: next((row['frame'] for row, other in zip(boost['records'], radix['records'])
                                    if key in row and row[key] != other[key]), None) for key in keys}
    if boost['voxel'] == 0:
        assert matches == totals, (path.name, matches, totals)
    comparisons.append(dict(case=path.stem, frames=len(boost['records']), matches=matches, totals=totals,
                            first_difference=first_difference))
results = {}
for voxel in (0.1, 0.5):
    trials = []
    for trial in (1, 2, 3):
        entry = dict(trial=trial)
        for method in ('boost', 'radix'):
            data = json.loads((root / method / f'voxel_{voxel}_{trial}.json').read_text())
            entry[method] = {name: statistics.mean(row[name] for row in data['records'][50:100])
                             for name in data['mean']}
        trials.append(entry)
    data = {method: json.loads((root / method / f'voxel_{voxel}_1.json').read_text()) for method in ('boost', 'radix')}
    means = {method: value['mean'] for method, value in data.items()}
    medians = {method: {name: statistics.median(trial[method][name] for trial in trials)
                       for name in means[method]} for method in ('boost', 'radix')}
    quality = {}
    for method, value in data.items():
        rows = [row['quality'] for row in value['records'][50:] if 'quality' in row]
        quality[method] = {name: statistics.mean(row[name] for row in rows) for name in rows[0]}
    quality_delta = {}
    pairs = [(before['quality'], after['quality']) for before, after in zip(data['boost']['records'], data['radix']['records'])
             if before['frame'] >= 50 and 'quality' in before]
    for name in ('coverage_0_2', 'coverage_0_4', 'mean_dist', 'p95_dist'):
        differences = [after[name] - before[name] for before, after in pairs]
        quality_delta[name] = dict(mean=statistics.mean(differences), min=min(differences), max=max(differences))
    first = {method: np.load(root / method / f'voxel_{voxel}_1_frame_0.npz') for method in ('boost', 'radix')}
    assert np.array_equal(first['boost']['ranges'], first['radix']['ranges'])
    centroid_delta = first['radix']['centroids'].astype(np.float64) - first['boost']['centroids']
    learn_delta = first['radix']['learning_inputs'].astype(np.float64) - first['boost']['learning_inputs']
    centroid_dist = np.linalg.norm(centroid_delta, axis=1)
    learn_dist = np.linalg.norm(learn_delta, axis=1)
    snapshots = dict(centroid_max_component_m=float(np.abs(centroid_delta).max()),
        centroid_max_dist_m=float(centroid_dist.max()), changed_centroids=int(np.count_nonzero(centroid_dist)),
        reordered_raw_indices=int(np.count_nonzero(first['boost']['indices'][:, 1] != first['radix']['indices'][:, 1])),
        learning_point_max_dist_m=float(learn_dist.max()), learning_point_mean_dist_m=float(learn_dist.mean()),
        changed_learning_points=int(np.count_nonzero(learn_dist)),
        learning_points_over_1mm=int(np.count_nonzero(learn_dist > 0.001)))
    graph_size = {method: {key: statistics.mean(row[key] for row in value['records'][50:])
                          for key in ('nodes', 'edges', 'clusters', 'attention_num')}
                  for method, value in data.items()}
    results[str(voxel)] = dict(long_run=means, trials=trials, median=medians, quality=quality,
        quality_delta=quality_delta, first_frame=snapshots, graph_size=graph_size,
        total_reduction_percent=100*(1-means['radix']['total_ms']/means['boost']['total_ms']),
        sort_reduction_percent=100*(1-means['radix']['sort_ms']/means['boost']['sort_ms']))
summary = dict(compared_frames=sum(item['frames'] for item in comparisons),
               executed_frames=2*sum(item['frames'] for item in comparisons),
               comparisons=comparisons, results=results)
(root / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))

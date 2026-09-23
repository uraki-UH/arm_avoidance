"""同一入力・乱数列による出力一致と、段階時間の比較。"""
import json
import statistics
from pathlib import Path

root = Path('/ros2_ws/src/artifacts/gng_normal_efficiency_20260924')
comparisons = []

def compare(method, name):
    before = json.loads((root/'before'/f'{name}.json').read_text())
    after = json.loads((root/method/f'{name}.json').read_text())
    assert len(before['records']) == len(after['records'])
    keys = [key for key in before['records'][0] if key.endswith('_sha256') or key in
            ('nodes', 'edges', 'clusters', 'voxel_num', 'kept_input_num', 'attention_num', 'learning_num', 'events_num')]
    matches = {key: sum(a[key] == b[key] for a,b in zip(before['records'], after['records'])) for key in keys}
    assert all(row['learning_num'] == 4000 for run in (before,after) for row in run['records'])
    assert all(num == len(before['records']) for num in matches.values()), (name, matches)
    result = dict(method=method, name=name, frames=len(before['records']), matches=matches,
                  before=before['mean'], after=after['mean'])
    comparisons.append(result)
    return result

for voxel in ('0.1', '0.5'):
    compare('after', f'probe_{voxel}')
    compare('reuse', f'probe_{voxel}')
    for trial in (1,2,3):
        compare('reuse', f'voxel_{voxel}_{trial}')
for voxel in ('0','0.1','0.5'):
    compare('reuse', f'features_{voxel}')
conditions = {}
for voxel in ('0.1','0.5'):
    first = next(row for row in comparisons if row['name'] == f'voxel_{voxel}_1')
    repeated = {}
    for metric in ('total_ms','label_ms','normal_ms','rho_label_ms'):
        trials = {}
        for method in ('before','reuse'):
            trials[method] = [statistics.mean(row[metric] for row in json.loads(
                (root/method/f'voxel_{voxel}_{trial}.json').read_text())['records'][50:100]) for trial in (1,2,3)]
        repeated[metric] = dict(trials=trials, before_median=statistics.median(trials['before']),
            after_median=statistics.median(trials['reuse']))
    conditions[voxel] = dict(before=first['before'], after=first['after'], repeated=repeated)
result = dict(compared_frames=sum(row['frames'] for row in comparisons), executed_frames=2660,
              comparisons=comparisons, conditions=conditions)
(root/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
for voxel, condition in conditions.items():
    print(voxel)
    for key in ('total_ms','label_ms','normal_ms','rho_label_ms'):
        before,after=condition['before'][key],condition['after'][key]
        repeated=condition['repeated'][key]
        print(key,round(before,4),'->',round(after,4), 'reduction',round((1-after/before)*100,2),
            'medians',round(repeated['before_median'],4),round(repeated['after_median'],4))
print('compared_frames',result['compared_frames'],'all outputs matched')

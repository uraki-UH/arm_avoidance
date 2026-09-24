"""同じ50フレーム区間の3試行中央値による、候補別の比較表。"""
import json
from pathlib import Path
import statistics

root = Path(__file__).resolve().parents[2]
output = root/'artifacts/gng_incremental_updates_20260924'
metrics = ('total_ms','matching_ms','candidates_ms','mapping_sort_ms','learn_ms','label_ms','maintenance_ms','cluster_ms')
result = {}
for method in ('before','sync','orphan','edges','combined','before_pool','pooled'):
    result[method] = {}
    for voxel in ('0.1','0.5'):
        paths = [output/method/f'voxel_{voxel}_{trial}.json' for trial in (1,2,3)]
        if not all(path.exists() for path in paths):
            continue
        runs = [json.loads(path.read_text()) for path in paths]
        medians = {metric:statistics.median(statistics.mean(row[metric] for row in run['records'][50:100])
                                          for run in runs) for metric in metrics}
        result[method][voxel] = dict(first_300_mean=runs[0]['mean'],common_window_median=medians,
                                    init_rss_kib=runs[0]['init_rss_kib'],peak_rss_kib=runs[0]['peak_rss_kib'])
        print(method,voxel,' '.join(f'{metric}={value:.3f}' for metric,value in medians.items()))
(output/'report.json').write_text(json.dumps(result,indent=2)+'\n')

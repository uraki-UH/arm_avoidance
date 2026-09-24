"""固定入力比較の全出力照合と段階時間・メモリの集計。"""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--methods', nargs='+', default=['attention_ids', 'attention_spans', 'search', 'cluster', 'edges', 'combined', 'combined_xyz'])
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
output = root / 'artifacts/gng_followup_efficiency_20260924'
comparisons = []
for method in args.methods:
    for path in sorted((output/method).glob('*.json')):
        before_path = output/'before'/path.name
        if not before_path.exists():
            continue
        before, after = (json.loads(p.read_text()) for p in (before_path, path))
        if 'records' not in before or 'records' not in after:
            continue
        assert before['accepted'] == after['accepted'], (method, path.name, 'parameters')
        assert len(before['records']) == len(after['records'])
        keys = [key for key in before['records'][0] if key.endswith('_sha256') or key in
                ('nodes', 'edges', 'clusters', 'voxel_num', 'kept_input_num', 'attention_num', 'learning_num', 'events_num')]
        matches = {key: sum(a[key] == b[key] for a,b in zip(before['records'], after['records'])) for key in keys}
        differences = [{key: [a[key],b[key]] for key in keys if a[key] != b[key]} | {'frame':a['frame']}
                       for a,b in zip(before['records'],after['records']) if any(a[key]!=b[key] for key in keys)]
        assert all(row['learning_num'] == 4000 for run in (before,after) for row in run['records'])
        comparisons.append(dict(method=method, name=path.stem, frames=len(before['records']),
            matches=matches, differences=differences[:5], before=before['mean'], after=after['mean'],
            before_init_rss_kib=before['init_rss_kib'], after_init_rss_kib=after['init_rss_kib'],
            before_init_ms=before['init_ms'], after_init_ms=after['init_ms']))
result = dict(compared_frames=sum(row['frames'] for row in comparisons), comparisons=comparisons,
              all_match=all(not row['differences'] for row in comparisons))
(output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
for row in comparisons:
    b,a=row['before'],row['after']
    print(row['method'],row['name'], 'matched' if not row['differences'] else 'MISMATCH',
          'total',round(b['total_ms'],3),'->',round(a['total_ms'],3),
          'attention',round(b['attention_ms'],3),'->',round(a['attention_ms'],3),
          'cluster',round(b['cluster_ms'],3),'->',round(a['cluster_ms'],3))
assert comparisons, 'no comparisons'
assert result['all_match'], [(r['method'],r['name'],r['differences']) for r in comparisons if r['differences']]

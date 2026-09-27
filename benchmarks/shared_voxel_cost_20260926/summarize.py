"""独立試行の集計。CPU時間と経過時間、計測用と配布済みの分離。"""
from collections import defaultdict
import json
import os
from pathlib import Path

root = Path(__file__).resolve().parents[2]
variant = os.environ.get('SHARED_COST_VARIANT', '')
output = root / 'artifacts/shared_voxel_cost_20260926' / variant
groups = defaultdict(list)
for filename in ('results.json', 'installed.json', 'threads24.json', 'arena2_threads24.json',
                 'moving_installed.json', 'moving_baseline_installed.json', 'before_common_installed.json'):
    path = output / filename
    if variant and filename == 'moving_baseline_installed.json':
        path = root / 'artifacts/shared_voxel_cost_20260926/moving_installed.json'
    if variant.startswith('common') and filename == 'before_common_installed.json':
        path = root / 'artifacts/shared_voxel_cost_20260926/optimized/before_common_installed.json'
    if not path.exists():
        continue
    for result in json.loads(path.read_text()):
        key = (filename, result['points'], result['mode'], result['narrow_fvg'], result['wide_roi'])
        groups[key].append(result)

summaries = []
for (filename, points, mode, narrow, wide_roi), results in groups.items():
    installed = filename.endswith('installed.json')
    stages = {}
    sent = sum(r['sent'] for r in results)
    for stage in sorted({name for r in results for name in r['stages']}):
        entries = [r['stages'][stage] for r in results if stage in r['stages']]
        num = sum(item['wall_ms']['num'] for item in entries)
        stages[stage] = {unit + '_mean': sum(e[unit]['mean']*e[unit]['num'] for e in entries)/num
                         for unit in ('wall_ms', 'cpu_ms', 'num_values')}
        stages[stage]['wall_ms_p95_range'] = [min(e['wall_ms']['p95'] for e in entries),
                                            max(e['wall_ms']['p95'] for e in entries)]
        stages[stage]['cpu_ms_per_input'] = sum(e['cpu_ms']['mean']*e['cpu_ms']['num'] for e in entries)/sent
    summary = {'source': filename, 'installed': installed, 'points': points, 'mode': mode, 'narrow_fvg': narrow,
        'wide_roi': wide_roi, 'trials': len(results), 'sent': sent,
        'roi_received': sum(r['received_roi'] for r in results),
        'fvg_received': sum(r['received_fvg'] for r in results),
        'cpu_ms_per_input': sum(r['process_cpu_ms_per_input']*r['sent'] for r in results)/sent,
        'cpu_ms_per_input_range': [min(r['process_cpu_ms_per_input'] for r in results),
                                   max(r['process_cpu_ms_per_input'] for r in results)],
        'peak_rss_mib_range': [min(r['peak_rss_mib'] for r in results), max(r['peak_rss_mib'] for r in results)],
        'marker_mib_sec': sum(r['visual_bytes_per_sec']['markers'] for r in results)/len(results)/2**20,
        'bucket_mib_sec': sum(r['visual_bytes_per_sec']['buckets'] for r in results)/len(results)/2**20,
        'roi_latency_ms_mean': sum(r['roi_latency_ms']['mean'] for r in results)/len(results),
        'fvg_latency_ms_mean': sum(r['fvg_latency_ms'].get('mean', 0.) for r in results)/len(results),
        'stages': stages}
    summaries.append(summary)
    print(f'{installed=} {points=} {mode:7s} {narrow=} {wide_roi=} cpu={summary["cpu_ms_per_input"]:.2f} '
          f'rss={summary["peak_rss_mib_range"]} roi/fvg={summary["roi_received"]}/{summary["fvg_received"]} '
          f'latency={summary["roi_latency_ms_mean"]:.1f}/{summary["fvg_latency_ms_mean"]:.1f}')
    if mode == 'markers' and not installed:
        for name in ('world_total', 'world_build', 'world_roi', 'world_publish', 'fvg_points',
                     'fvg_cells', 'fvg_labels', 'fvg_marker', 'fvg_tmap', 'fvg_timer'):
            value = stages.get(name, {})
            print(f'  {name:14s}: wall={value.get("wall_ms_mean", 0):.3f} '
                  f'cpu/input={value.get("cpu_ms_per_input", 0):.3f} p95={value.get("wall_ms_p95_range")}')
        print('  cells:', stages.get('fvg_state', {}).get('num_values_mean'),
              'MiB/s:', summary['marker_mib_sec'], summary['bucket_mib_sec'])
(Path(__file__).parent / (f'summary_{variant}.json' if variant else 'summary.json')).write_text(json.dumps(summaries, indent=2))

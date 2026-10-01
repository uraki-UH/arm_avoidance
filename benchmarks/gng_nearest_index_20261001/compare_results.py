"""索引の有無による全保存グラフ一致と、同一seedの実時間の比較。"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(data)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--reports', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists() and not args.output.is_symlink()
    root = args.root.resolve()
    source = root / 'benchmarks/voxel_pose_compression_20260930/export_preview.py'
    spec = importlib.util.spec_from_file_location('nearest_export', source)
    export = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(export)

    def local(path):
        value = Path(path)
        if value.is_relative_to('/ros2_ws/src'):
            return root / value.relative_to('/ros2_ws/src')
        return value

    groups = {}
    for report_path in args.reports:
        report = json.loads(report_path.read_text())
        assert report['status'] == 'completed'
        for record in report['records']:
            assert record['returncode'] == 0 and record['cleanup_ok']
            folder = local(record['log']).parent / 'result'
            metrics = json.loads((folder / 'metrics.json').read_text())
            key = (metrics['input'], metrics['profile'], metrics['seed'], metrics['num_angle_iter'],
                   metrics['num_coord_iter_per_layer'], metrics['num_samples'])
            flag = metrics['enable_nearest_index']
            pair = groups.setdefault(key, {})
            assert flag not in pair, '同一条件の重複'
            pair[flag] = (folder, metrics)
    result = {'is_passed': True, 'pairs': [], 'aggregates': [],
              'reports': [str(path) for path in args.reports]}
    aggregates = {}
    for key, pair in sorted(groups.items()):
        assert set(pair) == {False, True}, '線形走査・索引の組不足'
        linear_dir, linear = pair[False]
        indexed_dir, indexed = pair[True]
        sample_hash = digest(linear_dir / 'samples.f32')
        assert sample_hash == digest(indexed_dir / 'samples.f32'), '固定入力の不一致'
        stable_fields = ('num_input_nodes', 'num_output_nodes', 'num_added_nodes', 'num_removed_nodes',
                         'num_moved_existing_nodes', 'num_edges_per_layer', 'max_edge_age', 'lambda',
                         'ais_dist_th', 'n_best_candidates', 'beta', 'alpha', 'learn_rate_s1', 'learn_rate_s2')
        is_metrics_equal = all(linear[name] == indexed[name] for name in stable_fields)
        row = {'input': key[0], 'profile': key[1], 'seed': key[2], 'is_passed': is_metrics_equal,
               'is_stable_metrics_equal': is_metrics_equal,
               'samples_sha256': sample_hash, 'input_sha256': digest(local(key[0])),
               'linear_metrics': linear, 'indexed_metrics': indexed, 'checkpoints': [], 'speedup': {}}
        for checkpoint in ('angle.gng', 'coord0.gng', 'final.gng'):
            first = export.read_gng(linear_dir / checkpoint)
            second = export.read_gng(indexed_dir / checkpoint)
            is_nodes_equal = first['nodes'] == second['nodes']
            edges_equal = [sorted(edge['record'] for edge in lhs) == sorted(edge['record'] for edge in rhs)
                           for lhs, rhs in zip(first['edge_layers'], second['edge_layers'])]
            is_header_equal = first['header'] == second['header']
            is_passed = is_nodes_equal and is_header_equal and all(edges_equal)
            row['is_passed'] = row['is_passed'] and is_passed
            row['checkpoints'].append({'name': checkpoint, 'is_passed': is_passed,
                'is_node_raw_equal': is_nodes_equal, 'is_header_equal': is_header_equal,
                'is_edge_raw_multiset_equal_per_layer': edges_equal,
                'num_nodes': len(first['nodes']), 'num_edges_per_layer': [len(layer) for layer in first['edge_layers']],
                'sha256_linear': digest(linear_dir / checkpoint), 'sha256_indexed': digest(indexed_dir / checkpoint)})
        for name, lhs, rhs in [('total', linear['total_sec'], indexed['total_sec']),
                               ('angle', linear['angle_sec'], indexed['angle_sec']),
                               ('coord_left', linear['coord_sec'][0], indexed['coord_sec'][0]),
                               ('coord_right', linear['coord_sec'][1], indexed['coord_sec'][1])]:
            assert lhs > 0 and rhs > 0
            row['speedup'][name] = lhs / rhs
        result['is_passed'] = result['is_passed'] and row['is_passed']
        result['pairs'].append(row)
        group_key = (key[0], key[1], key[3], key[4], key[5])
        aggregates.setdefault(group_key, []).append(row)
    for key, rows in sorted(aggregates.items()):
        result['aggregates'].append({'input': key[0], 'profile': key[1], 'num_angle_iter': key[2],
            'num_coord_iter_per_layer': key[3], 'num_samples': key[4], 'num_pairs': len(rows),
            'median_speedup': {name: statistics.median(row['speedup'][name] for row in rows)
                               for name in ('total', 'angle', 'coord_left', 'coord_right')},
            'linear_total_sec': [row['linear_metrics']['total_sec'] for row in rows],
            'indexed_total_sec': [row['indexed_metrics']['total_sec'] for row in rows]})
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'is_passed': result['is_passed'], 'num_pairs': len(result['pairs']),
                      'output': str(args.output), 'aggregates': result['aggregates']}, ensure_ascii=False), flush=True)
    assert result['is_passed'], '保存グラフの不一致'


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""逐次workerと並列workerの保存グラフ・候補順・安定指標の完全一致検査。"""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path


stable_top_keys = (
    'collision_method', 'collision_voxel_size_m', 'max_joint_step_rad',
    'manual_exclusions',
)
stable_rebuild_keys = (
    'num_input_nodes', 'num_kept_nodes', 'num_added_witness_nodes',
    'num_output_nodes', 'num_old_edges_discarded',
    'num_unique_candidates_checked', 'num_rejected_edges',
    'num_validated_edges_per_layer', 'num_intermediate_samples',
    'num_fallback_passes', 'num_fallback_added_edges', 'graph',
    'is_roundtrip_verified',
)


def require(is_valid, message):
    if not is_valid:
        raise ValueError(message)


def compare_file_bytes(first_path, second_path):
    """ファイル全体の順序を含む比較と、各入力のSHA-256記録。"""
    first_hash = hashlib.sha256()
    second_hash = hashlib.sha256()
    is_equal = True
    with first_path.open('rb') as first, second_path.open('rb') as second:
        while True:
            first_block = first.read(1024 * 1024)
            second_block = second.read(1024 * 1024)
            if not first_block and not second_block:
                break
            first_hash.update(first_block)
            second_hash.update(second_block)
            is_equal = is_equal and first_block == second_block
    return {
        'is_equal': is_equal,
        'single': {'path': str(first_path), 'num_bytes': first_path.stat().st_size,
                   'sha256': first_hash.hexdigest()},
        'parallel': {'path': str(second_path), 'num_bytes': second_path.stat().st_size,
                     'sha256': second_hash.hexdigest()},
    }


def load_decoder(root):
    path = root / 'benchmarks/voxel_pose_compression_20260930/export_preview.py'
    spec = importlib.util.spec_from_file_location('worker_comparison_export', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def compare_outputs(root, single, parallel):
    first = json.loads((single / 'metrics.json').read_text())
    second = json.loads((parallel / 'metrics.json').read_text())
    require(first['mode'] == second['mode'] == 'rebuild', 'rebuild以外の入力')
    require(first['rebuild']['num_workers'] == 1, '逐次入力のworker数の不一致')
    require(second['rebuild']['num_workers'] > 1, '並列入力のworker数の不一致')
    require(first['rebuild']['is_roundtrip_verified'] is True and
            second['rebuild']['is_roundtrip_verified'] is True,
            '保存・再読込の検証未完了')
    differences = []
    files = {}
    for name in ('gng.bin', 'edge_audit.csv', 'unconnected_nodes.csv'):
        result = compare_file_bytes(single / name, parallel / name)
        files[name] = result
        if not result['is_equal']:
            differences.append(name)
    decoder = load_decoder(root)
    first_graph = decoder.read_gng(single / 'gng.bin')
    second_graph = decoder.read_gng(parallel / 'gng.bin')
    # ノード・辺の保存順を含む生レコード比較
    node_records = [node['record'] for node in first_graph['nodes']]
    other_node_records = [node['record'] for node in second_graph['nodes']]
    edge_records = [[edge['record'] for edge in layer]
                    for layer in first_graph['edge_layers']]
    other_edge_records = [[edge['record'] for edge in layer]
                          for layer in second_graph['edge_layers']]
    raw_checks = {
        'is_header_equal': first_graph['header'] == second_graph['header'],
        'is_node_records_equal': node_records == other_node_records,
        'is_edge_records_equal': edge_records == other_edge_records,
        'single_num_nodes': len(node_records),
        'parallel_num_nodes': len(other_node_records),
        'single_num_edges_per_layer': [len(layer) for layer in edge_records],
        'parallel_num_edges_per_layer': [len(layer) for layer in other_edge_records],
    }
    for key in ('is_header_equal', 'is_node_records_equal', 'is_edge_records_equal'):
        if not raw_checks[key]:
            differences.append('raw_records.' + key)
    metrics = {}
    for prefix, keys in (('', stable_top_keys), ('rebuild', stable_rebuild_keys)):
        first_values = first[prefix] if prefix else first
        second_values = second[prefix] if prefix else second
        for key in keys:
            require(key in first_values and key in second_values,
                    '比較対象指標の欠落: ' + key)
            name = (prefix + '.' if prefix else '') + key
            is_equal = first_values[key] == second_values[key]
            metrics[name] = {'is_equal': is_equal,
                             'single': first_values[key], 'parallel': second_values[key]}
            if not is_equal:
                differences.append(name)
    for name, graph, values in (('single', first_graph, first),
                                ('parallel', second_graph, second)):
        require(len(graph['nodes']) == values['rebuild']['num_output_nodes'],
                name + ': 保存ノード数とmetricsの不一致')
        require(all(len(layer) == values['rebuild']['num_validated_edges_per_layer']
                    for layer in graph['edge_layers']),
                name + ': 保存辺数とmetricsの不一致')
    return {
        'is_equal': not differences, 'differences': differences,
        'num_workers': {'single': first['rebuild']['num_workers'],
                        'parallel': second['rebuild']['num_workers']},
        'files': files, 'raw_records': raw_checks, 'stable_metrics': metrics,
        'excluded_metrics': ['*_sec', 'worker数', 'worker別指標',
                             '衝突判定呼出し数', 'キャッシュ件数・ヒット数'],
        'limit': '同じ入力に対するworker間の結果一致の検査。独立な自己干渉保証は対象外。',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--single', type=Path, required=True)
    parser.add_argument('--parallel', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.is_symlink(),
            '既存比較結果への上書き拒否')
    try:
        result = compare_outputs(args.root.resolve(), args.single.resolve(), args.parallel.resolve())
    except (ValueError, KeyError, OSError, TypeError) as error:
        result = {'is_equal': False, 'error': str(error),
                  'single': str(args.single.resolve()), 'parallel': str(args.parallel.resolve())}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'is_equal': result['is_equal'], 'output': str(args.output.resolve()),
                      'differences': result.get('differences', []), 'error': result.get('error')},
                     ensure_ascii=False))
    return 0 if result['is_equal'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

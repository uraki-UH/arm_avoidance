"""元学習姿勢に対する代表姿勢の位置・関節・接続被覆の評価。"""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stats(values, limits):
    values = np.asarray(values)
    return {'num': int(values.size), 'mean': float(values.mean()),
            'median': float(np.median(values)), 'p95': float(np.quantile(values, .95)),
            'max': float(values.max()),
            'within': {str(limit): float(np.mean(values <= limit)) for limit in limits}}


def paired_tcp_dist(source, targets):
    result = []
    for start in range(0, len(source), 128):
        left = cdist(source[start:start+128, 0], targets[:, 0], metric='sqeuclidean')
        right = cdist(source[start:start+128, 1], targets[:, 1], metric='sqeuclidean')
        # 左右両TCPが同一代表に近いことを要求する最大距離
        result.extend(np.sqrt(np.minimum.reduce(np.maximum(left, right), axis=1)))
    return np.asarray(result)


def graph_stats(model):
    ids = model['node_ids']
    result = []
    for name, edges in zip(('angle', 'left_tcp', 'right_tcp'), model['edge_layers']):
        parent = {key: key for key in ids}
        degree = {key: 0 for key in ids}

        def find(key):
            while parent[key] != key:
                parent[key] = parent[parent[key]]
                key = parent[key]
            return key

        for edge in edges:
            first, second = edge['first_id'], edge['second_id']
            degree[first] += 1
            degree[second] += 1
            parent[find(first)] = find(second)
        sizes = {}
        for key in ids:
            root = find(key)
            sizes[root] = sizes.get(root, 0) + 1
        result.append({'layer': name, 'num_nodes': len(ids), 'num_edges': len(edges),
                       'num_isolated': sum(value == 0 for value in degree.values()),
                       'num_components': len(sizes), 'max_component_nodes': max(sizes.values()),
                       'mean_degree': 2*len(edges)/len(ids)})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--model', choices=('max', 'long'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    start = time.monotonic()
    bench = args.root/'benchmarks/voxel_pose_compression_20260930'
    prepare = load_module('coverage_prepare', bench/'prepare.py')
    export = load_module('coverage_export', bench/'export_preview.py')
    preview = args.root/'artifacts/voxel_viewer_preview_20260930'/args.model
    metadata = json.loads((preview/'preview.json').read_text())
    original = Path(metadata['input_paths']['gng'])
    assignments = Path(metadata['input_paths']['assignments'])
    for name, path in [('gng', original), ('assignments', assignments)]:
        assert hashlib.sha256(path.read_bytes()).hexdigest() == metadata['input_sha256'][name]
    ids, angles, tcp, _ = prepare.read_gng(original)
    angles, tcp = angles.astype(np.float64), tcp.astype(np.float64)
    rows = list(csv.DictReader(assignments.open()))
    representatives = {int(row['original_id']) for row in rows if row['is_representative'] == '1'}
    selected = np.array([int(node_id) in representatives for node_id in ids])
    lookup = {int(node_id): idx for idx, node_id in enumerate(ids)}
    assigned = {int(row['original_id']): int(row['representative_id']) for row in rows}
    assigned_idx = np.array([lookup[assigned[int(node_id)]] for node_id in ids])
    representative_tcp = tcp[selected]
    position_limits = [.01, .02, .04, .08, .1]
    left = cKDTree(representative_tcp[:, 0]).query(tcp[:, 0], workers=1)[0]
    right = cKDTree(representative_tcp[:, 1]).query(tcp[:, 1], workers=1)[0]
    paired = paired_tcp_dist(tcp, representative_tcp)
    joint = np.rad2deg(cKDTree(angles[selected]).query(angles, p=np.inf, workers=1)[0])
    result = {'model': args.model, 'scope': '保存済1万姿勢に対する有限集合の被覆。連続可動域全体の保証なし。',
              'num_original': len(ids), 'num_representatives': int(selected.sum()),
              'input_sha256': metadata['input_sha256'],
              'left_tcp_dist_m': stats(left, position_limits),
              'right_tcp_dist_m': stats(right, position_limits),
              'paired_tcp_dist_m': stats(paired, position_limits),
              'removed_paired_tcp_dist_m': stats(paired[~selected], position_limits),
              'joint_max_abs_diff_deg': stats(joint, [5, 10, 20, 30]),
              'removed_joint_max_abs_diff_deg': stats(joint[~selected], [5, 10, 20, 30]),
              'assigned_tcp_dist_m': stats(np.linalg.norm(tcp-tcp[assigned_idx], axis=2).max(axis=1),position_limits),
              'joint_metric_note': '14関節の最大絶対差を最小化する代表との距離。周期補正なし、手先姿勢角の評価なし。',
              'original_graph': graph_stats(export.read_gng(original)),
              'representative_graph': graph_stats(export.read_gng(preview/'gng.bin'))}
    # 独立な全点対計算による先頭64クエリの探索検査
    for layer, measured in [(0, left), (1, right)]:
        direct = cdist(tcp[:64, layer], representative_tcp[:, layer]).min(axis=1)
        assert np.allclose(measured[:64], direct, atol=1e-12, rtol=1e-12)
    direct_joint = np.rad2deg(cdist(angles[:64], angles[selected], metric='chebyshev').min(axis=1))
    assert np.allclose(joint[:64], direct_joint, atol=1e-10, rtol=1e-12)
    assert np.all(paired >= np.maximum(left, right)-1e-10)
    assert np.all(paired <= np.linalg.norm(tcp-tcp[assigned_idx], axis=2).max(axis=1)+1e-10)
    assert np.max(left[selected]) == np.max(right[selected]) == np.max(paired[selected]) == np.max(joint[selected]) == 0
    result['checks'] = ['探索と全点対計算の一致（先頭64姿勢）', '全代表の自己距離0', '左右同時距離の上下界一致']
    result['elapsed_sec'] = time.monotonic()-start
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()

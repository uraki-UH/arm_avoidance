"""自己干渉修正後の参照準備、2 cm補完選択、GNG・VLUTと被覆の検証。"""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import struct

import numpy as np
from scipy.spatial import cKDTree

expected_collision_method = 'mesh_surface_and_component_voxel_containment'


def require(is_valid, message):
    if not is_valid:
        raise ValueError(message)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save_json(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def write_rows(path, columns, rows):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def pose_columns():
    return ['id'] + [f'q{idx}' for idx in range(14)] + [f'tcp{layer}_{axis}' for layer in range(2)
            for axis in ('x', 'y', 'z')] + ['source_arm', 'source_idx']


def arm_idx(row):
    name = row['source_arm'].lower()
    require(name in ('l', 'left', 'r', 'right'), '参照姿勢の未知の腕名')
    return 0 if name in ('l', 'left') else 1


def points_from_rows(rows):
    return np.asarray([[float(row[f'tcp{layer}_{axis}']) for layer in range(2)
                        for axis in ('x', 'y', 'z')] for row in rows], dtype=np.float32).reshape(-1, 2, 3)


def summarize(values):
    values = np.asarray(values, dtype=np.float64)
    num = len(values)
    if not num:
        return {'num': 0, 'num_covered': 0, 'fraction_covered': None, 'max_m': None, 'mean_m': None}
    return {'num': num, 'num_covered': int(np.count_nonzero(values <= .02)),
            'fraction_covered': float(np.mean(values <= .02)),
            'max_m': float(values.max()) if np.isfinite(values).all() else None,
            'mean_m': float(values.mean()) if np.isfinite(values).all() else None}


def nearest(base_tcp, rows):
    output = np.empty(len(rows), dtype=np.float64)
    tcp = points_from_rows(rows)
    for layer in range(2):
        indices = np.array([idx for idx, row in enumerate(rows) if arm_idx(row) == layer], dtype=np.int64)
        if not len(indices):
            continue
        output[indices] = cKDTree(base_tcp[:, layer]).query(tcp[indices, layer], workers=1)[0] if len(base_tcp) else np.inf
    return output


def coverage_by_arm(base_tcp, rows):
    distances = nearest(base_tcp, rows)
    return [summarize([distances[idx] for idx, row in enumerate(rows) if arm_idx(row) == layer])
            for layer in range(2)], distances


def select_safe_witnesses(base_tcp, rows):
    """初期最遠順の安全証拠選択と、選択姿勢の両TCPによる被覆更新。"""
    before = nearest(base_tcp, rows)
    tcp = points_from_rows(rows)
    indices_by_arm = [np.array([idx for idx, row in enumerate(rows) if arm_idx(row) == layer], dtype=np.int64)
                      for layer in range(2)]
    trees = [cKDTree(tcp[indices, layer]) if len(indices) else None
             for layer, indices in enumerate(indices_by_arm)]
    is_covered = before <= .02
    selected = []
    for idx in sorted(range(len(rows)), key=lambda idx: (-before[idx], arm_idx(rows[idx]), int(rows[idx]['id']))):
        if is_covered[idx]:
            continue
        selected.append(idx)
        for layer in range(2):
            if trees[layer] is None:
                continue
            nearby = trees[layer].query_ball_point(tcp[idx, layer], .02, workers=1)
            actual = indices_by_arm[layer][nearby]
            if len(actual):
                dist = np.linalg.norm(tcp[actual, layer].astype(np.float64) - tcp[idx, layer], axis=1)
                is_covered[actual[dist <= .02]] = True
        require(is_covered[idx], '選択姿勢の自己被覆失敗')
    combined = np.concatenate([base_tcp, tcp[selected]], axis=0) if selected else base_tcp
    after = nearest(combined, rows)
    require(np.all(after <= .02), '安全参照の最終2 cm被覆失敗')
    return selected, before, after


def missing_rows(rows, distances):
    return [{'id': row['id'], 'source_arm': row['source_arm'], 'source_idx': row['source_idx'],
             'dist_m': float(distance) if np.isfinite(distance) else 'inf'}
            for row, distance in zip(rows, distances) if distance > .02]


def prepare(args, old):
    source = old / 'selection/candidates'
    q = np.load(source / 'all_q.npy', allow_pickle=False)
    tcp = np.load(source / 'all_tcp.npy', allow_pickle=False)
    source_arm = np.load(source / 'source_arm.npy', allow_pickle=False)
    source_idx = np.load(source / 'source_idx.npy', allow_pickle=False)
    require(q.shape == (len(tcp), 14) and tcp.shape[1:] == (2, 3), '参照配列の形状不一致')
    require(np.isfinite(q).all() and np.isfinite(tcp).all(), '参照配列の非有限値')
    columns = pose_columns()
    rows = []
    for idx in np.flatnonzero(source_arm >= 0):
        require(source_arm[idx] in (0, 1), '未知の参照腕番号')
        values = [int(idx), *q[idx].tolist(), *tcp[idx].reshape(-1).tolist(),
                  'left' if source_arm[idx] == 0 else 'right', int(source_idx[idx])]
        rows.append(dict(zip(columns, values)))
    args.output.mkdir(parents=True, exist_ok=False)
    write_rows(args.output / 'references.csv', columns, rows)
    inputs = [source / name for name in ('all_q.npy', 'all_tcp.npy', 'source_arm.npy', 'source_idx.npy')]
    save_json(args.output / 'prepare.json', {'model': args.model, 'num_references': len(rows),
        'gng': str(old / 'model/gng.bin'), 'heldout': str(old / 'model/heldout.csv'),
        'input_sha256': {str(path): sha256(path) for path in inputs}})


def select(args, old, export, verify):
    metrics = json.loads((args.audit / 'metrics.json').read_text())
    require(metrics.get('collision_method') == expected_collision_method, '旧方式の監査結果による補完選択')
    require(metrics['mode'] == 'audit' and metrics['nodes']['is_complete'] and metrics['references']['is_complete'],
            '全件完了していない監査結果')
    original = export.read_gng(old / 'model/gng.bin')
    ids, q, tcp = verify.decode_states(original)
    audit_rows = read_rows(args.audit / 'nodes_audit.csv')
    require(len(audit_rows) == len(ids) and {int(row['id']) for row in audit_rows} == set(ids), 'ノード監査のID不一致')
    safe_ids = {int(row['id']) for row in audit_rows if row['is_safe'] == '1'}
    has_safe = np.isin(ids, list(safe_ids))
    references = read_rows(args.audit / 'references_kept.csv')
    selected, before, after = select_safe_witnesses(tcp[has_safe], references)
    additions = []
    for offset, idx in enumerate(selected, start=int(ids.max()) + 1):
        row = dict(references[idx]); row['id'] = offset; additions.append(row)
    args.output.mkdir(parents=True, exist_ok=False)
    write_rows(args.output / 'safe_node_ids.csv', ['id'], [{'id': int(node_id)} for node_id in ids[has_safe]])
    write_rows(args.output / 'extra_nodes.csv', pose_columns(), additions)
    columns = ['id', 'source_arm', 'source_idx', 'dist_m']
    write_rows(args.output / 'missing_before.csv', columns, missing_rows(references, before))
    write_rows(args.output / 'missing_after.csv', columns, missing_rows(references, after))
    save_json(args.output / 'selection.json', {
        'model': args.model, 'audit': str(args.audit.resolve()), 'num_input_nodes': len(ids),
        'num_safe_existing_nodes': len(safe_ids), 'num_unsafe_removed_nodes': len(ids) - len(safe_ids),
        'num_safe_references': len(references), 'num_rejected_references': metrics['references']['num_rejected'],
        'num_added_witnesses': len(additions), 'before': summarize(before), 'after': summarize(after),
        'input_sha256': {str(path): sha256(path) for path in [old / 'model/gng.bin', args.audit / 'metrics.json',
            args.audit / 'nodes_audit.csv', args.audit / 'references_kept.csv']}})


def verify_result(args, old, export, verify):
    audit_metrics = json.loads((args.audit / 'metrics.json').read_text())
    require(audit_metrics['nodes']['is_complete'] and audit_metrics['references']['is_complete'],
            '全件完了していない監査結果')
    if 'heldout' in audit_metrics:
        require(audit_metrics['heldout']['is_complete'], '独立参照監査の未完了')
    rebuild_metrics = json.loads((args.model_dir / 'metrics.json').read_text())
    require(audit_metrics.get('collision_method') == expected_collision_method and
            rebuild_metrics.get('collision_method') == expected_collision_method,
            '監査・再構築に異なる衝突判定方式の混入')
    for key in ('collision_method', 'collision_voxel_size_m', 'manual_exclusions', 'max_joint_step_rad'):
        require(audit_metrics[key] == rebuild_metrics[key], f'監査と再構築の判定条件の不一致: {key}')
    original = export.read_gng(old / 'model/gng.bin')
    final = export.read_gng(args.model_dir / 'gng.bin')
    original_ids, original_q, original_tcp = verify.decode_states(original)
    ids, q, tcp = verify.decode_states(final)
    safe_ids = {int(row['id']) for row in read_rows(args.selection / 'safe_node_ids.csv')}
    extras = read_rows(args.selection / 'extra_nodes.csv')
    require(set(ids) == safe_ids | {int(row['id']) for row in extras}, '出力ノード集合と選択の不一致')
    original_by_id = {int(node_id): idx for idx, node_id in enumerate(original_ids)}
    original_records = {node['id']: node['record'] for node in original['nodes']}
    final_records = {node['id']: node['record'] for node in final['nodes']}
    require(all(final_records[node_id] == original_records[node_id] for node_id in safe_ids),
            '保持ノード生レコードの変更')
    extra_by_id = {int(row['id']): row for row in extras}
    for idx, node_id in enumerate(ids):
        if node_id in safe_ids:
            expected_q = original_q[original_by_id[int(node_id)]]
            expected_tcp = original_tcp[original_by_id[int(node_id)]]
        else:
            row = extra_by_id[int(node_id)]
            expected_q = np.asarray([float(row[f'q{joint_idx}']) for joint_idx in range(14)], dtype=np.float32)
            expected_tcp = points_from_rows([row])[0]
        require(np.array_equal(q[idx], expected_q), '出力関節角の変更')
        require(np.max(np.abs(tcp[idx] - expected_tcp)) <= 2e-5, '出力TCPの不一致')
    edge_rows = read_rows(args.model_dir / 'edge_audit.csv')
    safe_edges = {tuple(sorted((int(row['first_id']), int(row['second_id'])))) for row in edge_rows if row['is_safe'] == '1'}
    require(len(safe_edges) == sum(row['is_safe'] == '1' for row in edge_rows), '検証済み辺の重複')
    for layer in final['edge_layers']:
        actual = set()
        for edge in layer:
            first, second, age, is_active = struct.unpack('<iii?', edge['record'])
            require(age == 1 and is_active, '出力辺の無効状態')
            actual.add(tuple(sorted((first, second))))
        require(actual == safe_edges and len(actual) == len(layer), '未検証辺の残留・辺集合の不一致')
    parent = {int(node_id): int(node_id) for node_id in ids}

    def find(node_id):
        while parent[node_id] != node_id:
            parent[node_id] = parent[parent[node_id]]
            node_id = parent[node_id]
        return node_id

    for first, second in safe_edges:
        parent[find(first)] = find(second)
    components = {}
    for node_id in parent:
        components.setdefault(find(node_id), []).append(node_id)
    largest = set(max(components.values(), key=lambda values: (len(values), -min(values))))
    connected_tcp = tcp[np.isin(ids, list(largest))]
    references = read_rows(args.audit / 'references_kept.csv')
    heldout = read_rows(args.audit / 'heldout_kept.csv') if (args.audit / 'heldout_kept.csv').exists() else []
    all_coverage, all_dist = coverage_by_arm(tcp, references)
    connected_coverage, connected_dist = coverage_by_arm(connected_tcp, references)
    heldout_coverage, _ = coverage_by_arm(tcp, heldout)
    args.output.mkdir(parents=True, exist_ok=False)
    columns = ['id', 'source_arm', 'source_idx', 'dist_m']
    write_rows(args.output / 'missing_safe_references.csv', columns, missing_rows(references, all_dist))
    write_rows(args.output / 'missing_connected_references.csv', columns, missing_rows(references, connected_dist))
    result = {'model': args.model, 'num_nodes': len(ids), 'num_removed_nodes': len(original_ids) - len(safe_ids),
        'num_added_witnesses': len(extras), 'num_validated_edges_per_layer': len(safe_edges),
        'graph': verify.graph_stats(final), 'safe_reference_coverage': all_coverage,
        'largest_component_reference_coverage': connected_coverage, 'safe_heldout_coverage': heldout_coverage,
        'num_largest_component_nodes': len(largest), 'is_node_set_verified': True,
        'is_q_preserved': True, 'is_saved_records_preserved': True, 'is_edge_audit_equal_to_all_layers': True,
        'is_safe_reference_coverage_complete': bool(np.all(all_dist <= .02)),
        'collision_evidence': rebuild_metrics,
        'pose_audit_evidence': audit_metrics,
        'is_collision_policy_equal': True,
        'input_sha256': {str(path): sha256(path) for path in [old / 'model/gng.bin',
            args.audit / 'metrics.json', args.selection / 'selection.json']},
        'output_sha256': {'gng': sha256(args.model_dir / 'gng.bin')},
        'limit': '記録済み安全参照点に対する位置被覆。全連続姿勢領域や辺の連続時間非干渉の保証は対象外。'}
    vlut_path = args.model_dir / 'vlut.bin'
    if args.require_vlut:
        require(vlut_path.exists(), 'VLUT未生成')
    if vlut_path.exists():
        header, relations = export.read_vlut(vlut_path, set(ids))
        seen = set()
        for start in range(0, len(relations), 1000000):
            seen.update(int(value) for value in np.unique(relations['node'][start:start + 1000000]))
        require(seen == set(ids), 'VLUTに残る削除済み参照または占有欠落ノード')
        result['output_sha256']['vlut'] = sha256(vlut_path)
        result['vlut'] = {'num_references': len(relations), 'is_node_set_equal': True,
                          'resolution_m': struct.unpack_from('<f', header, 8)[0]}
    save_json(args.output / 'verification.json', result)
    require(result['is_safe_reference_coverage_complete'], '出力GNGの安全参照2 cm被覆の欠落')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('prepare', 'select', 'verify'))
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--model', choices=('max', 'long'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--audit', type=Path)
    parser.add_argument('--selection', type=Path)
    parser.add_argument('--model-dir', type=Path)
    parser.add_argument('--require-vlut', action='store_true')
    args = parser.parse_args()
    require(not args.output.exists(), '既存出力先への上書き拒否')
    root = args.root.resolve()
    old = root / 'artifacts/gng_coverage_repair_20261001' / args.model
    export = load_module('safe_export', root / 'benchmarks/voxel_pose_compression_20260930/export_preview.py')
    verify = load_module('safe_verify', root / 'benchmarks/gng_coverage_repair_20261001/verify_repair.py')
    if args.mode == 'prepare':
        prepare(args, old)
    elif args.mode == 'select':
        require(args.audit is not None, 'selectには--auditが必要')
        select(args, old, export, verify)
    else:
        require(args.audit is not None and args.selection is not None and args.model_dir is not None,
                'verifyには--audit --selection --model-dirが必要')
        verify_result(args, old, export, verify)
    print(json.dumps({'mode': args.mode, 'model': args.model, 'output': str(args.output)}, ensure_ascii=False))


if __name__ == '__main__':
    main()

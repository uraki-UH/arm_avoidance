#!/usr/bin/env python3
"""元GNGを固定保持した、既知到達点被覆に必要な実姿勢の補完選択。"""

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import xml.etree.ElementTree as et

import numpy as np
from scipy.spatial import cKDTree


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def dist_summary(values, max_tcp_dist_th):
    values = np.asarray(values, dtype=np.float64)
    return {'num': int(len(values)), 'mean_m': float(values.mean()),
            'p95_m': float(np.quantile(values, .95)), 'max_m': float(values.max()),
            'num_covered': int(np.count_nonzero(values <= max_tcp_dist_th)),
            'fraction_covered': float(np.mean(values <= max_tcp_dist_th))}


def select_witnesses(base_tcp, witness_tcp, references, max_tcp_dist_th):
    """初期最遠順の決定的選択と、選択姿勢の両腕による被覆更新。"""
    require(np.isfinite(max_tcp_dist_th) and max_tcp_dist_th > 0, '被覆距離の不正')
    require(base_tcp.ndim == 3 and base_tcp.shape[1:] == (2, 3) and len(base_tcp) > 0,
            '元TCP配列の不正')
    require(np.isfinite(base_tcp).all(), '元TCP配列の非有限値')
    require(len(witness_tcp) == len(references) == 2, '左右参照配列の不足')
    for layer in range(2):
        require(references[layer].ndim == 2 and references[layer].shape[1] == 3,
                '参照TCP配列の不正')
        require(witness_tcp[layer].shape == (len(references[layer]), 2, 3),
                '証拠TCP配列の不正')
        require(np.isfinite(references[layer]).all() and np.isfinite(witness_tcp[layer]).all(),
                '証拠TCP配列の非有限値')
        require(len(references[layer]) > 0, '参照点の欠落')
    reference_trees = [cKDTree(points) for points in references]
    before = [cKDTree(base_tcp[:, layer]).query(references[layer], workers=1)[0]
              for layer in range(2)]
    covered = [values <= max_tcp_dist_th for values in before]
    # 距離が同じ場合の左腕・入力添字による安定した順序
    order = sorted(((-float(dist), layer, idx)
                    for layer in range(2) for idx, dist in enumerate(before[layer])
                    if not covered[layer][idx]))
    selected = []
    for _, source_arm, source_idx in order:
        if covered[source_arm][source_idx]:
            continue
        tcp = witness_tcp[source_arm][source_idx]
        selected.append((source_arm, source_idx))
        for layer in range(2):
            indices = reference_trees[layer].query_ball_point(tcp[layer], max_tcp_dist_th,
                                                             p=2, eps=0, workers=1)
            if indices:
                indices = np.asarray(indices, dtype=np.int64)
                # 探索結果を実距離で確認した上での被覆登録
                dist = np.linalg.norm(references[layer][indices] - tcp[layer], axis=1)
                covered[layer][indices[dist <= max_tcp_dist_th]] = True
        require(covered[source_arm][source_idx], '保存精度で証拠点自身を被覆できない距離設定')
    require(all(values.all() for values in covered), '補完後の未被覆点')
    additions = np.asarray([witness_tcp[layer][idx] for layer, idx in selected], dtype=np.float32)
    if len(additions):
        all_tcp = np.concatenate([base_tcp, additions], axis=0)
    else:
        all_tcp = base_tcp.copy()
    # 全参照点に対する最近傍探索による選択結果の独立した最終確認
    after = [cKDTree(all_tcp[:, layer]).query(references[layer], workers=1)[0]
             for layer in range(2)]
    require(all(np.all(values <= max_tcp_dist_th) for values in after), '厳密な最終被覆検査の失敗')
    return selected, before, after


def read_joint_limits(urdf_path, joint_names):
    joints = {joint.get('name'): joint for joint in et.parse(urdf_path).getroot().findall('joint')}
    limits = []
    for name in joint_names:
        joint = joints[name]
        require(joint.get('type') in ('continuous', 'revolute'), '未対応の可動関節型')
        if joint.get('type') == 'continuous':
            limits.append((-np.pi, np.pi))
        else:
            limits.append(tuple(float(joint.find('limit').get(key)) for key in ('lower', 'upper')))
    return np.asarray(limits, dtype=np.float64)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--model', choices=('max', 'long'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-tcp-dist-th', type=float, default=.02)
    args = parser.parse_args()
    started = time.monotonic()
    root = args.root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    require(not output.exists(), f'既存出力先への上書き拒否: {output}')
    require(np.isfinite(args.max_tcp_dist_th) and args.max_tcp_dist_th > 0,
            'max_tcp_dist_thは有限の正数が必要')
    bench = root / 'benchmarks/voxel_pose_compression_20260930'
    prepare = load_module('repair_prepare', bench / 'prepare.py')
    workspace = load_module('repair_workspace', bench / 'workspace_coverage.py')
    model_name = 'topo_dual_arm_max' + ('_long' if args.model == 'long' else '')
    model_dir = root / 'gng_vlut_system/gng_results' / model_name
    gng_path = model_dir / 'gng.bin'
    map_paths = [model_dir / 'reachability' / (arm + '_arm.bin') for arm in ('left', 'right')]
    metadata_paths = [Path(str(path) + '.json') for path in map_paths]
    metadata = [json.loads(path.read_text()) for path in metadata_paths]
    urdf_paths = [root / Path(item['urdf_path']).relative_to('/ros2_ws/src') for item in metadata]
    require(urdf_paths[0] == urdf_paths[1], '左右マップのURDF不一致')
    urdf_path = urdf_paths[0]
    joint_names = sum([item['joint_names'] for item in metadata], [])
    require(joint_names == [f'{side}_joint{idx}' for side in ('L', 'R') for idx in range(1, 8)],
            '既存GNGの14関節順序と異なるマップ')
    input_paths = [gng_path, model_dir / 'vlut.bin', urdf_path, *map_paths, *metadata_paths]
    input_paths += [bench / 'prepare.py', bench / 'workspace_coverage.py']
    input_hashes = {str(path): sha256(path) for path in input_paths}
    ids, q, tcp, num_edges = prepare.read_gng(gng_path)
    require(len(ids) == 10000, '想定する元GNGの1万ノードと不一致')
    limits = read_joint_limits(urdf_path, joint_names)
    witness_q, witness_tcp, references = [], [], []
    max_fk_error = []
    max_cell_axis_error = []
    roots = []
    for layer, map_path in enumerate(map_paths):
        source = metadata[layer]
        require(source['enable_self_collision'] and source['other_joints'] == 'zero',
                '証拠マップの自己干渉検査または他関節条件の不一致')
        require(not source['environment_collision_checked'], '未対応の環境衝突検査条件')
        nodes = workspace.read_map(map_path)
        require(len(nodes) == source['num_cells'], '証拠マップの件数不一致')
        require(nodes['angles'].shape == (len(nodes), 7), '証拠マップの関節次元不一致')
        full_q = np.zeros((len(nodes), 14), dtype=np.float32)
        full_q[:, layer * 7:(layer + 1) * 7] = nodes['angles']
        two_tcp = []
        for tcp_layer in range(2):
            positions, frame = workspace.forward_kinematics(
                urdf_path, metadata[tcp_layer]['eef_link'], joint_names, full_q.astype(np.float64))
            two_tcp.append(positions)
            roots.append(frame)
        actual = two_tcp[layer]
        reference_error = float(np.abs(actual - nodes['position']).max())
        require(reference_error <= source['voxel_size'] / 2 + 1e-6, '証拠姿勢と元セルの不一致')
        references.append(actual)
        witness_q.append(full_q)
        # GNGのfloat32保存と同じ精度での候補TCP
        witness_tcp.append(np.stack(two_tcp, axis=1).astype(np.float32))
        max_cell_axis_error.append(reference_error)
        original_fk, frame = workspace.forward_kinematics(
            urdf_path, source['eef_link'], joint_names, q.astype(np.float64))
        error = float(np.linalg.norm(original_fk - tcp[:, layer], axis=1).max())
        require(error < 1e-5, '元GNG全ノードの保存TCPと独立FKの不一致')
        max_fk_error.append(error)
        roots.append(frame)
    require(len(set(roots)) == 1, '独立FKの基準フレーム不一致')
    all_q = np.concatenate([q, *witness_q], axis=0).astype(np.float32)
    all_tcp = np.concatenate([tcp, *witness_tcp], axis=0).astype(np.float32)
    require(np.isfinite(all_q).all() and np.isfinite(all_tcp).all(), '候補配列の非有限値')
    require(np.all(all_q >= limits[:, 0]) and np.all(all_q <= limits[:, 1]),
            '元姿勢または候補姿勢のURDF関節範囲外')
    selected, before, after = select_witnesses(tcp, witness_tcp, references, args.max_tcp_dist_th)
    num_original = len(ids)
    offsets = [num_original, num_original + len(witness_q[0])]
    selected_candidate_idx = np.array([offsets[layer] + idx for layer, idx in selected], dtype=np.int64)
    new_q = all_q[selected_candidate_idx]
    new_tcp = all_tcp[selected_candidate_idx]
    new_ids = np.arange(int(ids.max()) + 1, int(ids.max()) + 1 + len(selected), dtype=np.int64)
    require(not len(new_ids) or new_ids[-1] <= np.iinfo(np.int32).max, 'GNGノードID上限超過')
    require(np.array_equal(all_q[:num_original], q) and np.array_equal(all_tcp[:num_original], tcp),
            '元姿勢の保持検査失敗')
    output.mkdir(parents=True, exist_ok=False)
    candidate_dir = output / 'candidates'
    candidate_dir.mkdir()
    np.save(candidate_dir / 'all_q.npy', all_q, allow_pickle=False)
    np.save(candidate_dir / 'all_tcp.npy', all_tcp, allow_pickle=False)
    np.save(candidate_dir / 'original_ids.npy', ids, allow_pickle=False)
    np.save(candidate_dir / 'selected_candidate_idx.npy', selected_candidate_idx, allow_pickle=False)
    source_arm = np.concatenate([np.full(num_original, -1, dtype=np.int8),
                                 np.zeros(len(witness_q[0]), dtype=np.int8),
                                 np.ones(len(witness_q[1]), dtype=np.int8)])
    source_idx = np.concatenate([np.arange(num_original), np.arange(len(witness_q[0])),
                                np.arange(len(witness_q[1]))]).astype(np.int64)
    np.save(candidate_dir / 'source_arm.npy', source_arm, allow_pickle=False)
    np.save(candidate_dir / 'source_idx.npy', source_idx, allow_pickle=False)
    columns = ['id'] + [f'q{idx}' for idx in range(14)]
    columns += [f'tcp{layer}_{axis}' for layer in range(2) for axis in ('x', 'y', 'z')]
    columns += ['source_arm', 'source_idx']
    csv_path = output / 'new_nodes.csv'
    with csv_path.open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for idx, (layer, source_idx) in enumerate(selected):
            writer.writerow([int(new_ids[idx]), *new_q[idx].tolist(), *new_tcp[idx].reshape(-1).tolist(),
                             ('left', 'right')[layer], source_idx])
    # CSV再読込後のfloat32姿勢値の一致と、元入力ファイルの不変確認
    with csv_path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    require(len(rows) == len(selected), '保存された追加姿勢件数の不一致')
    if rows:
        restored_q = np.array([[float(row[f'q{idx}']) for idx in range(14)] for row in rows], dtype=np.float32)
        restored_tcp = np.array([[float(row[f'tcp{layer}_{axis}']) for layer in range(2)
                                  for axis in ('x', 'y', 'z')] for row in rows], dtype=np.float32).reshape(-1, 2, 3)
        require(np.array_equal(restored_q, new_q) and np.array_equal(restored_tcp, new_tcp),
                'CSV保存精度の不一致')
        require([int(row['id']) for row in rows] == new_ids.tolist(), '追加IDの不一致')
    after_hashes = {str(path): sha256(path) for path in input_paths}
    require(input_hashes == after_hashes, '実行中の元入力ファイルの変更')
    result = {
        'model': args.model, 'model_name': model_name, 'max_tcp_dist_th_m': args.max_tcp_dist_th,
        'algorithm': '元全ノード固定、初期最遠参照順、固定実証拠姿勢の追加と両TCPによる被覆更新',
        'num_original_nodes': num_original, 'num_original_edges': num_edges,
        'num_witness_candidates': [len(values) for values in witness_q],
        'num_all_candidates': len(all_q), 'num_selected_additions': len(selected),
        'num_final_nodes': num_original + len(selected),
        'num_selected_by_source_arm': [sum(layer == arm for layer, _ in selected) for arm in range(2)],
        'first_new_id': int(new_ids[0]) if len(new_ids) else None,
        'last_new_id': int(new_ids[-1]) if len(new_ids) else None,
        'joint_names': joint_names, 'joint_limits_rad': limits.tolist(), 'urdf_root_link': roots[0],
        'max_original_fk_error_m': max_fk_error, 'max_witness_cell_axis_error_m': max_cell_axis_error,
        'coverage': {arm: {'before': dist_summary(before[layer], args.max_tcp_dist_th),
                           'after': dist_summary(after[layer], args.max_tcp_dist_th)}
                     for layer, arm in enumerate(('left', 'right'))},
        'input_sha256': input_hashes, 'has_input_mutation': False,
        'has_original_pose_change': False, 'has_original_pose_removal': False,
        'has_added_edges': False, 'has_collision_recheck': False,
        'scope': '片腕mapの既知証拠TCP点への位置被覆。双腕同時到達・姿勢角・連続領域の保証なし。元1万姿勢の同時姿勢は全保持。',
        'collision_scope': '追加候補は既存mapの生成時FCL通過姿勢、他腕・腰・頭・グリッパー0。現在のモデルによる衝突再判定と辺の検査は別工程。',
        'candidate_array_order': '元GNG順、左map順、右map順。source_arm=-1は元、0は左、1は右。',
        'elapsed_sec': time.monotonic() - started,
    }
    result['output_sha256'] = {str(path.relative_to(output)): sha256(path)
                              for path in sorted(output.rglob('*')) if path.is_file()}
    with (output / 'selection.json').open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""補完GNG・VLUTの保存整合、独立FK、既知点と独立姿勢の位置被覆の検査。"""

import argparse
from collections import Counter
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as et

import numpy as np
from scipy.spatial import cKDTree


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


def decode_states(model):
    """別パーサで境界検査済みの生ノードレコードからの角度・TCP抽出。"""
    ids, angles, tcp = [], [], []
    for node in model['nodes']:
        data = node['record']
        offset = 12

        def vector():
            nonlocal offset
            rows, cols = struct.unpack_from('<qq', data, offset)
            offset += 16
            count = rows * cols
            require(0 < count < 1000, 'ノードベクトルの不正な次元')
            values = np.frombuffer(data, dtype='<f4', count=count, offset=offset)
            offset += count * 4
            return values

        q = vector()
        primary = vector()
        num_layers = struct.unpack_from('<i', data, offset)[0]
        offset += 4
        require(num_layers == 2 and q.shape == (14,), '双腕GNGの次元不一致')
        points = np.stack([vector() for _ in range(num_layers)])
        require(points.shape == (2, 3) and primary.shape == (3,), 'TCP次元の不一致')
        require(np.array_equal(primary, points[0]), '主TCPと第0層TCPの不一致')
        ids.append(node['id'])
        angles.append(q)
        tcp.append(points)
    return np.asarray(ids, dtype=np.int64), np.asarray(angles), np.asarray(tcp)


def read_pose_csv(path, *, has_ids):
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {f'q{idx}' for idx in range(14)}
        required |= {f'tcp{layer}_{axis}' for layer in range(2) for axis in ('x', 'y', 'z')}
        if has_ids:
            required.add('id')
        require(reader.fieldnames is not None and required <= set(reader.fieldnames),
                f'姿勢CSVの必須列不足: {path}')
        require(len(reader.fieldnames) == len(set(reader.fieldnames)), '姿勢CSVの列重複')
        rows = list(reader)
    q = np.array([[float(row[f'q{idx}']) for idx in range(14)] for row in rows],
                 dtype=np.float32).reshape(-1, 14)
    tcp = np.array([[float(row[f'tcp{layer}_{axis}']) for layer in range(2)
                     for axis in ('x', 'y', 'z')] for row in rows],
                   dtype=np.float64).reshape(-1, 2, 3)
    require(np.isfinite(q).all() and np.isfinite(tcp).all(), '姿勢CSVの非有限値')
    ids = np.array([int(row['id']) for row in rows], dtype=np.int64) if has_ids else None
    if has_ids:
        require(len(set(ids)) == len(ids), '姿勢CSVのID重複')
    return ids, q, tcp, rows


def graph_stats(model):
    output = []
    for name, edges in zip(('angle', 'left_tcp', 'right_tcp'), model['edge_layers']):
        parent = {node_id: node_id for node_id in model['node_ids']}
        degree = dict.fromkeys(parent, 0)

        def find(node_id):
            while parent[node_id] != node_id:
                parent[node_id] = parent[parent[node_id]]
                node_id = parent[node_id]
            return node_id

        num_active = 0
        for edge in edges:
            first, second, age, is_active = struct.unpack('<iii?', edge['record'])
            if not is_active:
                continue
            num_active += 1
            degree[first] += 1
            degree[second] += 1
            parent[find(first)] = find(second)
        components = Counter(find(node_id) for node_id in parent)
        output.append({'layer': name, 'num_nodes': len(parent), 'num_serialized_edges': len(edges),
                       'num_active_edges': num_active, 'num_components': len(components),
                       'max_component_nodes': max(components.values(), default=0),
                       'num_isolated': sum(value == 0 for value in degree.values()),
                       'mean_active_degree': 2 * num_active / max(1, len(parent))})
    return output



def link_presence(relations, node_ids, num_links):
    """有限サイズの分割読込みによるノード別リンク参照の有無。"""
    sorted_ids = np.asarray(sorted(node_ids), dtype=np.int64)
    has_link_refs = np.zeros((len(sorted_ids), num_links), dtype=bool)
    for start in range(0, len(relations), 1000000):
        block = relations[start:start + 1000000]
        indices = np.searchsorted(sorted_ids, block['node'])
        has_link_refs[indices, block['link']] = True
    return sorted_ids, has_link_refs


def dist_summary(values):
    values = np.asarray(values, dtype=np.float64)
    require(len(values) > 0 and np.isfinite(values).all(), '距離集計の空配列または非有限値')
    return {'num': len(values), 'mean_m': float(values.mean()),
            'median_m': float(np.median(values)), 'p95_m': float(np.quantile(values, .95)),
            'max_m': float(values.max()),
            'within': {str(dist): float(np.mean(values <= dist)) for dist in (.02, .03, .05)}}


def joint_limits(urdf_path, names):
    joints = {joint.get('name'): joint for joint in et.parse(urdf_path).getroot().findall('joint')}
    limits = []
    for name in names:
        joint = joints[name]
        require(joint.get('type') in ('continuous', 'revolute'), '未対応の可動関節型')
        limits.append((-np.pi, np.pi) if joint.get('type') == 'continuous' else
                      tuple(float(joint.find('limit').get(key)) for key in ('lower', 'upper')))
    return np.asarray(limits)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--model', choices=('max', 'long'), required=True)
    parser.add_argument('--output', type=Path, required=True, help='gng.bin・vlut.bin・heldout.csvを含む検査対象')
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--cpp-report', type=Path)
    parser.add_argument('--robot-name')
    args = parser.parse_args()
    started = time.monotonic()
    root = args.root.expanduser().resolve()
    output = args.output.expanduser().resolve(strict=True)
    selection_dir = args.selection.expanduser().resolve(strict=True)
    report_path = output / 'verification.json'
    expected_path = output / 'expected.json'
    require(not report_path.exists() and not expected_path.exists(), '既存検証出力への上書き拒否')
    bench = root / 'benchmarks/voxel_pose_compression_20260930'
    export = load_module('repair_export', bench / 'export_preview.py')
    workspace = load_module('repair_workspace', bench / 'workspace_coverage.py')
    model_name = 'topo_dual_arm_max' + ('_long' if args.model == 'long' else '')
    original_dir = root / 'gng_vlut_system/gng_results' / model_name
    require(output != original_dir.resolve(), '元モデルディレクトリへの検証出力拒否')
    original_path = original_dir / 'gng.bin'
    gng_path = output / 'gng.bin'
    vlut_path = output / 'vlut.bin'
    heldout_path = output / 'heldout.csv'
    cpp_report_path = args.cpp_report.expanduser().resolve() if args.cpp_report else output / 'repair_metrics.json'
    cpp_report = json.loads(cpp_report_path.read_text())
    selection_path = selection_dir / 'selection.json'
    selected_path = selection_dir / 'new_nodes.csv'
    selection = json.loads(selection_path.read_text())
    require(selection['model'] == args.model, '補完選択モデルの不一致')
    max_tcp_dist_th = float(selection['max_tcp_dist_th_m'])
    require(np.isfinite(max_tcp_dist_th) and max_tcp_dist_th > 0, '被覆距離の不正')
    map_paths = [original_dir / 'reachability' / (arm + '_arm.bin') for arm in ('left', 'right')]
    map_metadata_paths = [Path(str(path) + '.json') for path in map_paths]
    metadata = [json.loads(path.read_text()) for path in map_metadata_paths]
    names = sum([item['joint_names'] for item in metadata], [])
    require(names == [f'{side}_joint{idx}' for side in ('L', 'R') for idx in range(1, 8)],
            'URDFとGNGの関節順序不一致')
    urdf_paths = [root / Path(item['urdf_path']).relative_to('/ros2_ws/src') for item in metadata]
    require(urdf_paths[0] == urdf_paths[1], '左右マップのURDF不一致')
    urdf_path = urdf_paths[0]
    input_paths = [original_path, original_dir / 'vlut.bin', gng_path, vlut_path, heldout_path,
                   cpp_report_path, selection_path, selected_path, urdf_path, *map_paths, *map_metadata_paths]
    input_hashes = {str(path): sha256(path) for path in input_paths}
    for path in [original_path, original_dir / 'vlut.bin', urdf_path, *map_paths, *map_metadata_paths]:
        require(selection['input_sha256'].get(str(path)) == input_hashes[str(path)],
                f'選択時点からの入力ファイル変更: {path}')

    original = export.read_gng(original_path)
    repaired = export.read_gng(gng_path)
    original_ids, original_q, original_tcp = decode_states(original)
    ids, q, tcp = decode_states(repaired)
    added_ids, added_q, added_tcp, _ = read_pose_csv(selected_path, has_ids=True)
    require(len(original_ids) == 10000, '元GNGのノード数不一致')
    require(set(original_ids).isdisjoint(added_ids), '元ノードIDと追加IDの重複')
    require(set(ids) == set(original_ids) | set(added_ids), '保存後ノードID集合の欠落または過剰')
    require(len(ids) == selection['num_final_nodes'], '補完選択と保存後ノード数の不一致')
    require(cpp_report['num_original_nodes'] == len(original_ids) and
            cpp_report['num_added_nodes'] == len(added_ids) and
            cpp_report['num_output_nodes'] == len(ids), 'C++検査報告のノード件数不一致')
    require(cpp_report['is_binary_roundtrip_verified'] and cpp_report['is_existing_gng_loader_verified'],
            'C++保存再読込検査の未完了')
    require(cpp_report['num_collision_checks'] > 0, 'C++のFCL検査実績なし')
    require(cpp_report['num_original_collision_rejected'] == 0 and
            cpp_report['num_original_limit_rejected'] == 0 and
            cpp_report['num_original_collision_free'] == len(original_ids),
            '元姿勢のFCLまたは関節限界検査の失敗')
    require(cpp_report['output_graph'][0]['num_new_nodes_connected_to_original'] == len(added_ids),
            '元の角度グラフ成分へ接続できない追加ノード')
    id_to_idx = {int(node_id): idx for idx, node_id in enumerate(ids)}
    original_idx = np.array([id_to_idx[int(node_id)] for node_id in original_ids])
    added_idx = np.array([id_to_idx[int(node_id)] for node_id in added_ids], dtype=np.int64)
    require(q[original_idx].tobytes() == original_q.tobytes(), '元全関節角の生バイト不一致')
    require(q[added_idx].tobytes() == added_q.tobytes(), '追加関節角と選択CSVの生バイト不一致')
    max_original_tcp_change = float(np.linalg.norm(tcp[original_idx] - original_tcp, axis=2).max())
    require(max_original_tcp_change < 1e-5, '元TCPの許容差超過')
    max_added_tcp_change = float(np.linalg.norm(tcp[added_idx] - added_tcp, axis=2).max()) if len(added_idx) else 0.0
    require(max_added_tcp_change < 1e-5, '追加TCPの選択CSVとの差超過')
    for layer in range(3):
        before = Counter(edge['record'] for edge in original['edge_layers'][layer])
        after = Counter(edge['record'] for edge in repaired['edge_layers'][layer])
        require(not (before - after), f'元辺レコードの欠落または変更: layer={layer}')
    limits = joint_limits(urdf_path, names)
    require(np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]), '保存関節角のURDF範囲外')

    original_trees = [cKDTree(original_tcp[:, layer]) for layer in range(2)]
    repaired_trees = [cKDTree(tcp[:, layer]) for layer in range(2)]
    fk_errors = []
    known_coverage = {}
    for layer, arm in enumerate(('left', 'right')):
        source = metadata[layer]
        saved_fk, frame = workspace.forward_kinematics(urdf_path, source['eef_link'], names, q.astype(np.float64))
        error = np.linalg.norm(saved_fk - tcp[:, layer], axis=1)
        require(float(error.max()) < 1e-5, '保存後全ノードの独立FK不一致')
        fk_errors.append({'max_all_m': float(error.max()),
                          'max_added_m': float(error[added_idx].max()) if len(added_idx) else 0.0})
        nodes = workspace.read_map(map_paths[layer])
        witness, witness_frame = workspace.forward_kinematics(
            urdf_path, source['eef_link'], source['joint_names'], nodes['angles'].astype(np.float64))
        require(frame == witness_frame, '独立FK基準フレームの不一致')
        cell_error = float(np.abs(witness - nodes['position']).max())
        require(cell_error <= source['voxel_size'] / 2 + 1e-6, '既知証拠点と保存セルの不一致')
        stored_dist = repaired_trees[layer].query(witness, workers=1)[0]
        # 保存TCPと独立FK復元TCPの両方に対する厳密な距離確認
        original_trees[layer] = cKDTree(saved_fk[original_idx])
        repaired_trees[layer] = cKDTree(saved_fk)
        before = original_trees[layer].query(witness, workers=1)[0]
        after = repaired_trees[layer].query(witness, workers=1)[0]
        require(np.all(stored_dist <= max_tcp_dist_th), f'保存後の既知参照点被覆不足: {arm}')
        require(np.all(after <= max_tcp_dist_th), f'独立FK復元後の既知参照点被覆不足: {arm}')
        known_coverage[arm] = {'num_reference_points': len(witness),
                               'max_witness_cell_axis_error_m': cell_error,
                               'before': dist_summary(before), 'after': dist_summary(after),
                               'max_saved_tcp_dist_m': float(stored_dist.max()),
                               'fraction_within_required_dist': float(np.mean(after <= max_tcp_dist_th))}

    vlut_header, relations = export.read_vlut(vlut_path, set(ids))
    original_vlut_header, original_relations = export.read_vlut(original_dir / 'vlut.bin', set(original_ids))
    require(set(np.unique(relations['node'])) == set(ids), 'VLUT参照のない保存ノード')
    expected_link_ids = sorted(int(value) for value in np.unique(original_relations['link']))
    require(expected_link_ids, '元VLUTの非空リンク集合の欠落')
    require(set(np.unique(relations['link'])) <= set(expected_link_ids), '未知のVLUTリンクID')
    num_links = max(expected_link_ids) + 1
    _, original_has_links = link_presence(original_relations, set(original_ids), num_links)
    expected_has_links = np.zeros(num_links, dtype=bool)
    expected_has_links[expected_link_ids] = True
    require(np.all(original_has_links == expected_has_links), '元VLUT自体のノード別非空リンク集合不一致')
    sorted_ids, repaired_has_links = link_presence(relations, set(ids), num_links)
    has_complete_links = np.all(repaired_has_links == expected_has_links, axis=1)
    require(has_complete_links.all(),
            f'VLUTのノード別リンク参照欠落: ids={sorted_ids[~has_complete_links][:10].tolist()}')
    resolution = struct.unpack_from('<f', vlut_header, 8)[0]
    original_resolution = struct.unpack_from('<f', original_vlut_header, 8)[0]
    require(resolution == original_resolution, 'VLUT格子解像度の変更')
    _, heldout_q, heldout_tcp, heldout_rows = read_pose_csv(heldout_path, has_ids=False)
    require(len(heldout_rows) > 0, '独立な全身姿勢サンプルの欠落')
    heldout_source_arm = np.array([row['source_arm'] for row in heldout_rows])
    require(set(heldout_source_arm) == {'L', 'R'}, '独立サンプルの左右分類不一致')
    require([int(np.sum(heldout_source_arm == side)) for side in ('L', 'R')] ==
            cpp_report['heldout']['num_accepted_per_arm'], 'C++独立サンプル採用件数との不一致')
    require(np.all(heldout_q >= limits[:, 0]) and np.all(heldout_q <= limits[:, 1]),
            '独立姿勢サンプルのURDF範囲外')
    heldout_coverage = {}
    for layer, arm in enumerate(('left', 'right')):
        positions, frame = workspace.forward_kinematics(
            urdf_path, metadata[layer]['eef_link'], names, heldout_q.astype(np.float64))
        error = float(np.linalg.norm(positions - heldout_tcp[:, layer], axis=1).max())
        require(error < 1e-5, '独立姿勢サンプルのTCPと独立FKの不一致')
        before = original_trees[layer].query(positions, workers=1)[0]
        after = repaired_trees[layer].query(positions, workers=1)[0]
        has_sampled_arm = heldout_source_arm == ('L', 'R')[layer]
        # 非可動側のゼロ姿勢による被覆率の水増しを避ける可動側のみの集計
        heldout_coverage[arm] = {'num_samples': int(has_sampled_arm.sum()),
                                 'max_fk_error_m': error,
                                 'before': dist_summary(before[has_sampled_arm]),
                                 'after': dist_summary(after[has_sampled_arm]),
                                 'all_rows_including_stationary_arm': {
                                     'num_samples': len(positions),
                                     'before': dist_summary(before), 'after': dist_summary(after)}}
    require(input_hashes == {str(path): sha256(path) for path in input_paths}, '検査中の入力ファイル変更')
    original_graph = graph_stats(original)
    repaired_graph = graph_stats(repaired)
    expected = {'robot_name': args.robot_name or f'coverage_{args.model}_2cm',
                'edge_counts': [item['num_active_edges'] for item in repaired_graph],
                'frame_id': f"{args.robot_name or f'coverage_{args.model}_2cm'}/base_link",
                'nodes': {str(node_id): {'angles': q[idx].tolist(), 'tcp': tcp[idx].tolist()}
                          for idx, node_id in enumerate(ids)}}
    result = {
        'model': args.model, 'has_passed': True, 'max_tcp_dist_th_m': max_tcp_dist_th,
        'num_original_nodes': len(original_ids), 'num_added_nodes': len(added_ids), 'num_final_nodes': len(ids),
        'has_original_angle_change': False, 'has_original_node_removal': False,
        'has_original_edge_record_change': False,
        'max_original_tcp_change_m': max_original_tcp_change,
        'max_added_csv_tcp_change_m': max_added_tcp_change, 'fk_errors': fk_errors,
        'known_reference_coverage': known_coverage, 'heldout_full_body_coverage': heldout_coverage,
        'original_graph': original_graph, 'repaired_graph': repaired_graph,
        'vlut': {'resolution_m': resolution, 'num_original_relations': len(original_relations),
                 'num_final_relations': len(relations), 'num_nodes_with_relations': len(np.unique(relations['node'])),
                 'num_invalid_node_refs': 0, 'num_missing_node_refs': 0,
                 'expected_nonempty_link_ids': expected_link_ids,
                 'num_nodes_with_complete_link_refs': int(has_complete_links.sum()),
                 'num_nodes_with_missing_link_refs': 0},
        'cpp_report_path': str(cpp_report_path), 'cpp_report': cpp_report,
        'collision_validation_owner': 'C++repair_metricsの全身FCL姿勢・辺検査。独立Pythonでの衝突再計算なし。',
        'scope': '2cm以内の必須被覆は既存片腕mapの有限証拠点のみ。双腕同時目標・姿勢角・連続可動域の被覆保証なし。',
        'heldout_scope': '全身FCL検査済みの片腕一様サンプル、他腕0。主指標は可動側の行のみ。被覆率はサンプル分布依存、空間体積比ではない。',
        'input_sha256': input_hashes, 'has_input_mutation': False,
        'elapsed_sec': time.monotonic() - started,
    }
    with expected_path.open('x') as stream:
        json.dump(expected, stream, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    result['expected_sha256'] = sha256(expected_path)
    with report_path.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()

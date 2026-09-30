"""保存済格子占有と実姿勢参照に対する独立検証。"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import time

import numpy as np


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_input(path):
    data = path.read_bytes()
    header_size = struct.calcsize('<8sIIIIf')
    require(len(data) >= header_size, 'ヘッダ不足')
    magic, num_nodes, num_links, angle_dim, num_coord_layers, res = struct.unpack_from(
        '<8sIIIIf', data)
    require(magic == b'VOXPOSE1', '形式識別子の不一致')
    require(num_nodes > 0 and num_links > 0 and angle_dim > 0 and num_coord_layers > 0,
            'データ次元の不正')
    require(np.isfinite(res) and res > 0, '格子解像度の不正')
    offset = header_size
    nodes = {}
    num_cell_refs = 0

    def read_values(dtype, count):
        nonlocal offset
        size = np.dtype(dtype).itemsize * count
        require(offset + size <= len(data), 'ノードデータ不足')
        result = np.frombuffer(data, dtype=dtype, count=count, offset=offset)
        offset += size
        return result

    for _ in range(num_nodes):
        node_id = int(read_values('<i4', 1)[0])
        require(node_id not in nodes, f'入力node_idの重複: {node_id}')
        angles = read_values('<f4', angle_dim)
        tcp = read_values('<f4', num_coord_layers * 3).reshape(num_coord_layers, 3)
        require(np.isfinite(angles).all() and np.isfinite(tcp).all(),
                f'姿勢値の非有限値: {node_id}')
        masks = []
        for link_idx in range(num_links):
            num_cells = int(read_values('<u4', 1)[0])
            cells = read_values('<i4', num_cells * 3).reshape(num_cells, 3)
            # 入力集合の重複確認と順序非依存のセル数
            if num_cells > 1:
                packed_rows = cells.view(np.dtype((np.void, 12))).reshape(-1)
                require(np.unique(packed_rows).size == num_cells,
                        f'入力cellの重複: node={node_id}, link={link_idx}')
            masks.append(cells)
            num_cell_refs += num_cells
        require(any(len(mask) for mask in masks), f'全リンク占有の欠落: {node_id}')
        nodes[node_id] = {'angles': angles, 'tcp': tcp, 'masks': masks}
    require(offset == len(data), '入力末尾の余剰データ')
    return nodes, {
        'num_nodes': int(num_nodes),
        'num_links': int(num_links),
        'angle_dim': int(angle_dim),
        'num_coord_layers': int(num_coord_layers),
        'voxel_res_m': float(res),
        'num_original_cell_refs': int(num_cell_refs),
        'input_sha256': hashlib.sha256(data).hexdigest(),
    }


def read_assignments(path, nodes):
    assignments = {}
    group_indices = {}
    representative_indices = {}
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames is not None, 'assignmentヘッダの欠落')
        required_names = {'original_id', 'representative_id', 'group_idx', 'is_representative'}
        require(required_names <= set(reader.fieldnames),
                'assignment必須列: original_id,representative_id,group_idx,is_representative')
        for row in reader:
            node_id = int(row['original_id'])
            representative_id = int(row['representative_id'])
            group_idx = int(row['group_idx'])
            is_representative = int(row['is_representative'])
            require(node_id in nodes, f'未知の入力node_id: {node_id}')
            require(node_id not in assignments, f'assignment重複: {node_id}')
            require(representative_id in nodes, f'未知の代表ID: {representative_id}')
            require(group_idx >= 0, f'負のgroup_idx: {group_idx}')
            require(is_representative in (0, 1) and
                    bool(is_representative) == (node_id == representative_id),
                    f'代表フラグの不一致: {node_id}')
            require(group_indices.setdefault(group_idx, representative_id) == representative_id,
                    f'グループ内代表IDの不一致: {group_idx}')
            require(representative_indices.setdefault(representative_id, group_idx) == group_idx,
                    f'代表IDのグループ重複: {representative_id}')
            assignments[node_id] = representative_id
    require(set(assignments) == set(nodes), 'assignmentの入力ノード全集不一致')
    groups = {}
    for node_id, representative_id in assignments.items():
        require(assignments[representative_id] == representative_id,
                f'代表の自己割当欠落: {representative_id}')
        groups.setdefault(representative_id, []).append(node_id)
    return assignments, groups


def mask_set(mask):
    return {tuple(int(value) for value in row) for row in mask}


def hausdorff_sq(first, second):
    """全点対距離による双方向の最近点距離上限。"""
    if len(first) == 0 or len(second) == 0:
        require(len(first) == len(second), '片側リンク占有の欠落')
        return 0.0
    first_values = first.astype(np.float64)
    second_values = second.astype(np.float64)
    min_second_sq = np.full(len(second_values), np.inf)
    max_first_sq = 0.0
    # メモリ量を制限した直接点間距離の計算
    for start_idx in range(0, len(first_values), 64):
        diff = first_values[start_idx:start_idx + 64, None, :] - second_values[None, :, :]
        pair_sq = np.einsum('ijk,ijk->ij', diff, diff)
        max_first_sq = max(max_first_sq, float(pair_sq.min(axis=1).max()))
        min_second_sq = np.minimum(min_second_sq, pair_sq.min(axis=0))
    return max(max_first_sq, float(min_second_sq.max()))


def summary(values):
    values = np.asarray(values, dtype=np.float64)
    if len(values) == 0:
        return {'mean': 0.0, 'p95': 0.0, 'max': 0.0}
    require(np.isfinite(values).all(), '集計値の非有限値')
    return {'mean': float(values.mean()), 'p95': float(np.percentile(values, 95)),
            'max': float(values.max())}


def verify(input_path, assignment_path, radius_cells):
    started = time.monotonic()
    require(radius_cells >= 0 and np.isfinite(radius_cells), '許容距離の不正')
    nodes, result = read_input(input_path)
    assignments, groups = read_assignments(assignment_path, nodes)
    max_dist_sq = 0.0
    num_checked_pairs = 0
    num_missing_representative_refs = 0
    num_union_cell_refs = 0
    num_union_member_cell_refs = 0
    num_merged_member_cell_refs = 0
    union_additional_ratios = []
    merged_union_additional_ratios = []
    joint_max_rad = []
    joint_norm_rad = []
    tcp_max_m = []
    num_singletons = 0
    radius_sq = float(radius_cells) ** 2

    for representative_id, member_ids in groups.items():
        representative = nodes[representative_id]
        if len(member_ids) == 1:
            num_singletons += 1
            num_cells = sum(len(mask) for mask in representative['masks'])
            num_union_cell_refs += num_cells
            num_union_member_cell_refs += num_cells
            union_additional_ratios.append(0.0)
            joint_max_rad.append(0.0)
            joint_norm_rad.append(0.0)
            tcp_max_m.append(0.0)
            continue

        representative_sets = [mask_set(mask) for mask in representative['masks']]
        union_sets = [set(mask) for mask in representative_sets]
        member_sets = {}
        for node_id in member_ids:
            node = nodes[node_id]
            if node_id == representative_id:
                member_sets[node_id] = representative_sets
                continue
            num_checked_pairs += 1
            sets = []
            for link_idx, (member_mask, representative_mask) in enumerate(
                    zip(node['masks'], representative['masks'])):
                try:
                    dist_sq = hausdorff_sq(member_mask, representative_mask)
                except ValueError as error:
                    raise ValueError(f'node={node_id}, representative={representative_id}, '
                                     f'link={link_idx}: {error}') from error
                require(dist_sq <= radius_sq,
                        f'許容距離超過: node={node_id}, representative={representative_id}, '
                        f'link={link_idx}, dist_sq={dist_sq}, radius_sq={radius_sq}')
                max_dist_sq = max(max_dist_sq, dist_sq)
                cells = mask_set(member_mask)
                sets.append(cells)
                union_sets[link_idx].update(cells)
                num_missing_representative_refs += len(cells - representative_sets[link_idx])
            member_sets[node_id] = sets

        num_union_cells = sum(len(mask) for mask in union_sets)
        num_union_cell_refs += num_union_cells
        num_union_member_cell_refs += num_union_cells * len(member_ids)
        for node_id in member_ids:
            sets = member_sets[node_id]
            for link_idx, cells in enumerate(sets):
                require(cells <= union_sets[link_idx],
                        f'union包含の不一致: node={node_id}, link={link_idx}')
            num_member_cells = sum(len(mask) for mask in sets)
            num_merged_member_cell_refs += num_member_cells
            ratio = (num_union_cells - num_member_cells) / num_member_cells
            union_additional_ratios.append(ratio)
            merged_union_additional_ratios.append(ratio)
            node = nodes[node_id]
            # 元入力float32姿勢の代表ID参照と生の関節差
            joint_diff = node['angles'].astype(np.float64) - representative['angles'].astype(np.float64)
            tcp_diff = node['tcp'].astype(np.float64) - representative['tcp'].astype(np.float64)
            joint_max_rad.append(float(np.max(np.abs(joint_diff))))
            joint_norm_rad.append(float(np.linalg.norm(joint_diff)))
            tcp_max_m.append(float(np.linalg.norm(tcp_diff, axis=1).max()))

    require(num_checked_pairs == len(nodes) - len(groups), '非代表の検証数不一致')
    require(len(union_additional_ratios) == len(nodes), '姿勢別集計数不一致')
    num_original_refs = result['num_original_cell_refs']
    result.update({
        'is_valid': True,
        'radius_cells': float(radius_cells),
        'metric': 'リンク別双方向ユークリッド距離（格子セル中心）',
        'guarantee_scope': '保存済入力格子占有のみ。URDF連続実形状・辺の補間経路は未保証。',
        'representative_pose_storage': '原入力のnode_id参照。角度・TCPは原入力float32値。',
        'num_representatives': len(groups),
        'num_singleton_groups': num_singletons,
        'num_merged_groups': len(groups) - num_singletons,
        'num_checked_non_representative_pairs': num_checked_pairs,
        'max_hausdorff_cells': float(np.sqrt(max_dist_sq)),
        'max_hausdorff_m': float(np.sqrt(max_dist_sq) * result['voxel_res_m']),
        'num_union_missing_member_cell_refs': 0,
        'num_representative_only_missing_cell_refs': num_missing_representative_refs,
        'representative_only_missing_cell_ref_ratio': num_missing_representative_refs / num_original_refs,
        'representative_only_missing_merged_cell_ref_ratio': (
            num_missing_representative_refs / num_merged_member_cell_refs
            if num_merged_member_cell_refs else 0.0),
        'num_union_dictionary_cell_refs': num_union_cell_refs,
        'num_union_member_cell_refs': num_union_member_cell_refs,
        'union_dictionary_cell_ref_ratio': num_union_cell_refs / num_original_refs,
        'union_additional_member_cell_ref_ratio': (num_union_member_cell_refs - num_original_refs) / num_original_refs,
        'union_additional_pose_cell_ratio': summary(union_additional_ratios),
        'union_additional_merged_pose_cell_ratio': summary(merged_union_additional_ratios),
        'joint_max_abs_diff_rad': summary(joint_max_rad),
        'joint_l2_diff_rad': summary(joint_norm_rad),
        'max_tcp_diff_m': summary(tcp_max_m),
        'joint_diff_note': '周期補正なしの原入力関節差。',
        'elapsed_sec': time.monotonic() - started,
    })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_path', type=Path)
    parser.add_argument('assignment_path', type=Path)
    parser.add_argument('radius_cells', type=float)
    parser.add_argument('output_path', type=Path)
    args = parser.parse_args()
    result = verify(args.input_path, args.assignment_path, args.radius_cells)
    # 非有限値を拒否した検証結果の保存
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    args.output_path.parent.mkdir(parents=True, exist_ok=True)
    args.output_path.write_text(encoded + '\n')
    print(encoded, flush=True)


if __name__ == '__main__':
    main()

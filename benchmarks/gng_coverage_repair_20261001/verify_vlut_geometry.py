#!/usr/bin/env python3
"""リンクローカル占有と独立XML FKによる、保存VLUTの位置・集合検査。"""

import argparse
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as et

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def require(is_valid, message):
    if not is_valid:
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


def all_link_fk(urdf_path, names, angles):
    """本体adapter非依存のXML原点・軸・mimicからの全リンク剛体変換。"""
    document = et.parse(urdf_path).getroot()
    joints = {joint.get('name'): joint for joint in document.findall('joint')}
    links = {link.get('name') for link in document.findall('link')}
    children = {joint.find('child').get('link') for joint in joints.values()}
    roots = links - children
    require(len(roots) == 1, 'URDFルートの一意性不足')
    root = next(iter(roots))
    num_poses = len(angles)
    q_values = {name: angles[:, idx].astype(np.float64) for idx, name in enumerate(names)}
    has_resolution = set()

    def joint_values(name):
        if name in q_values:
            return q_values[name]
        require(name not in has_resolution, 'mimic参照の循環')
        has_resolution.add(name)
        mimic = joints[name].find('mimic')
        value = (joint_values(mimic.get('joint')) * float(mimic.get('multiplier', '1')) +
                 float(mimic.get('offset', '0'))) if mimic is not None else np.zeros(num_poses)
        q_values[name] = value
        has_resolution.remove(name)
        return value

    transforms = {root: np.broadcast_to(np.eye(4), (num_poses, 4, 4)).copy()}
    pending = list(joints.values())
    while pending:
        rest = []
        for joint in pending:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                rest.append(joint)
                continue
            child = joint.find('child').get('link')
            origin = joint.find('origin')
            fixed = np.eye(4)
            if origin is not None:
                fixed[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                fixed[:3, :3] = Rotation.from_euler(
                    'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            motion = np.broadcast_to(np.eye(4), (num_poses, 4, 4)).copy()
            kind = joint.get('type')
            if kind != 'fixed':
                axis_xml = joint.find('axis')
                axis = np.fromstring(axis_xml.get('xyz', '1 0 0'), sep=' ') if axis_xml is not None else np.array([1., 0., 0.])
                require(np.isfinite(axis).all() and np.linalg.norm(axis) > 0, '不正な関節軸')
                axis /= np.linalg.norm(axis)
                values = joint_values(joint.get('name'))
                if kind in ('continuous', 'revolute'):
                    motion[:, :3, :3] = Rotation.from_rotvec(values[:, None] * axis).as_matrix()
                elif kind == 'prismatic':
                    motion[:, :3, 3] = values[:, None] * axis
                else:
                    raise ValueError(f'未対応の関節型: {kind}')
            transforms[child] = transforms[parent] @ fixed @ motion
        require(len(rest) < len(pending), 'URDFリンク接続の循環または欠落')
        pending = rest
    require(set(transforms) == links, '未計算のURDFリンク')
    return transforms, root


def cell_set(points, resolution):
    return {tuple(int(value) for value in row) for row in np.floor(points / resolution).astype(np.int64)}


def compare_cells(world, actual, resolution):
    """格子境界から1e-7m以内だけを対象とする隣接セル許容。"""
    boundary_dist_th = 1e-7
    nominal = cell_set(world, resolution)
    allowed = set()
    possible_by_point = []
    num_boundary_points = 0
    for point in world:
        axis_cells = []
        has_boundary = False
        for coordinate in point:
            rounded = int(np.rint(coordinate / resolution))
            if abs(coordinate - rounded * resolution) <= boundary_dist_th:
                axis_cells.append((rounded - 1, rounded))
                has_boundary = True
            else:
                axis_cells.append((int(np.floor(coordinate / resolution)),))
        possible = set(itertools.product(*axis_cells))
        possible_by_point.append(possible)
        allowed.update(possible)
        num_boundary_points += has_boundary
    unexpected = actual - allowed
    unrepresented = [idx for idx, possible in enumerate(possible_by_point) if not (actual & possible)]
    is_exact = nominal == actual
    is_valid = not unexpected and not unrepresented and len(actual) <= len(world)
    return {'has_passed': is_valid, 'is_exact_match': is_exact,
            'num_local_points': len(world), 'num_nominal_cells': len(nominal),
            'num_actual_cells': len(actual), 'num_boundary_points': num_boundary_points,
            'num_nominal_missing_cells': len(nominal - actual),
            'num_nominal_extra_cells': len(actual - nominal),
            'num_unexpected_cells': len(unexpected), 'num_unrepresented_local_points': len(unrepresented),
            'unexpected_cell_examples': sorted(unexpected)[:10],
            'unrepresented_point_indices': unrepresented[:10],
            'boundary_dist_th_m': boundary_dist_th}


def decode_cells(values, schema):
    values = np.asarray(values, dtype=np.int64)
    x_shift, y_shift, z_shift = (schema[axis + '_shift'] for axis in ('x', 'y', 'z'))
    offset = schema['offset']
    y_mask = (1 << (x_shift - y_shift)) - 1
    z_mask = (1 << (y_shift - z_shift)) - 1
    return np.column_stack(((values >> x_shift) - offset,
                            ((values >> y_shift) & y_mask) - offset,
                            ((values >> z_shift) & z_mask) - offset))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--model', choices=('max', 'long'), required=True)
    parser.add_argument('--output', type=Path, required=True, help='保存GNG・VLUTを含む対象ディレクトリ')
    parser.add_argument('--local-voxels', type=Path)
    args = parser.parse_args()
    started = time.monotonic()
    root = args.root.resolve()
    output = args.output.resolve(strict=True)
    report_path = output / 'vlut_geometry_verification.json'
    require(not report_path.exists(), '既存検証結果への上書き拒否')
    local_path = args.local_voxels
    if local_path is None:
        local_path = output / 'local_voxels.json'
        if not local_path.exists():
            local_path = Path(__file__).with_name(f'local_voxels_{args.model}.json')
    local_path = local_path.resolve(strict=True)
    local = json.loads(local_path.read_text())
    require(local['num_link_slots'] == 22 and local['num_nonempty_links'] == 18, 'ローカルリンク件数の不一致')
    for relative, expected_hash in local['geometry_sha256'].items():
        require(sha256(root / relative) == expected_hash, f'ローカル占有生成後の形状変更: {relative}')
    for relative, expected_hash in local['voxelizer_source_sha256'].items():
        require(sha256(root / relative) == expected_hash, f'ローカル占有生成後の生成コード変更: {relative}')
    bench = root / 'benchmarks/voxel_pose_compression_20260930'
    export = load_module('geometry_export', bench / 'export_preview.py')
    prepare = load_module('geometry_prepare', bench / 'prepare.py')
    verify = load_module('geometry_decode', Path(__file__).with_name('verify_repair.py'))
    model_name = 'topo_dual_arm_max' + ('_long' if args.model == 'long' else '')
    config_path = root / 'gng_vlut_system/config' / (model_name + '.yaml')
    config = yaml.safe_load(config_path.read_text())['/**']['ros__parameters']
    urdf_path = root / Path(config['urdf_path']).relative_to('/ros2_ws/src')
    require(local['geometry_sha256'][str(urdf_path.relative_to(root))] == sha256(urdf_path), 'URDF指紋の不一致')
    names = prepare.link_names_from_urdf(config, urdf_path)
    require([link['name'] for link in local['links']] == names, 'trainerリンクID順序の不一致')
    require([link['link_id'] for link in local['links']] == list(range(22)), 'ローカルリンクIDの不一致')
    resolution = float(local['resolution_m'])
    require(resolution == config['gng']['vlut_resolution'], 'ローカル占有と現行VLUT設定解像度の不一致')
    original_path = root / 'gng_vlut_system/gng_results' / model_name / 'gng.bin'
    gng_path, vlut_path = output / 'gng.bin', output / 'vlut.bin'
    input_paths = [local_path, config_path, urdf_path, original_path, gng_path, vlut_path]
    input_hashes = {str(path): sha256(path) for path in input_paths}
    original = export.read_gng(original_path)
    repaired = export.read_gng(gng_path)
    ids, q, tcp = verify.decode_states(repaired)
    original_ids = sorted(original['node_ids'])
    added_ids = sorted(int(node_id) for node_id in set(ids) - set(original_ids))
    require(len(original_ids) >= 5 and len(added_ids) >= 5, '元5姿勢・追加5姿勢の件数不足')
    selected_ids = []
    for candidates in (original_ids, added_ids):
        selected_ids.extend(candidates[idx] for idx in np.linspace(0, len(candidates) - 1, 5, dtype=int))
    by_id = {int(node_id): idx for idx, node_id in enumerate(ids)}
    selected_indices = np.array([by_id[node_id] for node_id in selected_ids])
    joint_names = [f'{side}_joint{idx}' for side in ('L', 'R') for idx in range(1, 8)]
    transforms, frame = all_link_fk(urdf_path, joint_names, q[selected_indices])
    for layer, side in enumerate(('L', 'R')):
        error = np.linalg.norm(transforms[side + '_tcp'][:, :3, 3] - tcp[selected_indices, layer], axis=1)
        require(error.max() < 1e-5, '独立な全リンクFKと保存TCPの不一致')
    header, relations = export.read_vlut(vlut_path, set(ids))
    require(struct.unpack_from('<f', header, 8)[0] == float(np.float32(resolution)), 'VLUT保存解像度の不一致')
    masks = {(node_id, link_id): set() for node_id in selected_ids for link_id in range(22)}
    num_selected_records = 0
    for start in range(0, len(relations), 1000000):
        block = relations[start:start + 1000000]
        selected = block[np.isin(block['node'], selected_ids)]
        cells = decode_cells(selected['voxel'], config['voxel_idx_shift'])
        for relation, point in zip(selected, cells):
            require(0 <= relation['link'] < 22, '未知のVLUTリンクID')
            masks[(int(relation['node']), int(relation['link']))].add(tuple(int(value) for value in point))
        num_selected_records += len(selected)
    require(num_selected_records == sum(map(len, masks.values())), '同一ノード・リンク・ボクセルの重複')
    pairs = []
    num_empty_slot_violations = 0
    for pose_idx, node_id in enumerate(selected_ids):
        for link in local['links']:
            actual = masks[(node_id, link['link_id'])]
            points = np.asarray(link['centers'], dtype=np.float64).reshape(-1, 3)
            if len(points) == 0:
                num_empty_slot_violations += bool(actual)
                continue
            transform = transforms[link['name']][pose_idx]
            world = points @ transform[:3, :3].T + transform[:3, 3]
            pair = compare_cells(world, actual, resolution)
            pair.update(node_id=node_id, link_id=link['link_id'], link_name=link['name'],
                        link_origin_m=transform[:3, 3].tolist(),
                        min_transformed_local_m=world.min(axis=0).tolist(),
                        max_transformed_local_m=world.max(axis=0).tolist())
            pairs.append(pair)
    require(input_hashes == {str(path): sha256(path) for path in input_paths}, '検証中の入力変更')
    num_failed = sum(not pair['has_passed'] for pair in pairs)
    result = {'model': args.model, 'has_passed': num_failed == 0 and num_empty_slot_violations == 0,
              'num_original_poses': 5, 'num_added_poses': 5, 'selected_node_ids': selected_ids,
              'num_compared_link_masks': len(pairs),
              'num_exact_match_pairs': sum(pair['is_exact_match'] for pair in pairs),
              'num_boundary_tolerated_pairs': sum(pair['has_passed'] and not pair['is_exact_match'] for pair in pairs),
              'num_boundary_points': sum(pair['num_boundary_points'] for pair in pairs),
              'num_failed_pairs': num_failed, 'num_empty_slot_violations': num_empty_slot_violations,
              'boundary_dist_th_m': 1e-7, 'resolution_m': resolution, 'urdf_root_link': frame,
              'input_sha256': input_hashes, 'geometry_sha256': local['geometry_sha256'],
              'has_input_mutation': False, 'pairs': pairs,
              'scope': '10姿勢18リンクの局所占有集合と独立全リンクFKの照合。形状ボクセル化は本体共通、FKと世界格子化経路は独立。全姿勢の保証ではない。',
              'elapsed_sec': time.monotonic() - started}
    with report_path.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'pairs'}, ensure_ascii=False), flush=True)
    require(result['has_passed'], f'VLUT位置照合の失敗: {report_path}')


if __name__ == '__main__':
    main()

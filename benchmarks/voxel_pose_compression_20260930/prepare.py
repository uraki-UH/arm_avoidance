"""保存済GNG・VLUTからのリンク別ボクセル姿勢データ抽出。"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as et

import numpy as np
import yaml


def sha256(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def read_gng(path):
    data = path.read_bytes()
    offset = 0

    def read(fmt):
        nonlocal offset
        result = struct.unpack_from('<' + fmt, data, offset)
        offset += struct.calcsize('<' + fmt)
        return result[0] if len(result) == 1 else result

    def vector():
        rows, cols = read('qq')
        assert 0 < rows * cols < 1000
        return np.array(read('f' * (rows * cols)), dtype='<f4', ndmin=1)

    assert read('I') == 9
    num_layers, num_nodes = read('ii')
    assert num_layers == 2 and 0 < num_nodes <= 10000
    ids, angles, points = [], [], []
    for _ in range(num_nodes):
        ids.append(read('i'))
        read('ff')
        angles.append(vector())
        vector()
        assert read('i') == num_layers
        points.append([vector() for _ in range(num_layers)])
        read('i?????')
        vector()
        read('fff?ff?')
    assert len(set(ids)) == num_nodes
    id_set = set(ids)
    num_edges = []
    for _ in range(num_layers + 1):
        count = read('i')
        assert 0 < count < 10000000
        num_edges.append(count)
        for _ in range(count):
            first, second, age, is_active = read('iii?')
            assert first in id_set and second in id_set and first != second and age >= 0
    assert offset == len(data)
    angles, points = np.asarray(angles), np.asarray(points)
    assert angles.shape == (num_nodes, 14) and points.shape == (num_nodes, 2, 3)
    assert np.isfinite(angles).all() and np.isfinite(points).all()
    return np.asarray(ids, dtype='<i4'), angles, points, num_edges


def link_names_from_urdf(params, urdf_path):
    """trainerの経路・末端分岐・除外順序に対応するリンク番号の再現。"""
    urdf = et.parse(urdf_path).getroot()
    joints = sorted(urdf.findall('joint'), key=lambda item: item.get('name'))
    by_child = {joint.find('child').get('link'): joint for joint in joints}
    children = {}
    for joint in joints:
        children.setdefault(joint.find('parent').get('link'), []).append(joint.find('child').get('link'))

    def descendants(start):
        result = []
        for child in children.get(start, []):
            result.append(child)
            result.extend(descendants(child))
        return result

    names, excludes = [], set()
    for profile_name in params['gng']['profile_names'].split(','):
        profile = params['gng']['profiles'][profile_name]
        root = profile['root']
        excludes.update(profile.get('voxel_exclude', []))
        for leaf in profile['eef'].split(','):
            current = leaf
            reverse = []
            while current != root:
                reverse.append(current)
                current = by_child[current].find('parent').get('link')
            branch = [root] + list(reversed(reverse))
            parent = by_child[leaf].find('parent').get('link')
            if len(children.get(parent, [])) >= 2:
                for child in children[parent]:
                    branch.extend(descendants(child))
                    branch.append(child)
            for name in branch:
                if name not in names:
                    names.append(name)
    return [name for name in names if name not in excludes]


def summarize_counts(values):
    return {'min': int(values.min()), 'max': int(values.max()),
            'mean': float(values.mean()), 'median': float(np.median(values))}


def prepare(root, output, model, short_name):
    started = time.monotonic()
    folder = root / 'gng_vlut_system/gng_results' / model
    config = root / 'gng_vlut_system/config' / (model + '.yaml')
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    urdf_path = root / Path(params['urdf_path']).relative_to('/ros2_ws/src')
    ids, angles, points, num_edges = read_gng(folder / 'gng.bin')
    names = link_names_from_urdf(params, urdf_path)
    vlut_path = folder / 'vlut.bin'
    with vlut_path.open('rb') as stream:
        header = stream.read(44)
    magic, version, resolution, *bounds = struct.unpack_from('<IIf6f', header)
    assert magic == int.from_bytes(b'VLUT', 'big') and version == 2
    assert np.isfinite(bounds).all() and np.all(np.array(bounds[:3]) <= bounds[3:])
    assert abs(resolution - .02) < 1e-7
    num_relations = struct.unpack_from('<Q', header, 36)[0]
    dtype = np.dtype([('voxel', '<i8'), ('node', '<i4'), ('dist', '<f4'), ('link', '<i4')])
    assert num_relations > 0 and vlut_path.stat().st_size == 44 + num_relations * dtype.itemsize
    relations = np.memmap(vlut_path, dtype=dtype, mode='r', offset=44, shape=(num_relations,))
    assert np.isin(relations['node'], ids).all()
    assert set(np.unique(relations['node'])) == set(ids)
    assert np.isfinite(relations['dist']).all() and (relations['dist'] >= 0).all()
    assert (relations['link'] >= 0).all()
    assert (relations['voxel'][1:] >= relations['voxel'][:-1]).all()
    num_links = int(relations['link'].max()) + 1
    assert num_links == len(names)
    # ノード番号・リンク番号・ボクセルID順の整列
    order = np.lexsort((relations['voxel'], relations['link'], relations['node']))
    node_values = np.asarray(relations['node'][order], dtype=np.int64)
    link_values = np.asarray(relations['link'][order], dtype=np.int64)
    voxels = np.asarray(relations['voxel'][order], dtype=np.int64)
    duplicate = ((node_values[1:] == node_values[:-1]) & (link_values[1:] == link_values[:-1]) &
                 (voxels[1:] == voxels[:-1]))
    num_duplicates = int(duplicate.sum())
    assert num_duplicates == 0
    schema = params['voxel_idx_shift']
    x_shift, y_shift, z_shift = (schema[name + '_shift'] for name in ('x', 'y', 'z'))
    offset = schema['offset']
    y_mask = (1 << (x_shift - y_shift)) - 1
    z_mask = (1 << (y_shift - z_shift)) - 1
    cells = np.column_stack(((voxels >> x_shift) - offset,
                             ((voxels >> y_shift) & y_mask) - offset,
                             ((voxels >> z_shift) & z_mask) - offset)).astype('<i4')
    packed = ((cells[:, 0].astype(np.int64) + offset) << x_shift) | \
             ((cells[:, 1].astype(np.int64) + offset) << y_shift) | \
             ((cells[:, 2].astype(np.int64) + offset) << z_shift)
    assert np.array_equal(packed, voxels)
    keys = node_values * num_links + link_values
    boundaries = np.r_[0, np.flatnonzero(keys[1:] != keys[:-1]) + 1, len(keys)]
    groups = {int(keys[first]): (int(first), int(last)) for first, last in zip(boundaries[:-1], boundaries[1:])}
    counts = np.zeros((len(ids), num_links), dtype=np.int64)
    binary_path = output / (short_name + '.voxpose')
    with binary_path.open('wb') as stream:
        stream.write(struct.pack('<8sIIIIf', b'VOXPOSE1', len(ids), num_links, angles.shape[1], points.shape[1], resolution))
        for node_idx, node_id in enumerate(ids):
            stream.write(struct.pack('<i', int(node_id)))
            stream.write(angles[node_idx].tobytes())
            stream.write(points[node_idx].tobytes())
            for link_idx in range(num_links):
                first, last = groups.get(int(node_id) * num_links + link_idx, (0, 0))
                count = last - first
                counts[node_idx, link_idx] = count
                stream.write(struct.pack('<I', count))
                if count:
                    stream.write(cells[first:last].tobytes())
    assert int(counts.sum()) == num_relations
    active_link_ids = np.flatnonzero((counts > 0).any(axis=0))
    all_active_present = (counts[:, active_link_ids] > 0).all(axis=1)
    expected_size = 28 + len(ids) * (4 + 4 * angles.shape[1] + 12 * points.shape[1] + 4 * num_links) + 12 * num_relations
    assert binary_path.stat().st_size == expected_size
    # 出力ヘッダと全ノードの関節角・TCP、全セルの整列状態の再読込検査
    with binary_path.open('rb') as stream:
        assert stream.read(28) == struct.pack('<8sIIIIf', b'VOXPOSE1', len(ids), num_links, 14, 2, resolution)
        for node_idx, node_id in enumerate(ids):
            assert struct.unpack('<i', stream.read(4))[0] == node_id
            assert stream.read(56) == angles[node_idx].tobytes()
            assert stream.read(24) == points[node_idx].tobytes()
            for link_idx in range(num_links):
                count = struct.unpack('<I', stream.read(4))[0]
                assert count == counts[node_idx, link_idx]
                first, last = groups.get(int(node_id) * num_links + link_idx, (0, 0))
                assert stream.read(12 * count) == cells[first:last].tobytes()
        assert stream.read(1) == b''
    result = {
        'model': model, 'output': str(binary_path), 'format': 'VOXPOSE1',
        'num_nodes': len(ids), 'num_links': num_links, 'angle_dim': 14, 'num_coord_layers': 2,
        'resolution_m': resolution, 'voxel_idx_shift': schema,
        'num_relations': num_relations, 'num_duplicate_node_link_cells': num_duplicates,
        'num_unique_world_cells': int(len(np.unique(voxels))), 'num_edges': num_edges,
        'min_cell_xyz': cells.min(axis=0).tolist(), 'max_cell_xyz': cells.max(axis=0).tolist(),
        'vlut_bounds_m': [bounds[:3], bounds[3:]],
        'node_cell_counts': summarize_counts(counts.sum(axis=1)),
        'active_link_ids': active_link_ids.tolist(),
        'num_nodes_missing_any_active_link': int((~all_active_present).sum()),
        'link_counts': [{'link_id': link_idx, 'name': name, 'num_empty_nodes': int((counts[:, link_idx] == 0).sum()),
                         **summarize_counts(counts[:, link_idx])} for link_idx, name in enumerate(names)],
        'relation_dist_min_m': float(relations['dist'].min()),
        'relation_dist_max_m': float(relations['dist'].max()),
        'input_sha256': {str(path): sha256(path) for path in (folder / 'gng.bin', vlut_path, config, urdf_path)},
        'output_sha256': sha256(binary_path),
        'num_output_bytes': binary_path.stat().st_size,
        'elapsed_sec': time.monotonic() - started,
        'checks': ['GNG v9全レコード・辺の参照検査', 'VLUT v2ノード参照・距離有限性・整列検査',
                   'URDFとtrainer順序に基づくリンクID再現', 'ボクセルIDの復号・再符号化一致',
                   '全ノード・全リンク・全セルの出力再読込一致'],
        'limitation': '保存済VLUTの表面占有。姿勢ごとの再ボクセル化時間を含まない。',
    }
    (output / (short_name + '.meta.json')).write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({key: result[key] for key in ('model', 'num_nodes', 'num_links', 'num_relations',
                                                  'node_cell_counts', 'num_nodes_missing_any_active_link',
                                                  'elapsed_sec')}, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--output', type=Path, default=Path('/tmp/voxel_pose_compression_20260930/data'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for short_name, model in [('max', 'topo_dual_arm_max'), ('long', 'topo_dual_arm_max_long')]:
        prepare(args.root, args.output, model, short_name)


if __name__ == '__main__':
    main()

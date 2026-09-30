"""元の関節姿勢と占有を保持する代表ノードの Viewer 用抽出。"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct

import numpy as np


relation_dtype = np.dtype([
    ('voxel', '<i8'), ('node', '<i4'), ('dist', '<f4'), ('link', '<i4'),
])


def require(is_valid, message):
    if not is_valid:
        raise ValueError(message)


def sha256(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def read_gng(path):
    """GNG v9 の可変長レコード境界と参照の検査。辺数ゼロの許容。"""
    data = path.read_bytes()
    offset = 0

    def read(fmt):
        nonlocal offset
        size = struct.calcsize('<' + fmt)
        require(offset + size <= len(data), f'GNG の途中終端: offset={offset}')
        values = struct.unpack_from('<' + fmt, data, offset)
        offset += size
        return values[0] if len(values) == 1 else values

    def vector():
        nonlocal offset
        rows, cols = read('qq')
        require(rows > 0 and cols > 0, 'GNG ベクトルの不正な形状')
        num_values = rows * cols
        num_bytes = 4 * num_values
        require(offset + num_bytes <= len(data), 'GNG ベクトルの途中終端')
        values = np.frombuffer(data, dtype='<f4', count=num_values, offset=offset)
        offset += num_bytes
        return values

    require(read('I') == 9, 'GNG v9 以外の入力')
    num_layers, num_nodes = read('ii')
    require(num_layers == 2, '座標層数が 2 以外の GNG')
    require(0 <= num_nodes <= len(data) // 12, 'GNG ノード数の不整合')
    header = data[:offset]
    nodes = []
    node_ids = set()
    for _ in range(num_nodes):
        node_start = offset
        node_id = read('i')
        require(node_id not in node_ids, f'GNG ノードIDの重複: {node_id}')
        node_ids.add(node_id)
        read('ff')
        angles = vector()
        require(np.isfinite(angles).all(), f'非有限の関節角: node={node_id}')
        vector()
        require(read('i') == num_layers, f'座標層数の不整合: node={node_id}')
        for _ in range(num_layers):
            point = vector()
            require(np.isfinite(point).all(), f'非有限の座標: node={node_id}')
        read('i?????')
        vector()
        read('fff?ff?')
        nodes.append({'id': node_id, 'record': data[node_start:offset]})
    edge_layers = []
    for layer_idx in range(num_layers + 1):
        num_edges = read('i')
        require(0 <= num_edges <= (len(data) - offset) // 13,
                f'GNG 辺数の不整合: layer={layer_idx}')
        edges = []
        for _ in range(num_edges):
            edge_start = offset
            first_id, second_id, age, is_active = read('iii?')
            require(first_id in node_ids and second_id in node_ids,
                    f'GNG 辺の不正な参照: layer={layer_idx}')
            require(first_id != second_id and age >= 0,
                    f'GNG 辺の不正な内容: layer={layer_idx}')
            edges.append({'first_id': first_id, 'second_id': second_id,
                          'record': data[edge_start:offset]})
        edge_layers.append(edges)
    require(offset == len(data), 'GNG 末尾の余分なデータ')
    return {'header': header, 'num_layers': num_layers, 'nodes': nodes,
            'node_ids': node_ids, 'edge_layers': edge_layers}


def read_assignments(path, node_ids):
    """全元IDの一意性・代表自己参照・グループ対応の検査。"""
    required = {'original_id', 'representative_id', 'is_representative'}
    assignments = {}
    with path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames is not None and required <= set(reader.fieldnames),
                'CSV の必須列不足')
        require(len(reader.fieldnames) == len(set(reader.fieldnames)), 'CSV 列名の重複')
        has_group_idx = 'group_idx' in reader.fieldnames
        for line_num, row in enumerate(reader, start=2):
            require(None not in row and all(row[name] is not None for name in required),
                    f'CSV 列数の不整合: line={line_num}')
            node_id = int(row['original_id'])
            representative_id = int(row['representative_id'])
            require(row['is_representative'] in ('0', '1'),
                    f'CSV 代表フラグの不正値: line={line_num}')
            is_representative = row['is_representative'] == '1'
            require(node_id in node_ids and representative_id in node_ids,
                    f'CSV の未知ノードID: line={line_num}')
            require(node_id not in assignments, f'CSV 元IDの重複: {node_id}')
            require(is_representative == (node_id == representative_id),
                    f'CSV 代表フラグと自己参照の不整合: node={node_id}')
            group_idx = int(row['group_idx']) if has_group_idx else None
            require(group_idx is None or group_idx >= 0,
                    f'CSV グループ番号の不正値: line={line_num}')
            assignments[node_id] = {'representative_id': representative_id,
                                    'is_representative': is_representative,
                                    'group_idx': group_idx}
    require(set(assignments) == node_ids, 'CSV と GNG の元ノードID集合の不一致')
    representative_ids = {node_id for node_id, row in assignments.items()
                          if row['is_representative']}
    group_representatives = {}
    for node_id, row in assignments.items():
        representative_id = row['representative_id']
        require(representative_id in representative_ids,
                f'CSV の代表参照先が非代表: node={node_id}')
        representative_row = assignments[representative_id]
        require(representative_row['representative_id'] == representative_id,
                f'CSV の代表自己参照不足: node={representative_id}')
        if has_group_idx:
            group_idx = row['group_idx']
            require(group_idx == representative_row['group_idx'],
                    f'CSV グループ番号と代表の不整合: node={node_id}')
            existing = group_representatives.setdefault(group_idx, representative_id)
            require(existing == representative_id, f'CSV グループ内の複数代表: group={group_idx}')
    return representative_ids


def read_vlut(path, node_ids):
    """VLUT v2 ヘッダ・参照・距離・元レコード順序の検査。"""
    with path.open('rb') as stream:
        header = stream.read(44)
    require(len(header) == 44, 'VLUT ヘッダの途中終端')
    magic, version, resolution, *remaining = struct.unpack('<IIf6fQ', header)
    bounds = remaining[:6]
    num_relations = remaining[6]
    require(magic == int.from_bytes(b'VLUT', 'big') and version == 2,
            'VLUT v2 以外の入力')
    require(np.isfinite(resolution) and resolution > 0 and np.isfinite(bounds).all(),
            'VLUT ヘッダの不正な実数値')
    require(np.all(np.asarray(bounds[:3]) <= np.asarray(bounds[3:])),
            'VLUT 領域の不整合')
    require(path.stat().st_size == 44 + num_relations * relation_dtype.itemsize,
            'VLUT レコード数とファイル長の不整合')
    if num_relations:
        relations = np.memmap(path, dtype=relation_dtype, mode='r',
                              offset=44, shape=(num_relations,))
    else:
        relations = np.empty(0, dtype=relation_dtype)
    require(np.isin(relations['node'], list(node_ids)).all(), 'VLUT の未知ノード参照')
    require(np.isfinite(relations['dist']).all() and (relations['dist'] >= 0).all(),
            'VLUT 距離の不正値')
    require((relations['link'] >= 0).all(), 'VLUT リンクIDの不正値')
    require((relations['voxel'][1:] >= relations['voxel'][:-1]).all(),
            'VLUT のボクセル順序不整合')
    return header, relations


def export_preview(gng_path, vlut_path, assignments_path, output):
    require(not output.exists() and not output.is_symlink(),
            f'既存の出力先への書込み拒否: {output}')
    gng_path = gng_path.resolve(strict=True)
    vlut_path = vlut_path.resolve(strict=True)
    assignments_path = assignments_path.resolve(strict=True)
    input_paths = {'gng': gng_path, 'vlut': vlut_path, 'assignments': assignments_path}
    input_sha256 = {name: sha256(path) for name, path in input_paths.items()}
    original = read_gng(gng_path)
    representative_ids = read_assignments(assignments_path, original['node_ids'])
    selected_nodes = [node for node in original['nodes'] if node['id'] in representative_ids]
    selected_edges = [[edge for edge in edges
                       if edge['first_id'] in representative_ids and
                       edge['second_id'] in representative_ids]
                      for edges in original['edge_layers']]
    vlut_header, original_relations = read_vlut(vlut_path, original['node_ids'])
    selected_relations = original_relations[np.isin(original_relations['node'],
                                                    list(representative_ids))]
    # mkdir の排他性による検査後の既存出力先との競合回避。
    output.mkdir(parents=True, exist_ok=False)
    output_gng = output / 'gng.bin'
    output_vlut = output / 'vlut.bin'
    gng_header = bytearray(original['header'])
    struct.pack_into('<i', gng_header, 8, len(selected_nodes))
    with output_gng.open('xb') as stream:
        stream.write(gng_header)
        for node in selected_nodes:
            stream.write(node['record'])
        for edges in selected_edges:
            stream.write(struct.pack('<i', len(edges)))
            for edge in edges:
                stream.write(edge['record'])
    new_vlut_header = bytearray(vlut_header)
    struct.pack_into('<Q', new_vlut_header, 36, len(selected_relations))
    with output_vlut.open('xb') as stream:
        stream.write(new_vlut_header)
        selected_relations.tofile(stream)

    # 出力の再読込による生バイト保存と元データ部分集合の検査。
    reread = read_gng(output_gng)
    require(reread['node_ids'] == representative_ids, '再読込した代表IDの不一致')
    require(reread['header'] == bytes(gng_header), '再読込した GNG ヘッダの不一致')
    require(reread['nodes'] == selected_nodes, '元ノードレコードの生バイト不一致')
    require(reread['edge_layers'] == selected_edges, '元辺レコードの生バイト不一致')
    for layer_idx, edges in enumerate(reread['edge_layers']):
        original_records = {edge['record'] for edge in original['edge_layers'][layer_idx]}
        require(all(edge['record'] in original_records for edge in edges),
                f'元辺部分集合の検証失敗: layer={layer_idx}')
    reread_vlut_header, reread_relations = read_vlut(output_vlut, representative_ids)
    require(reread_vlut_header == bytes(new_vlut_header), '再読込した VLUT ヘッダの不一致')
    require(np.array_equal(reread_relations.view(np.uint8), selected_relations.view(np.uint8)),
            '元 VLUT レコードの生バイト・順序不一致')
    # 読込中の入力差替えの検出。
    require(input_sha256 == {name: sha256(path) for name, path in input_paths.items()},
            '処理中の入力ファイル変更')
    result = {
        'representation': 'representative_original_occupancy',
        'input_paths': {name: str(path) for name, path in input_paths.items()},
        'input_sha256': input_sha256,
        'output_sha256': {'gng': sha256(output_gng), 'vlut': sha256(output_vlut)},
        'num_original_nodes': len(original['nodes']),
        'num_output_nodes': len(selected_nodes),
        'num_original_edges': [len(edges) for edges in original['edge_layers']],
        'num_output_edges': [len(edges) for edges in selected_edges],
        'edge_layer_order': ['angle', 'coord_0', 'coord_1'],
        'num_original_vlut_refs': len(original_relations),
        'num_output_vlut_refs': len(selected_relations),
        'num_representatives_with_vlut_refs': int(len(np.unique(selected_relations['node']))),
        'checks': [
            'CSV と全元ノードID集合の一致・一意性',
            '代表フラグ・代表自己参照・グループ対応の一致',
            '代表ノードの全生レコードの再読込一致',
            '各層の元辺部分集合・辺生レコードの再読込一致',
            '代表IDで抽出した全 VLUT 生レコード・順序の再読込一致',
            '入力ファイル SHA-256 の処理前後一致',
        ],
        'limitations': [
            '表示対象は代表実姿勢の元占有のみ。メンバー占有の集合和を含まない。',
            '辺は元辺の誘導部分グラフのみ。グループ間の新規辺や経路保証を含まない。',
        ],
    }
    with (output / 'preview.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gng', required=True, type=Path)
    parser.add_argument('--vlut', required=True, type=Path)
    parser.add_argument('--assignments', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    export_preview(args.gng, args.vlut, args.assignments, args.output)


if __name__ == '__main__':
    main()

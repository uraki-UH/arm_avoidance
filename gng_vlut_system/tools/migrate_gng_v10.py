#!/usr/bin/env python3
"""旧GNGの一括移行専用ツール。通常の読込み経路から独立した旧形式処理。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import tempfile


binary_format_version = 10


class reader:
    def __init__(self, data):
        self.data = data
        self.offset = 0

    def take(self, num_bytes):
        end = self.offset + num_bytes
        if num_bytes < 0 or end > len(self.data):
            raise ValueError('GNGデータの欠損')
        value = self.data[self.offset:end]
        self.offset = end
        return value

    def integer(self):
        return struct.unpack('<i', self.take(4))[0]

    def eigen(self):
        shape = self.take(16)
        rows, cols = struct.unpack('<qq', shape)
        if not 0 <= rows <= 4096 or not 0 <= cols <= 4096 or rows * cols > 4096:
            raise ValueError('GNG配列の次元が不正です')
        return shape + self.take(rows * cols * 4)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def convert(data):
    source = reader(data)
    version = source.integer()
    if not 5 <= version <= binary_format_version:
        raise ValueError(f'移行対象外のGNGバージョン: {version}')
    num_layers = source.integer() if version >= 6 else 1
    num_nodes = source.integer()
    if not 1 <= num_layers <= 16 or not 0 <= num_nodes <= 1000000:
        raise ValueError('GNG層数・ノード数が不正です')
    output = bytearray(struct.pack('<Iii', binary_format_version, num_layers, num_nodes))
    geometry = hashlib.sha256()
    geometry.update(struct.pack('<ii', num_layers, num_nodes))
    node_ids = set()
    for _ in range(num_nodes):
        prefix = source.take(12)
        node_id, = struct.unpack_from('<i', prefix)
        if not 0 <= node_id < 1000000 or node_id in node_ids:
            raise ValueError('GNGノードIDの範囲外・重複')
        node_ids.add(node_id)
        angles, position = source.eigen(), source.eigen()
        num_coords = source.integer() if version >= 6 else 1
        if not 0 <= num_coords <= num_layers:
            raise ValueError('GNG座標層数が不正です')
        coords = [source.eigen() for _ in range(num_coords)] if version >= 6 else [position]
        level = source.take(4)
        if version == binary_format_version:
            flags = source.take(1)
            if flags[0] & 0xE0:
                raise ValueError('GNG状態フラグの予約ビットが不正です')
        else:
            old_flags = source.take(5)
            if any(value not in (0, 1) for value in old_flags):
                raise ValueError('GNG状態フラグが不正です')
            flags = bytes([sum(value << bit for bit, value in enumerate(old_flags))])
        direction = source.eigen()
        if version == binary_format_version:
            metrics = source.take(30)
            min_singular_value, joint_limit_score = metrics[8:12], metrics[12:16]
            dynamic_manipulability = metrics[17:21]
        else:
            # 復元不能なdouble部分保存値の破棄。残る評価値のバイト列保持
            source.take(4)
            min_singular_value, joint_limit_score = source.take(4), source.take(4)
            if version <= 8:
                source.take(4)
            source.take(1)
            dynamic_manipulability = source.take(4)
            if version >= 8:
                source.take(5)
            metrics = (struct.pack('<d', 0.) + min_singular_value + joint_limit_score + b'\0'
                       + dynamic_manipulability + struct.pack('<d?', 0., False))
            if version < 7:
                num_positions = source.integer()
                if not 0 <= num_positions <= 4096:
                    raise ValueError('GNG関節位置数が不正です')
                for _ in range(num_positions):
                    source.eigen()
        record = (prefix + angles + position + struct.pack('<i', num_coords)
                  + b''.join(coords) + level + flags + direction)
        output.extend(record + metrics)
        geometry.update(record + min_singular_value + joint_limit_score + dynamic_manipulability)
    edge_start = source.offset
    edge_counts = []
    for _ in range(num_layers + 1):
        num_edges = source.integer()
        if not 0 <= num_edges <= (len(data) - source.offset) // 13:
            raise ValueError('GNG接続数・データ長が不正です')
        edge_counts.append(num_edges)
        source.take(num_edges * 13)
    if source.offset != len(data):
        raise ValueError('GNG末尾に未処理のデータがあります')
    edges = data[edge_start:]
    output.extend(edges)
    return bytes(output), {'source_version': version, 'num_nodes': num_nodes,
        'num_coord_layers': num_layers, 'edge_counts': edge_counts,
        'geometry_sha256': geometry.hexdigest(), 'edges_sha256': digest(edges)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+', type=Path)
    parser.add_argument('--backup-dir', required=True, type=Path)
    args = parser.parse_args()
    backup_dir = args.backup_dir.resolve()
    if backup_dir.exists():
        raise ValueError('新しい退避ディレクトリを指定してください')
    paths = sorted({item.resolve() for path in args.paths
                    for item in (path.rglob('gng.bin') if path.is_dir() else [path])})
    if not paths:
        raise ValueError('移行対象のGNGがありません')
    pending = []
    for path in paths:
        if not os.access(path.parent, os.W_OK):
            raise PermissionError(f'原子的な置換に必要なディレクトリ書込み権限なし: {path.parent}')
        before = path.read_bytes()
        after, info = convert(before)
        verified, check = convert(after)
        if verified != after or any(info[key] != check[key] for key in
                ('num_nodes', 'num_coord_layers', 'edge_counts', 'geometry_sha256', 'edges_sha256')):
            raise ValueError(f'変換前後の不一致: {path}')
        pending.append((path, before, after, info))
    backup_dir.mkdir(parents=True)
    manifest = []
    for idx, (path, before, after, info) in enumerate(pending):
        if path.read_bytes() != before:
            raise ValueError(f'移行準備中の外部更新: {path}')
        backup = backup_dir / f'{idx:03d}_{path.parent.name}_gng.bin'
        shutil.copy2(path, backup)
        if backup.read_bytes() != before:
            raise ValueError(f'元データ退避の不一致: {path}')
        if before != after:
            # 同一ディレクトリ内の一時ファイルによる原子的な置換
            temp_path = None
            try:
                with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.gng_v10_', delete=False) as stream:
                    temp_path = Path(stream.name)
                    stream.write(after)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.chmod(temp_path, path.stat().st_mode)
                if path.read_bytes() != before:
                    raise ValueError(f'移行直前の外部更新: {path}')
                temp_path.replace(path)
            finally:
                if temp_path is not None and temp_path.exists():
                    temp_path.unlink()
        if path.read_bytes() != after:
            raise ValueError(f'書込み後の不一致: {path}')
        manifest.append(dict(info, path=str(path), backup=str(backup),
            source_sha256=digest(before), target_sha256=digest(after),
            source_bytes=len(before), target_bytes=len(after)))
        (backup_dir / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'num_files': len(manifest), 'backup_dir': str(backup_dir),
                      'num_nodes': sum(item['num_nodes'] for item in manifest)}, ensure_ascii=False))


if __name__ == '__main__':
    main()

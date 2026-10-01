"""production YAMLの明示自己干渉除外をCSVへ抽出、入力・出力SHA保存。"""
import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import re
import tempfile

import yaml


def require(is_valid, message):
    if not is_valid:
        raise ValueError(message)


def extract_pairs(config):
    collision = config['/**']['ros__parameters']['collision']
    require(collision.get('enable_self_collision') is True, '自己干渉検査の無効な設定')
    require(collision.get('apply_self_collision_exclusion_pairs') in (True, False), '除外有効フラグの欠落')
    entries = collision.get('self_collision_exclusion_pairs', [])
    require(isinstance(entries, list), '除外ペア配列の不正な形式')
    pairs = []
    seen = set()
    for entry in entries:
        require(isinstance(entry, str), '除外ペアの不正な文字列')
        values = entry.split('|')
        require(len(values) == 2 and all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', value) for value in values),
                'first|second形式以外の除外ペア')
        require(values[0] != values[1], '同一リンクの除外ペア')
        ordered = tuple(sorted(values))
        require(ordered not in seen, '重複した除外ペア')
        seen.add(ordered)
        pairs.append(values)
    return (pairs if collision['apply_self_collision_exclusion_pairs'] else []), collision


def write_atomic_new(path, data):
    """同一ディレクトリ内の完成済み一時ファイルから、上書きなしの公開。"""
    descriptor, temporary_name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.config.read_bytes()
    pairs, collision = extract_pairs(yaml.safe_load(source))
    metadata_path = args.output.with_suffix(args.output.suffix + '.json')
    require(not args.output.exists() and not args.output.is_symlink() and
            not metadata_path.exists() and not metadata_path.is_symlink(), '既存出力への上書き拒否')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stream = io.StringIO(newline='')
    writer = csv.writer(stream, lineterminator='\n')
    writer.writerow(['first', 'second'])
    writer.writerows(pairs)
    data = stream.getvalue().encode('utf-8')
    metadata = {'source_config': str(args.config.resolve()), 'source_sha256': hashlib.sha256(source).hexdigest(),
                'csv_path': str(args.output.resolve()), 'csv_sha256': hashlib.sha256(data).hexdigest(),
                'is_exclusion_enabled': collision['apply_self_collision_exclusion_pairs'],
                'num_pairs': len(pairs), 'pairs': pairs, 'collision_voxel_size_m': collision.get('voxel_size')}
    require(args.config.read_bytes() == source, '抽出中の設定ファイル変更')
    write_atomic_new(args.output, data)
    try:
        write_atomic_new(metadata_path, (json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8'))
    except BaseException:
        args.output.unlink()
        raise
    print(json.dumps(metadata, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()

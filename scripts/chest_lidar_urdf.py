#!/usr/bin/env python3
"""胸部LiDARの45度／60度URDF生成。元URDFと既存出力の上書きなし。"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as et


default_urdf = Path(__file__).resolve().parents[1] / 'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf'


def vector_text(values):
    return ' '.join(f'{value:.12g}' for value in values)


def build_variant(source, angle_deg):
    """ブラケット・本体位置・傾斜の同時選択とメッシュ参照の解決。"""
    source = Path(source).resolve()
    metadata = json.loads((source.parent / 'meshes/chest_lidar.json').read_text(encoding='utf-8'))
    key = str(angle_deg)
    if key not in metadata['variants']:
        raise ValueError('取り付け角度は45度または60度が必要です')
    preset = metadata['variants'][key]
    parser = et.XMLParser(target=et.TreeBuilder(insert_comments=True))
    root = et.parse(source, parser).getroot()
    mount_joint = root.find("./joint[@name='chest_lidar_mount_fixed']")
    sensor_joint = root.find("./joint[@name='chest_lidar_fixed']")
    mount_link = root.find("./link[@name='chest_lidar_mount_link']")
    if mount_joint is None or sensor_joint is None or mount_link is None:
        raise ValueError('胸部LiDARを含むURDFが必要です')
    if (mount_joint.get('type') != 'fixed' or sensor_joint.get('type') != 'fixed'
            or mount_joint.find('parent').get('link') != metadata['parent_link']
            or sensor_joint.find('parent').get('link') != mount_link.get('name')):
        raise ValueError('胸部LiDARの固定リンク構成が一致しません')
    mount_joint.find('origin').set('xyz', vector_text(preset['mount_xyz_m']))
    mount_joint.find('origin').set('rpy', '0 0 0')
    sensor_joint.find('origin').set('xyz', vector_text(preset['sensor_xyz_m']))
    sensor_joint.find('origin').set('rpy', vector_text(preset['sensor_rpy_rad']))
    for mesh in mount_link.findall('.//mesh'):
        mesh.set('filename', preset['mount_mesh']['file'])
    # 出力ディレクトリによらない元URDF基準の資産参照
    for mesh in root.findall('.//mesh'):
        name = mesh.get('filename', '')
        if name.startswith(('package://', 'file://')):
            continue
        path = (source.parent / name).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set('filename', path.as_uri())
    return et.tostring(root, encoding='unicode', xml_declaration=True) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--angle', type=int, choices=(45, 60), required=True)
    parser.add_argument('--source', type=Path, default=default_urdf)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.resolve() == args.source.resolve():
        parser.error('元URDFと異なる出力先が必要です')
    try:
        text = build_variant(args.source, args.angle)
        with args.output.open('x', encoding='utf-8') as stream:
            stream.write(text)
    except (OSError, ValueError, et.ParseError) as error:
        parser.exit(1, f'{error}\n')
    print(f'取り付け角度: {args.angle} deg / 出力: {args.output}')


if __name__ == '__main__':
    main()

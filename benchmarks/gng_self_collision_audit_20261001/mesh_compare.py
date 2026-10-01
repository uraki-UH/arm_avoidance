#!/usr/bin/env python3
"""URDF 表示形状と衝突形状のリンク座標系内比較。"""

import argparse
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as element_tree
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def origin_matrix(element):
    """URDF origin の回転・並進行列。"""
    result = np.eye(4)
    if element is None:
        return result
    result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    roll, pitch, yaw = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
    roll_cos, pitch_cos, yaw_cos = map(math.cos, (roll, pitch, yaw))
    roll_sin, pitch_sin, yaw_sin = map(math.sin, (roll, pitch, yaw))
    result[:3, :3] = [
        [yaw_cos * pitch_cos, yaw_cos * pitch_sin * roll_sin - yaw_sin * roll_cos,
         yaw_cos * pitch_sin * roll_cos + yaw_sin * roll_sin],
        [yaw_sin * pitch_cos, yaw_sin * pitch_sin * roll_sin + yaw_cos * roll_cos,
         yaw_sin * pitch_sin * roll_cos - yaw_cos * roll_sin],
        [-pitch_sin, pitch_cos * roll_sin, pitch_cos * roll_cos],
    ]
    return result


def load_stl(path):
    """有限サイズの STL 頂点列読込。"""
    data = path.read_bytes()
    if len(data) >= 84:
        num_triangles = struct.unpack_from('<I', data, 80)[0]
        if len(data) == 84 + num_triangles * 50:
            data_type = np.dtype([('normal', '<f4', (3,)),
                                 ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
            return np.frombuffer(data, dtype=data_type, count=num_triangles,
                                 offset=84)['vertices'].astype(float)
    vertices = [list(map(float, line.split()[1:])) for line in data.decode().splitlines()
                if line.strip().startswith('vertex ')]
    return np.asarray(vertices, dtype=float).reshape(-1, 3, 3)


def load_shapes(link, kind, urdf_path):
    """同一リンクに属する複数形状の統合。"""
    shapes, sources = [], []
    for shape in link.findall(kind):
        mesh = shape.find('geometry/mesh')
        if mesh is None:
            raise ValueError(f"未対応 geometry: {link.get('name')} {kind}")
        name = mesh.get('filename')
        if name.startswith('package://'):
            name = name.removeprefix('package://').split('/', 1)[1]
        path = urdf_path.parent / name
        vertices = load_stl(path)
        vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        transform = origin_matrix(shape.find('origin'))
        vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
        shapes.append(vertices)
        sources.append({'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'num_triangles': len(vertices)})
    return np.concatenate(shapes), sources


def triangle_digest(vertices, quantization_m):
    """頂点順・三角形順・法線向きに依存しない量子化三角形集合の指紋。"""
    quantized = np.rint(vertices / quantization_m).astype('<i8')
    order = np.lexsort((quantized[:, :, 2], quantized[:, :, 1], quantized[:, :, 0]), axis=1)
    rows = np.take_along_axis(quantized, order[:, :, None], axis=1).reshape(-1, 9)
    rows = np.unique(rows, axis=0)
    return hashlib.sha256(rows.tobytes()).hexdigest(), len(rows)


def describe(vertices, sources):
    values = vertices.reshape(-1, 3)
    return {'sources': sources, 'num_triangles': len(vertices),
            'min_xyz_m': values.min(axis=0).tolist(), 'max_xyz_m': values.max(axis=0).tolist(),
            'num_vertices': len(values)}


def compare_model(root, model):
    """モデルごとの表示形状と衝突形状の独立寸法比較。"""
    folder = 'topo_dual_arm_max' if model == 'max' else 'topo_dual_arm_max_long'
    urdf_path = root / 'urdf' / folder / 'topo_dual_arm_max.urdf'
    robot = element_tree.parse(urdf_path).getroot()
    result = {'model': model, 'urdf_path': str(urdf_path),
              'urdf_sha256': hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
              'links': [], 'visual_only_links': []}
    for link in robot.findall('link'):
        if link.find('visual') is None:
            continue
        visual, visual_sources = load_shapes(link, 'visual', urdf_path)
        if link.find('collision') is None:
            result['visual_only_links'].append({'name': link.get('name'),
                                                'visual': describe(visual, visual_sources)})
            continue
        collision, collision_sources = load_shapes(link, 'collision', urdf_path)
        record = {'name': link.get('name'), 'visual': describe(visual, visual_sources),
                  'collision': describe(collision, collision_sources)}
        if link.get('name') in ('base_link', 'torso_link'):
            record['is_same_triangle_set_at_1_micrometer_quantization'] = triangle_digest(visual, 1e-6) == triangle_digest(collision, 1e-6)
            visual_vertices = visual.reshape(-1, 3)
            collision_vertices = collision.reshape(-1, 3)
            record['max_visual_vertex_to_collision_vertex_dist_m'] = float(cKDTree(collision_vertices).query(visual_vertices, workers=1)[0].max())
            record['max_collision_vertex_to_visual_vertex_dist_m'] = float(cKDTree(visual_vertices).query(collision_vertices, workers=1)[0].max())
        result['links'].append(record)
        print('compared', model, link.get('name'), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report = {'comparison_frame': 'each URDF link frame', 'units': 'm',
              'limitations': ['頂点最近傍距離はメッシュ表面距離ではないため包含判定への流用不可。',
                              '量子化一致は三角形座標集合の比較であり衝突判定ではない。',
                              '外装だけのリンクと別リンクの衝突形状との重複は未評価。',
                              '三角形集合と最近傍の比較はbase_linkとtorso_linkのみ。他リンクはローカルbbox比較のみ。'],
              'models': [compare_model(args.root.resolve(), model) for model in ('max', 'long')]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    for model in report['models']:
        print(model['model'])
        for link in model['links']:
            print(link['name'], link['visual']['num_triangles'], link['collision']['num_triangles'],
                  'same_triangles_1um', link.get('is_same_triangle_set_at_1_micrometer_quantization', 'not_checked'))
        print('visual_only', [link['name'] for link in model['visual_only_links']])


if __name__ == '__main__':
    main()

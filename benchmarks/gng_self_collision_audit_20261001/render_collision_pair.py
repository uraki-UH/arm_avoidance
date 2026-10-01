"""保存関節角と独立URDF FKによる、衝突対象2メッシュの全三角形描画。"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as et

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    args = parser.parse_args()
    started = time.monotonic()
    root = args.root.resolve()
    output = root / 'artifacts/gng_self_collision_audit_20261001'
    urdf_path = root / 'urdf/topo_dual_arm_max/topo_dual_arm_max.urdf'
    gng_path = root / 'artifacts/gng_coverage_repair_20261001/max/model/gng.bin'
    pilot_path = output / 'pilot_batch/001_max_body_visual_collision/result/audit_metrics.json'
    metadata_path = output / 'node10062_mesh_geometry.json'
    figure_paths = [output / 'node10062_mesh_view_a.png', output / 'node10062_mesh_view_b.png']
    for path in [metadata_path, *figure_paths]:
        if path.exists():
            raise ValueError(f'既存成果物への上書き拒否: {path}')
    geometry = load_module('pair_geometry', root / 'benchmarks/gng_coverage_repair_20261001/verify_vlut_geometry.py')
    export = load_module('pair_export', root / 'benchmarks/voxel_pose_compression_20260930/export_preview.py')
    verify = load_module('pair_verify', root / 'benchmarks/gng_coverage_repair_20261001/verify_repair.py')
    pilot = json.loads(pilot_path.read_text())
    evidence = next(pair for pair in pilot['pairs'] if pair['body_link'] == 'L_shoulder_cover_link'
                    and pair['arm_link'] == 'R_finger_right')
    sample = evidence['first_added_collision']
    if sample['id'] != 10062:
        raise ValueError('想定外の監査ノードID')
    ids, angles, tcp = verify.decode_states(export.read_gng(gng_path))
    node_idx = int(np.flatnonzero(ids == sample['id'])[0])
    q = angles[node_idx]
    if not np.array_equal(q, np.asarray(sample['q'], dtype=np.float32)):
        raise ValueError('監査関節角と保存GNGの不一致')
    joint_names = [f'{side}_joint{idx}' for side in ('L', 'R') for idx in range(1, 8)]
    transforms, frame = geometry.all_link_fk(urdf_path, joint_names, q[None])
    max_tcp_dev_m = max(float(np.linalg.norm(transforms[side + '_tcp'][0, :3, 3] - tcp[node_idx, layer]))
                        for layer, side in enumerate(('L', 'R')))
    if max_tcp_dev_m >= 1e-5:
        raise ValueError('保存TCPと独立FKの不一致')
    document = et.parse(urdf_path).getroot()
    meshes = []
    input_paths = [urdf_path, gng_path, pilot_path]
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    for link_name, kind, color, alpha in [
        ('L_shoulder_cover_link', 'visual', '#2a83cc', 0.30),
        ('R_finger_right', 'collision', '#e56b22', 1.0),
    ]:
        element = document.find(f'./link[@name="{link_name}"]/{kind}')
        mesh = element.find('geometry/mesh')
        mesh_path = (urdf_path.parent / mesh.get('filename')).resolve(strict=True)
        scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        data = mesh_path.read_bytes()
        num_triangles = struct.unpack_from('<I', data, 80)[0]
        if len(data) != 84 + 50 * num_triangles:
            raise ValueError('未対応または不正なbinary STL')
        local = np.frombuffer(data, dtype=dtype, count=num_triangles, offset=84)['vertices'].astype(np.float64)
        local *= scale
        origin = np.eye(4)
        origin_xml = element.find('origin')
        if origin_xml is not None:
            origin[:3, 3] = np.fromstring(origin_xml.get('xyz', '0 0 0'), sep=' ')
            origin[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin_xml.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        transform = transforms[link_name][0] @ origin
        world = (local @ transform[:3, :3].T + transform[:3, 3]) * 1000.0
        meshes.append({'link': link_name, 'kind': kind, 'path': mesh_path,
                       'triangles': world, 'color': color, 'alpha': alpha,
                       'transform': transform, 'scale': scale})
        input_paths.append(mesh_path)
    points = np.concatenate([mesh['triangles'].reshape(-1, 3) for mesh in meshes])
    min_xyz, max_xyz = points.min(axis=0), points.max(axis=0)
    center = (min_xyz + max_xyz) * 0.5
    half_span = float((max_xyz - min_xyz).max()) * 0.56
    for path, (elevation, azimuth) in zip(figure_paths, [(24, -62), (24, 35)]):
        figure = plt.figure(figsize=(10, 8), facecolor='white')
        axis = figure.add_subplot(111, projection='3d')
        for mesh in meshes:
            # 全三角形の保持。可視化用の透明度のみ変更。
            collection = Poly3DCollection(mesh['triangles'], facecolor=mesh['color'],
                                           edgecolor='none', linewidth=0, alpha=mesh['alpha'],
                                           antialiased=False, zsort='average')
            axis.add_collection3d(collection)
        axis.set_xlim(center[0] - half_span, center[0] + half_span)
        axis.set_ylim(center[1] - half_span, center[1] + half_span)
        axis.set_zlim(center[2] - half_span, center[2] + half_span)
        axis.set_box_aspect((1, 1, 1))
        axis.set_xlabel('X [mm]', labelpad=10)
        axis.set_ylabel('Y [mm]', labelpad=10)
        axis.set_zlabel('Z [mm]', labelpad=10)
        axis.view_init(elev=elevation, azim=azimuth)
        axis.set_proj_type('ortho')
        axis.set_title(f'GNG node 10062 | full mesh geometry\nFrame: {frame} | view azimuth {azimuth} deg', pad=18)
        axis.legend(handles=[Patch(facecolor=mesh['color'], alpha=mesh['alpha'],
                                  label=f"{mesh['link']} ({mesh['kind']})") for mesh in meshes],
                    loc='upper left', bbox_to_anchor=(0.01, 0.97), fontsize=10)
        figure.text(0.5, 0.045, 'Independent URDF FK; original mesh scale and all triangles retained.\n'
                    'FCL intersection recorded in the diagnostic URDF with the cover visual added as collision geometry.',
                     ha='center', va='center', fontsize=9)
        figure.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(figure)
    metadata = {'node_id': 10062, 'q': q.tolist(), 'joint_names': joint_names, 'frame': frame,
                'max_tcp_dev_m': max_tcp_dev_m, 'audit_pair': evidence,
                'meshes': [{'link': mesh['link'], 'kind': mesh['kind'], 'path': str(mesh['path']),
                            'num_triangles': len(mesh['triangles']), 'scale': mesh['scale'].tolist(),
                            'world_transform_m': mesh['transform'].tolist(),
                            'min_world_xyz_mm': mesh['triangles'].reshape(-1, 3).min(axis=0).tolist(),
                            'max_world_xyz_mm': mesh['triangles'].reshape(-1, 3).max(axis=0).tolist()}
                           for mesh in meshes],
                'all_link_transforms_m': {name: transform[0].tolist() for name, transform in transforms.items()},
                'input_sha256': {str(path): sha256(path) for path in input_paths},
                'figures': [str(path) for path in figure_paths], 'elapsed_sec': time.monotonic() - started,
                'limitation': '独立XML FKによる保存姿勢の描画。接触点や接触深さの推定なし。肩外装の半透明表示。'}
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({key: metadata[key] for key in ['node_id', 'max_tcp_dev_m', 'figures', 'elapsed_sec']}, ensure_ascii=False))


if __name__ == '__main__':
    main()

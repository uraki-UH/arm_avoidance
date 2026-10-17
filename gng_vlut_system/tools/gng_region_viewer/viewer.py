#!/usr/bin/env python3
"""保存済み姿勢領域の読取り専用3D表示。ROS・実機通信・再ボクセル化なし。"""
import argparse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
from urllib.parse import urlsplit
import xml.etree.ElementTree as element_tree

import numpy as np

package_dir = Path(__file__).resolve().parents[2]
workspace = package_dir.parent
sys.path.insert(0, str(package_dir/'scripts'))
from dual_arm_avoidance_geometry import robot_geometry
from gng_edge_vlut import geometry_signature, load_index
from gng_region_vlut import region_vlut, unpacked_cells


def pose_examples(model, region_idx, max_num_pose_samples):
    """凸領域内の有限な姿勢例。二次域の関節制限は方向全体の縮小で反映。"""
    arrays = model.arrays
    reference = arrays['region_positions'][region_idx]
    if model.metadata['region_type'] == 'joint_box':
        candidates = model.region_vertices(region_idx)
    elif 'generators' not in arrays:
        vertices = model.region_vertices(region_idx)
        candidates = np.vstack((reference, vertices.mean(axis=0), vertices))
    else:
        generators = arrays['generators'][region_idx]
        limits = arrays['joint_limits']
        candidates = [reference]
        for direction in np.r_[generators.T, -generators.T]:
            scale = 1.
            for idx in np.flatnonzero(direction):
                boundary = limits[idx, 1 if direction[idx] > 0 else 0]
                scale = min(scale, max(0., (boundary-reference[idx])/direction[idx]))
            candidates.append(reference+scale*direction)
        candidates = np.asarray(candidates)
    unique = list(dict.fromkeys(row.astype('<f8').tobytes() for row in candidates))
    candidates = np.asarray([np.frombuffer(value, dtype='<f8') for value in unique])
    if len(candidates) > max_num_pose_samples:
        candidates = candidates[np.linspace(0, len(candidates)-1, max_num_pose_samples).astype(int)]
    return candidates


def viewer_data(model, geometry, joint_types, *, max_num_pose_samples=12):
    """保存セルの一度だけの復号と、姿勢例だけのFK。全域占有は保存済み包絡。"""
    if geometry_signature(geometry) != model.metadata['geometry_signature']:
        raise ValueError('索引とURDF・計画リンク・固定環境の不一致。構築時と同じ設定の指定が必要')
    if not 2 <= max_num_pose_samples <= 32:
        raise ValueError('姿勢例数は2〜32個の指定が必要')
    arrays, metadata = model.arrays, model.metadata
    if len(arrays['region_positions']) > 2048 or len(geometry.joint_names) > 128:
        raise ValueError('Viewerの領域数・関節数の上限超過')
    if len(arrays['entry_regions'])+len(arrays['occupied_ids']) > 2000000:
        raise ValueError('Viewerの保存セル関係数の上限超過')
    palette = np.unique(np.r_[arrays['voxel_ids'], arrays['occupied_ids'], arrays['core_ids']])
    _, occupied_inverse, occupied_frequency = np.unique(arrays['occupied_ids'], return_inverse=True, return_counts=True)
    outer = [[] for _ in arrays['region_positions']]
    inner = [[] for _ in arrays['region_positions']]
    palette_voxels = np.searchsorted(palette, arrays['voxel_ids'])
    for row, palette_idx in enumerate(palette_voxels):
        if metadata.get('has_parent_leaf_hierarchy', False):
            for leaf_idx in model.posting_regions(row):
                outer[leaf_idx].append(int(palette_idx))
            continue
        start, end = arrays['pointers'][row:row+2]
        for entry in range(start, end):
            region_idx = arrays['entry_regions'][entry]
            outer[region_idx].append(int(palette_idx))
            if arrays['is_inner'][entry]:
                inner[region_idx].append(int(palette_idx))
    active = [metadata['joint_names'].index(name) for name in metadata['active_joint_names']]
    widths = np.diff(geometry.limits[active], axis=1).ravel()
    neighbors = [set() for _ in arrays['region_positions']]
    for first, second in arrays['connection_pairs']:
        if first != second:
            neighbors[first].add(int(second))
            neighbors[second].add(int(first))
    regions = []
    for idx, reference in enumerate(arrays['region_positions']):
        vertices = model.region_vertices(idx)
        poses = pose_examples(model, idx, max_num_pose_samples)
        start, end = arrays['occupied_pointers'][idx:idx+2]
        occupied = arrays['occupied_ids'][start:end]
        core = arrays['core_ids'][slice(*arrays['core_pointers'][idx:idx+2])]
        generators = arrays['generators'][idx] if 'generators' in arrays else (vertices-reference).T
        rank = int(np.count_nonzero(np.linalg.svd(generators[active]/widths[:, None], compute_uv=False) > 1e-10))
        if metadata['region_type'] == 'joint_box':
            lower, upper = arrays['box_lower'][idx], arrays['box_upper'][idx]
        elif 'generators' in arrays:
            dev = np.linalg.norm(generators, axis=1)
            lower, upper = np.maximum(reference-dev, geometry.limits[:, 0]), np.minimum(reference+dev, geometry.limits[:, 1])
        else:
            lower, upper = vertices.min(axis=0), vertices.max(axis=0)
        regions.append(dict(region_idx=idx, reference=reference.tolist(), vertices=vertices.tolist(),
            has_index=bool(arrays['has_index'][idx]),
            generators=generators.tolist() if 'generators' in arrays else None,
            min_position=lower.tolist(), max_position=upper.tolist(), affine_rank=rank,
            neighbors=sorted(neighbors[idx]), min_internal=float(arrays['min_internal'][idx]),
            occupied=np.searchsorted(palette, occupied).tolist(), core=np.searchsorted(palette, core).tolist(),
            exclusive=np.searchsorted(palette, occupied[occupied_frequency[occupied_inverse[start:end]] == 1]).tolist(),
            outer=outer[idx] if arrays['has_index'][idx] else None,
            inner=inner[idx] if arrays['has_index'][idx] else None,
            poses=[dict(positions=q.tolist(), centers=np.round(centers, 6).tolist())
                   for q, centers in zip(poses, geometry.centers(poses))]))
    parents = None
    if metadata.get('has_parent_leaf_hierarchy', False):
        parents = [dict(parent_idx=idx, leaf_regions=arrays['parent_leaf_regions'][slice(*
            arrays['parent_leaf_pointers'][idx:idx+2])].tolist()) for idx in range(len(arrays['parent_nodes']))]
    return dict(format='gng_region_viewer_v1', model_signature=metadata['content_signature'],
        region_type=metadata['region_type'], construction_method=metadata.get('construction_method', 'legacy'),
        internal_policy=metadata.get('internal_policy', 'coupled'),
        joint_names=metadata['joint_names'], joint_types=[joint_types[name] for name in metadata['joint_names']],
        active_indices=active, joint_limits=geometry.limits.tolist(), voxel_size=metadata['voxel_size'],
        clearance_margin=metadata['clearance_margin'], grid_cells=unpacked_cells(palette).tolist(),
        sphere_radii=geometry.radii.tolist(), sphere_links=[name for name, *_ in geometry.spheres],
        regions=regions, parents=parents, has_hardware_output=False, is_real_robot_certified=False,
        pose_scope='finite_examples_not_full_region', occupancy_scope='saved_conservative_envelope',
        pose_center_precision_m=1e-6)


class viewer_handler(BaseHTTPRequestHandler):
    """固定ファイルとメモリ上の表示データだけの配信。任意パス・外部操作APIなし。"""
    def __init__(self, *args, payload, routes, **kwargs):
        self.payload, self.routes = payload, routes
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.headers.get('Host') not in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'):
            self.send_error(421)
            return
        route = urlsplit(self.path).path
        if route == '/api/model':
            data, content_type = self.payload, 'application/json; charset=utf-8'
        elif route in self.routes:
            filename, content_type = self.routes[route]
            data = filename.read_bytes()
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser(description='保存済み領域索引の3D Viewer。ROS・実機接続なし')
    parser.add_argument('model_file')
    parser.add_argument('--urdf', default='')
    parser.add_argument('--avoidance-config', default='')
    parser.add_argument('--port', type=int, default=8785)
    parser.add_argument('--max-num-pose-samples', type=int, default=12)
    parser.add_argument('--export-json', default='')
    parser.add_argument('--allow-replace', action='store_true')
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error('ポート番号の範囲外')
    config = {}
    if args.avoidance_config:
        import yaml
        config = yaml.safe_load(Path(args.avoidance_config).read_text())['dual_arm_avoidance_demo']
    metadata, arrays = load_index(args.model_file)
    if metadata.get('format') in ('urdf_vrm_v1', 'urdf_vrm_hierarchy_v1'):
        from urdf_vrm import read_geometry, resolve_resource, urdf_hierarchy, urdf_vrm
        if args.avoidance_config:
            parser.error('URDF箱方式のViewerは保存済みの構築条件を使用。別の回避設定は指定不可')
        urdf = resolve_resource(args.urdf or metadata['urdf_path'], workspace)
        if metadata.get('self_collision_policy') == 'runtime_existing_policy':
            geometry = robot_geometry(str(urdf), metadata['planning_groups'])
        else:
            geometry = read_geometry(urdf, metadata['planning_groups'], metadata.get('self_collision_exclusion_pairs', ()),
                                     metadata.get('floor_z'))
        factory = urdf_hierarchy if metadata['format'] == 'urdf_vrm_hierarchy_v1' else urdf_vrm
        model = factory(metadata, arrays)
    else:
        urdf = Path(args.urdf) if args.urdf else workspace/'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf'
        geometry = robot_geometry(str(urdf), config.get('planning_groups'))
        model = region_vlut(metadata, arrays)
    joint_types = {joint.attrib['name']:joint.attrib['type'] for joint in element_tree.parse(urdf).getroot().findall('joint')}
    data = viewer_data(model, geometry, joint_types, max_num_pose_samples=args.max_num_pose_samples)
    payload = json.dumps(data, allow_nan=False, separators=(',', ':')).encode()
    if len(payload) > 32*1024*1024:
        raise ValueError('Viewerデータ32 MiBの上限超過。姿勢例数の削減が必要')
    if args.export_json:
        output = Path(args.export_json)
        if output.suffix.lower() != '.json' or output.resolve() in (Path(args.model_file).resolve(), urdf.resolve()):
            raise ValueError('元索引・URDFと異なるJSON保存先の指定が必要')
        with output.open('wb' if args.allow_replace else 'xb') as stream:
            stream.write(payload)
        print(json.dumps(dict(output=args.export_json, num_regions=len(data['regions']), num_bytes=len(payload))))
        return
    directory = Path(__file__).resolve().parent
    vendor = workspace/'ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/app/vendor/three'
    routes = {'/':(directory/'index.html', 'text/html; charset=utf-8'),
              '/viewer.js':(directory/'viewer.js', 'text/javascript; charset=utf-8')}
    for name in ('build/three.module.js', 'build/three.core.js', 'examples/jsm/controls/OrbitControls.js'):
        routes['/vendor/three/'+name] = vendor/name, 'text/javascript; charset=utf-8'
    if any(not filename.is_file() for filename, _ in routes.values()):
        raise FileNotFoundError('Viewerまたは既存Three.jsのファイル未配置')
    handler = partial(viewer_handler, payload=payload, routes=routes)
    with ThreadingHTTPServer(('127.0.0.1', args.port), handler) as server:
        print(json.dumps(dict(url=f'http://127.0.0.1:{server.server_port}',
            num_regions=len(data['regions']), num_bytes=len(payload), has_hardware_output=False)), flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()

"""既存の自己干渉確認付き可動域セルの証拠姿勢に対する位置被覆の評価。"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as et

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def load_module(path):
    spec = importlib.util.spec_from_file_location('coverage', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_map(path):
    data = path.read_bytes()
    magic, version, layer, angle_dim, num_nodes, num_edges = struct.unpack_from('<8sIIIII', data)
    assert (magic, version) in ((b'VIZGST1\0', 1), (b'VIZGST2\0', 2))
    fields = [('position', '<f4', (3,)), ('normal', '<f4', (3,)), ('label', 'u1')]
    if version == 2:
        fields += [('counts', '<u4', (3,))]
    fields += [('angles', '<f4', (angle_dim,))]
    dtype = np.dtype(fields)
    assert len(data) == 28 + num_nodes*dtype.itemsize + num_edges*8
    nodes = np.frombuffer(data, dtype=dtype, count=num_nodes, offset=28)
    assert np.isfinite(nodes['position']).all() and np.isfinite(nodes['angles']).all()
    assert np.all(nodes['label'] == 1)
    return nodes


def forward_kinematics(urdf_path, leaf, joint_names, angles):
    urdf = et.parse(urdf_path).getroot()
    by_child = {joint.find('child').get('link'): joint for joint in urdf.findall('joint')}
    chain = []
    current = leaf
    while current in by_child:
        joint = by_child[current]
        chain.append(joint)
        current = joint.find('parent').get('link')
    joint_idx = {name: idx for idx, name in enumerate(joint_names)}
    num = len(angles)
    orientation = np.broadcast_to(np.eye(3), (num, 3, 3)).copy()
    position = np.zeros((num, 3))
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        position += np.einsum('nij,j->ni', orientation, xyz)
        orientation = orientation @ Rotation.from_euler('xyz', rpy).as_matrix()
        name = joint.get('name')
        values = angles[:, joint_idx[name]] if name in joint_idx else np.zeros(num)
        kind = joint.get('type')
        axis_xml = joint.find('axis')
        axis = np.fromstring(axis_xml.get('xyz', '1 0 0'), sep=' ') if axis_xml is not None else np.array([1., 0., 0.])
        axis = axis / np.linalg.norm(axis)
        if kind in ('revolute', 'continuous'):
            orientation = orientation @ Rotation.from_rotvec(values[:, None]*axis).as_matrix()
        elif kind == 'prismatic':
            position += np.einsum('nij,nj->ni', orientation, values[:, None]*axis)
        else:
            assert kind == 'fixed'
    return position, current


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
    parser.add_argument('--model', choices=('max', 'long'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    coverage = load_module(Path(__file__).with_name('coverage.py'))
    prepare = coverage.load_module('prepare', args.root/'benchmarks/voxel_pose_compression_20260930/prepare.py')
    preview = args.root/'artifacts/voxel_viewer_preview_20260930'/args.model
    meta = json.loads((preview/'preview.json').read_text())
    original = Path(meta['input_paths']['gng'])
    ids, q, tcp, _ = prepare.read_gng(original)
    selected_ids, _, representative_tcp, _ = prepare.read_gng(preview/'gng.bin')
    model = original.parent
    arm_metadata = [json.loads((model/'reachability'/f'{arm}_arm.bin.json').read_text()) for arm in ('left', 'right')]
    names = sum([item['joint_names'] for item in arm_metadata], [])
    assert len(names) == q.shape[1] == 14
    result = {'model': args.model, 'scope': '既存片腕可動域マップの証拠姿勢による位置比較。他関節0、環境障害物なし。双腕同時可動域・姿勢角・経路の保証なし。', 'arms': []}
    for layer, arm in enumerate(('left', 'right')):
        map_path = model/'reachability'/f'{arm}_arm.bin'
        reference = arm_metadata[layer]
        urdf_path = args.root/Path(reference['urdf_path']).relative_to('/ros2_ws/src')
        nodes = read_map(map_path)
        assert len(nodes) == reference['num_cells']
        witness, root_link = forward_kinematics(urdf_path, reference['eef_link'], reference['joint_names'], nodes['angles'].astype(float))
        saved_fk, _ = forward_kinematics(urdf_path, reference['eef_link'], names, q.astype(float))
        max_saved_fk_error = float(np.linalg.norm(saved_fk-tcp[:, layer], axis=1).max())
        assert max_saved_fk_error < 1e-5, ('stored FK mismatch', arm, max_saved_fk_error, root_link)
        # 格子中心近傍の実到達点への復元。中心点そのものの到達性とは区別
        voxel_size = reference['voxel_size']
        max_cell_axis_error = float(np.abs(witness-nodes['position']).max())
        assert max_cell_axis_error <= voxel_size/2 + 1e-6, ('cell witness mismatch', max_cell_axis_error)
        original_dist, original_idx = cKDTree(tcp[:, layer]).query(witness, workers=1)
        representative_dist, representative_idx = cKDTree(representative_tcp[:, layer]).query(witness, workers=1)
        worst_idx = int(np.argmax(representative_dist))
        limits = [.01, .02, .03, .04, .05, .08, .1]
        result['arms'].append({'arm': arm, 'num_reference_cells': len(nodes), 'reference_voxel_size_m': voxel_size,
                              'reference_metadata': reference, 'urdf_root_link': root_link,
                              'input_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (original, preview/'gng.bin', map_path, Path(str(map_path)+'.json'), urdf_path)},
                              'max_original_fk_error_m': max_saved_fk_error, 'max_cell_axis_error_m': max_cell_axis_error,
                              'original_dist_m': coverage.stats(original_dist, limits),
                              'representative_dist_m': coverage.stats(representative_dist, limits),
                              'worst_witness': {'position': witness[worst_idx].tolist(), 'angles': nodes['angles'][worst_idx].tolist(),
                                                'nearest_representative_id': int(selected_ids[representative_idx[worst_idx]]),
                                                'nearest_original_id': int(ids[original_idx[worst_idx]]),
                                                'original_dist_m': float(original_dist[worst_idx]),
                                                'representative_dist_m': float(representative_dist[worst_idx])}})
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()

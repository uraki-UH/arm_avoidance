"""実bagの読み取り専用抽出、現行判定の機械的抽出、反復比較の条件生成。"""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import struct

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=60)
    args = parser.parse_args()
    root = Path('/ros2_ws/src')
    args.output.mkdir(parents=True, exist_ok=False)
    source = root / 'ais_gng_cpu/src/gng_cpu/src/cpu/cugng.cpp'
    text = source.read_text()
    start = text.index('bool CUGNG::is_explained_by_plane(')
    end = text.index('\n// 既存入力セル', start)
    reference = text[start:end].replace('bool CUGNG::is_explained_by_plane(',
        'bool node_lookup(const CUGNG &core, ')
    reference = reference.replace(') const {', ') {', 1)
    reference = reference.replace('    const auto matches_plane',
        '    const auto &nodes = core.nodes;\n'
        '    const auto &insertion_owners = core.insertion_owners;\n'
        '    const auto &insertion_planes = core.insertion_planes;\n'
        '    const auto &insertion_config = core.insertion_config;\n'
        '    const auto matches_plane', 1)
    (args.output / 'reference.hpp').write_text(reference)
    config_dir = root / 'ais_gng_cpu/src/ais_gng/config'
    config = yaml.safe_load((config_dir / 'gng_cpu/at128.yaml').read_text())['ais_gng_node']['ros__parameters']
    config.update({'input.point_cloud_num': 200000, 'node.num_max': 20000,
        'node.learning_num': 4000, 'input.local_coordinates': True, 'input.voxel_grid_unit': .5})
    with (args.output / 'parameters.txt').open('x') as file:
        for name, values in config.items():
            for idx, value in enumerate(values if isinstance(values, list) else [values]):
                if isinstance(value, (bool, int, float)):
                    file.write(f'{name} {idx} {float(value)}\n')
    plane_config = yaml.safe_load((config_dir / 'plane_cluster_incremental.yaml').read_text())['plane_cluster_incremental_node']['ros__parameters']
    with (args.output / 'plane_parameters.txt').open('x') as file:
        for name, value in plane_config.items():
            if isinstance(value, (bool, int, float)):
                file.write(f'{name} {float(value)}\n')
    bag = '/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3'
    with sqlite3.connect('file:'+bag+'?mode=ro', uri=True) as db, (args.output / 'points.bin').open('xb') as file:
        topic = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
        num_frames = 0
        for raw, in db.execute('select data from messages where topic_id=? order by timestamp limit ?', (topic, args.frames)):
            cloud = deserialize_message(raw, PointCloud2)
            assert not cloud.is_bigendian
            assert [(v.name, v.offset, v.datatype) for v in cloud.fields[:3]] == [('x', 0, 7), ('y', 4, 7), ('z', 8, 7)]
            points = np.ndarray((cloud.height, cloud.width, 3), dtype='<f4', buffer=cloud.data,
                strides=(cloud.row_step, cloud.point_step, 4)).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1)]
            if len(points) > 200000:
                points = points[np.linspace(0, len(points)-1, 200000, dtype=np.int64)]
            file.write(struct.pack('<I', len(points)))
            file.write(points.astype('<f4').tobytes())
            num_frames += 1
        assert num_frames == args.frames
    metadata = {'bag': bag, 'frames': args.frames, 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'points_sha256': hashlib.sha256((args.output / 'points.bin').read_bytes()).hexdigest(),
        'core_config': config, 'plane_config': plane_config}
    (args.output / 'conditions.json').write_text(json.dumps(metadata, indent=2))
    cases = [{'name': 'paired', 'cwd': str(root),
        'argv': ['taskset', '-c', '4', str(args.output/'build/compare'), str(args.output), '@seed@', '@case_dir@'],
        'metrics': 'metrics.json'}]
    (args.output / 'cases.json').write_text(json.dumps({'cases': cases}, indent=2))
    print(json.dumps({'prepared_frames': num_frames, 'output': str(args.output)}))


if __name__ == '__main__':
    main()

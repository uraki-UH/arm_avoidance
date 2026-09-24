"""交差点用YAMLからのworld → map → LiDAR静的TF配信。"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def read_vector(config, key):
    values = config.get(key)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{key}: 3要素の数値配列が必要")
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in values):
        raise ValueError(f"{key}: 有限の数値が必要")
    return [float(value) for value in values]


def launch_setup(context):
    config_file = Path(LaunchConfiguration('config_file').perform(context))
    with config_file.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"{config_file}: YAMLのマッピング形式が必要")

    frames = [config.get(key) for key in ('world_frame', 'map_frame', 'lidar_frame')]
    if any(not isinstance(frame, str) or not frame or frame.startswith('/')
           or any(char.isspace() for char in frame) for frame in frames):
        raise ValueError('フレーム名: 空文字・先頭の/・空白は指定不可')
    if len(set(frames)) != 3:
        raise ValueError('world_frame・map_frame・lidar_frame: 異なる名前が必要')
    world_frame, map_frame, lidar_frame = frames
    pos = read_vector(config, 'pos')
    x, y, z = [math.radians(value) / 2 for value in read_vector(config, 'rot_deg')]
    cx, cy, cz = math.cos(x), math.cos(y), math.cos(z)
    sx, sy, sz = math.sin(x), math.sin(y), math.sin(z)
    # Three.jsのEuler XYZと同じRx * Ry * Rz。ROSのRPY指定とは異なる回転順。
    rotation = [sx*cy*cz + cx*sy*sz, cx*sy*cz - sx*cy*sz,
                cx*cy*sz + sx*sy*cz, cx*cy*cz - sx*sy*sz]
    return [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='intersection_world_tf', output='screen',
             arguments=['--frame-id', world_frame, '--child-frame-id', map_frame]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='intersection_lidar_tf', output='screen',
             arguments=['--frame-id', map_frame, '--child-frame-id', lidar_frame,
                        '--x', str(pos[0]), '--y', str(pos[1]), '--z', str(pos[2]),
                        '--qx', str(rotation[0]), '--qy', str(rotation[1]),
                        '--qz', str(rotation[2]), '--qw', str(rotation[3])]),
    ]


def generate_launch_description():
    config_file = str(Path(get_package_share_directory('ais_gng')) / 'config/intersection_tf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_file,
                              description='交差点の位置・姿勢を指定するYAML'),
        OpaqueFunction(function=launch_setup),
    ])

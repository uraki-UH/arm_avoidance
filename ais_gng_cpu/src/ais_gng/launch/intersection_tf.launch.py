"""交差点用YAMLからのworld → map → LiDAR静的TF配信。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ais_gng_transform_config import read_pose
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


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
    pos, rotation = read_pose(config)
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

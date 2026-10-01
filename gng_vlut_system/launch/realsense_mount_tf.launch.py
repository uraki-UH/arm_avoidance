"""ロボットのカメラ取付リンクとRealSense内部TFの接続。"""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    config_path = LaunchConfiguration('mount_config').perform(context)
    config = yaml.safe_load(Path(config_path).read_text())
    if not isinstance(config, dict):
        raise ValueError('取付設定にはparent_frame・child_frame・camera_mount_poseが必要です')
    frames = []
    for key in ('parent_frame', 'child_frame'):
        frame = LaunchConfiguration(key).perform(context).strip() or config.get(key, '')
        if not isinstance(frame, str) or not frame or frame.startswith('/') or any(char.isspace() for char in frame):
            raise ValueError(key + 'には先頭スラッシュ・空白なしのTF名が必要です')
        frames.append(frame)
    if frames[0] == frames[1]:
        raise ValueError('取付TFの親と子には異なるframeが必要です')
    pose_override = LaunchConfiguration('camera_mount_pose').perform(context).strip()
    pose = yaml.safe_load(pose_override) if pose_override else config.get('camera_mount_pose')
    if (not isinstance(pose, list) or len(pose) != 6
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in pose)):
        raise ValueError('camera_mount_poseには有限値6個のxyz [m]・rpy [rad]が必要です')
    arguments = [item for key, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), pose)
                 for item in ('--' + key, str(float(value)))]
    arguments += ['--frame-id', frames[0], '--child-frame-id', frames[1]]
    return [Node(package='tf2_ros', executable='static_transform_publisher',
                 name='realsense_mount_tf', arguments=arguments, output='screen')]


def generate_launch_description():
    config = Path(get_package_share_directory('gng_vlut_system')) / 'config/realsense_mount.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('mount_config', default_value=str(config), description='カメラ取付設定YAML'),
        DeclareLaunchArgument('parent_frame', default_value='', description='取付リンク名の上書き'),
        DeclareLaunchArgument('child_frame', default_value='', description='RealSense本体frame名の上書き'),
        DeclareLaunchArgument('camera_mount_pose', default_value='', description='取付補正xyz [m]・rpy [rad]の上書き'),
        OpaqueFunction(function=launch_setup),
    ])

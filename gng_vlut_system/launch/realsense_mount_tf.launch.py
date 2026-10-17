"""ロボットのカメラ取付リンクとRealSense内部TFの接続。"""
from pathlib import Path
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


sys.path.insert(0, str(Path(__file__).resolve().parent))
from launch_config import realsense_mount, sensor_calibration_action


def launch_setup(context):
    config_path = LaunchConfiguration('mount_config').perform(context)
    config = yaml.safe_load(Path(config_path).read_text())
    mount = realsense_mount(config,
        LaunchConfiguration('parent_frame').perform(context).strip(),
        LaunchConfiguration('child_frame').perform(context).strip(),
        LaunchConfiguration('camera_mount_pose').perform(context).strip())
    return [sensor_calibration_action(get_package_share_directory('gng_vlut_system'), [mount],
        LaunchConfiguration('store_file').perform(context).strip())]


def generate_launch_description():
    config = Path(get_package_share_directory('gng_vlut_system')) / 'config/realsense_mount.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('store_file', default_value='', description='永続校正ストア。空文字は共通設定'),
        DeclareLaunchArgument('mount_config', default_value=str(config), description='カメラ取付設定YAML'),
        DeclareLaunchArgument('parent_frame', default_value='', description='取付リンク名の上書き'),
        DeclareLaunchArgument('child_frame', default_value='', description='RealSense本体frame名の上書き'),
        DeclareLaunchArgument('camera_mount_pose', default_value='', description='取付補正xyz [m]・rpy [rad]の上書き'),
        OpaqueFunction(function=launch_setup),
    ])

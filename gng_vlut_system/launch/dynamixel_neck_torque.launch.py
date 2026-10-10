"""首トルク制御の既存入口。共通手動操作launchへの設定転送。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    defaults = {'config_file': str(share / 'config/dynamixel_neck_torque.yaml'),
                'control_mode': 'gravity', 'allow_hardware_output': '',
                'mode_config_file': str(share / 'config/dynamixel_adaptive_hold.yaml'),
                'interaction_source': '', 'driver_namespace': ''}
    arguments = {name: LaunchConfiguration(name) for name in defaults}
    arguments.update(target='neck', node_name='dynamixel_neck_torque')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/dynamixel_hand_guiding.launch.py')),
                                 launch_arguments=arguments.items()),
    ])

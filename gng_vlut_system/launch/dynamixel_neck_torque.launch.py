"""ID51・52専用の重力補償・減衰。終了時のゼロ電流・トルクOFF要求。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('gng_vlut_system')) / 'config/dynamixel_neck_torque.yaml'
    node = Node(package='gng_vlut_system', executable='dynamixel_neck_torque.py',
                output='both', parameters=[LaunchConfiguration('config_file'), {'use_sim_time': False}],
                sigterm_timeout='5', sigkill_timeout='5')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(config)),
        node,
        RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='首トルク制御の終了'))])),
    ])

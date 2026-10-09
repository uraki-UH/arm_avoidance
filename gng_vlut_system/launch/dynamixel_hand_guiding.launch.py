"""両腕の重力補償付き手動操作。USB reader・モータモード変更なし。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    params = yaml.safe_load(Path(value('params_file')).read_text())['/**']['ros__parameters']
    config = yaml.safe_load(Path(value('config_file')).read_text())['/**']['ros__parameters']
    bridge = Path(get_package_share_directory('dynamixel_joint_state_bridge')) / 'config'
    node = Node(package='gng_vlut_system', executable='dynamixel_hand_guiding.py', output='screen',
                parameters=[config, {'use_sim_time': False, 'urdf_path': params['urdf_path'],
                'mapping_file': config.get('mapping_file') or str(bridge / 'dynamixel_joint_state_bridge_ids_31_52.yaml')}])
    return [node, RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='両腕手動操作ノードの終了'))]))]


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/topo_dual_arm_max_long.yaml')),
        DeclareLaunchArgument('config_file', default_value=str(share / 'config/dynamixel_hand_guiding.yaml')),
        OpaqueFunction(function=launch_setup),
    ])

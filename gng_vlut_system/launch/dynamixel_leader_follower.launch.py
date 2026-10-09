"""実機リーダー・フォロワーの監視と手動許可による低速追従。"""
import os
from pathlib import Path
import sys

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
    bridge = Path(get_package_share_directory('dynamixel_joint_state_bridge'))/'config'
    config = yaml.safe_load(Path(value('config_file')).read_text())['/**']['ros__parameters']
    is_max_model = params['robot_name'] in ('topo_dual_arm_max', 'topo_dual_arm_max_long')
    follower_mapping_name = ('dynamixel_joint_state_bridge_max_ids_31_52.yaml' if is_max_model
                             else 'dynamixel_joint_state_bridge_ids_31_52.yaml')
    namespace = 'leader_follower'
    overrides = {'use_sim_time': False, 'urdf_path': params['urdf_path'], 'target_source': 'leader',
                 'mapping_file': value('follower_mapping_file') or config.get('mapping_file') or str(bridge/follower_mapping_name),
                 'leader_mapping_file': value('leader_mapping_file') or config.get('leader_mapping_file') or str(bridge/'dynamixel_joint_state_bridge.yaml')}
    if value('allow_hardware_output'):
        if value('allow_hardware_output').lower() not in ('true', 'false'):
            raise ValueError('allow_hardware_outputにはtrueまたはfalseが必要です')
        overrides['allow_hardware_output'] = value('allow_hardware_output').lower() == 'true'
    if value('max_current_ma'):
        overrides['max_current_ma'] = float(value('max_current_ma'))
    output = Node(package='gng_vlut_system', executable='dynamixel_sim_output.py', namespace=namespace,
                  name='output', output='screen', parameters=[config, overrides])
    actions = [output, RegisterEventHandler(OnProcessExit(target_action=output,
               on_exit=[EmitEvent(event=Shutdown(reason='リーダー・フォロワー出力監視の終了'))]))]
    if value('enable_keyboard').lower() == 'true':
        if not sys.stdin.isatty():
            raise ValueError('キー操作には対話TTYが必要です。Dockerではexec -itを使用してください')
        keyboard = Node(package='gng_vlut_system', executable='dynamixel_sim_keyboard.py', output='log',
                        arguments=['--namespace', namespace, '--target-source', 'leader',
                                   '--tty-path', os.ttyname(sys.stdin.fileno())])
        actions.extend([keyboard, RegisterEventHandler(OnProcessExit(target_action=keyboard,
                        on_exit=[EmitEvent(event=Shutdown(reason='リーダー・フォロワー操作端末の終了'))]))])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    defaults = {'params_file': str(share/'config/topo_dual_arm_max_long.yaml'),
                'config_file': str(share/'config/dynamixel_leader_follower.yaml'),
                'leader_mapping_file': '', 'follower_mapping_file': '',
                'allow_hardware_output': '', 'max_current_ma': '', 'enable_keyboard': 'true'}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
                              OpaqueFunction(function=launch_setup)])

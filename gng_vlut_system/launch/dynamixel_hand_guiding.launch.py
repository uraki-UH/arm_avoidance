"""リーダー・フォロワー・首の手動操作。対象と制御オプションの共通起動。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def resolve_path(value, package):
    if value.startswith('package://'):
        name, relative = value[len('package://'):].split('/', 1)
        path = Path(get_package_share_directory(name)) / relative
    else:
        path = Path(value)
        if not path.is_file() and path.parent == Path('.'):
            path = Path(get_package_share_directory(package)) / 'config' / path
    if not path.is_file():
        raise FileNotFoundError('手動操作設定が見つかりません: ' + str(path))
    return path


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    target, mode = value('target'), value('control_mode')
    if target not in ('leader', 'follower', 'neck') or mode not in ('gravity', 'adaptive_hold'):
        raise ValueError('対象はleader／follower／neck、制御はgravity／adaptive_holdが必要です')
    share = Path(get_package_share_directory('gng_vlut_system'))
    config_name = 'dynamixel_neck_torque.yaml' if target == 'neck' else 'dynamixel_hand_guiding.yaml'
    config = yaml.safe_load(resolve_path(value('config_file') or str(share / 'config' / config_name),
                                        'gng_vlut_system').read_text())['/**']['ros__parameters']
    overrides = {'use_sim_time': False, 'control_mode': mode}
    if mode == 'adaptive_hold':
        config.update(yaml.safe_load(resolve_path(value('mode_config_file'), 'gng_vlut_system').read_text())['/**']['ros__parameters'])
        if value('interaction_source'):
            overrides['interaction_source'] = value('interaction_source')
    if value('allow_hardware_output'):
        if value('allow_hardware_output').lower() not in ('true', 'false'):
            raise ValueError('allow_hardware_outputにはtrueまたはfalseが必要です')
        overrides['allow_hardware_output'] = value('allow_hardware_output').lower() == 'true'
    if value('driver_namespace'):
        overrides['driver_namespace'] = value('driver_namespace')
    if target != 'neck':
        params = yaml.safe_load(resolve_path(value('params_file'), 'gng_vlut_system').read_text())['/**']['ros__parameters']
        bridge = Path(get_package_share_directory('dynamixel_joint_state_bridge')) / 'config'
        follower_name = ('dynamixel_joint_state_bridge_max_ids_31_52.yaml'
                         if params['robot_name'] in ('topo_dual_arm_max', 'topo_dual_arm_max_long')
                         else 'dynamixel_joint_state_bridge_ids_31_52.yaml')
        mapping = str(bridge / 'dynamixel_joint_state_bridge.yaml') if target == 'leader' else (
                  params.get('dynamixel_mapping_file') or str(bridge / follower_name))
        overrides.update(arm_role=target,
                         urdf_path=str(resolve_path(value('urdf_path') or params['urdf_path'], 'gng_vlut_system')),
                         mapping_file=str(resolve_path(value('mapping_file') or config.get('mapping_file') or mapping,
                                                       'dynamixel_joint_state_bridge')))
    executable = 'dynamixel_neck_torque.py' if target == 'neck' else 'dynamixel_hand_guiding.py'
    node = Node(package='gng_vlut_system', executable=executable, name=value('node_name'), output='both',
                parameters=[config, overrides], sigterm_timeout='5', sigkill_timeout='5')
    return [node, RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='手動操作制御ノードの終了'))]))]


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    defaults = {'params_file': str(share / 'config/topo_dual_arm_max_long.yaml'),
                'target': 'follower', 'control_mode': 'gravity', 'config_file': '',
                'mode_config_file': str(share / 'config/dynamixel_adaptive_hold.yaml'),
                'interaction_source': '', 'allow_hardware_output': 'false',
                'driver_namespace': '', 'urdf_path': '', 'mapping_file': '',
                'node_name': 'dynamixel_hand_guiding'}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
                              OpaqueFunction(function=launch_setup)])

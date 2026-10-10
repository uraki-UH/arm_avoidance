"""s・r・fの共通起動。接続構成はYAMLとprofileで選択。"""
import os
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def resolve_path(value, package='gng_vlut_system'):
    if value.startswith('package://'):
        package, value = value[10:].split('/', 1)
        return str(Path(get_package_share_directory(package))/value)
    path = Path(value)
    if not path.is_file() and path.parent == Path('.'):
        path = Path(get_package_share_directory(package))/'config'/path
    if not path.is_file():
        raise FileNotFoundError('追従設定が見つかりません: '+str(path))
    return str(path)


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def flag(name):
        setting = value(name).lower()
        if setting not in ('true', 'false'):
            raise ValueError(name+'にはtrueまたはfalseが必要です')
        return setting == 'true'
    share = Path(get_package_share_directory('gng_vlut_system'))
    params_file = resolve_path(value('params_file'))
    params = yaml.safe_load(Path(params_file).read_text())['/**']['ros__parameters']
    config_file = resolve_path(value('config_file'))
    config = yaml.safe_load(Path(config_file).read_text())
    if value('profile') not in config['profiles']:
        raise ValueError('未登録の追従構成: '+value('profile'))
    hardware = yaml.safe_load(Path(resolve_path(value('hardware_config'))).read_text())['/**']['ros__parameters']
    mapping_file = resolve_path(value('follower_mapping_file') or params['dynamixel_mapping_file'])
    guiding_target = value('hand_guiding_target')
    if guiding_target not in ('none', 'leader', 'neck'):
        raise ValueError('hand_guiding_targetにはnone／leader／neckが必要です')
    guiding_arguments = None
    if guiding_target != 'none':
        if value('hand_guiding_control_mode') not in ('gravity', 'adaptive_hold'):
            raise ValueError('hand_guiding_control_modeにはgravityまたはadaptive_holdが必要です')
        guiding_name = 'dynamixel_neck_torque.yaml' if guiding_target == 'neck' else 'dynamixel_hand_guiding.yaml'
        guiding_file = resolve_path(value('hand_guiding_config') or guiding_name)
        guiding_config = yaml.safe_load(Path(guiding_file).read_text())['/**']['ros__parameters']
        if (flag('allow_hardware_output') and flag('allow_hand_guiding_hardware_output') and
                guiding_config.get('driver_namespace', '/dynamixel').rstrip('/') ==
                hardware.get('driver_namespace', '/dynamixel').rstrip('/')):
            raise ValueError('手動操作とフォロワーの同時実機出力には別のhandler名前空間が必要です')
        guiding_arguments = {'params_file': params_file, 'target': guiding_target,
            'control_mode': value('hand_guiding_control_mode'), 'config_file': guiding_file,
            'allow_hardware_output': value('allow_hand_guiding_hardware_output'),
            'node_name': 'dynamixel_hand_guiding'}
        if value('hand_guiding_mode_config'):
            guiding_arguments['mode_config_file'] = resolve_path(value('hand_guiding_mode_config'))
        if value('hand_guiding_interaction_source'):
            guiding_arguments['interaction_source'] = value('hand_guiding_interaction_source')
    actions = []
    if guiding_arguments is not None:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/dynamixel_hand_guiding.launch.py')),
            launch_arguments=guiding_arguments.items()))
    if flag('enable_simulator'):
        root = Path(value('simulator_root')) if value('simulator_root') else Path(params['urdf_path']).parents[2]/'ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator'
        script = root/'integrations/ros2/start_simulator.py'
        if not script.is_file():
            raise FileNotFoundError('simulator_rootの指定が必要です: '+str(root))
        actions.append(ExecuteProcess(cmd=[sys.executable, str(script), '--port', value('simulator_port'), '--bridge-port', value('bridge_port')], cwd=str(root), output='screen'))
    if flag('enable_joint_state_input'):
        input_file = resolve_path(value('input_config'))
        input_params = yaml.safe_load(Path(input_file).read_text())
        if input_params['follower']['driver_namespace'].rstrip('/') != hardware.get('driver_namespace', '/dynamixel').rstrip('/'):
            raise ValueError('input_configとhardware_configのフォロワーバスが一致しません')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/dynamixel_joint_state_input.launch.py')),
            launch_arguments={'params_file': params_file, 'config_file': resolve_path(value('input_config')),
                'follower_mapping_file': mapping_file, 'follower_input_type': value('follower_input_type'),
                'leader_output_topic': config['roles']['r'], 'follower_output_topic': config['roles']['f']}.items()))
    manager = Node(package='gng_vlut_system', executable='robot_follow_manager.py', name='manager', namespace='robot_follow', output='screen',
        parameters=[{'use_sim_time': False, 'config_file': config_file, 'urdf_path': params['urdf_path'], 'joint_names': hardware['joint_names'],
                     'profile': value('profile'), 'allow_hardware_output': flag('allow_hardware_output')}])
    output = Node(package='gng_vlut_system', executable='dynamixel_sim_output.py', name='output', namespace='robot_follow/follower', output='screen',
        parameters=[hardware, {'use_sim_time': False, 'urdf_path': params['urdf_path'], 'mapping_file': mapping_file,
            'target_source': 'joint_state', 'target_topic': '/robot_follow/follower_target', 'target_session_topic': '/robot_follow/config',
            'allow_joint_state_follow': True, 'allow_hardware_output': flag('allow_hardware_output'),
            **({'max_current_ma': float(value('max_current_ma'))} if value('max_current_ma') else {})}])
    for process in (manager, output):
        actions.extend([process, RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='追従経路管理・実機監視の終了'))]))])
    if flag('enable_keyboard'):
        if not sys.stdin.isatty():
            raise ValueError('キー操作には対話TTYが必要です。Dockerではexec -itを使用してください')
        actions.append(Node(package='gng_vlut_system', executable='dynamixel_sim_keyboard.py', output='log',
            arguments=['--namespace', 'robot_follow/follower', '--target-source', 'joint_state', '--tty-path', os.ttyname(sys.stdin.fileno())]))
    if flag('enable_viewer'):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/gng_viewer_bridge.launch.py')),
            launch_arguments={'params_file': params_file, 'enable_dynamixel_joint_state_input': 'false',
                'joint_control_backend': 'external', 'state_topic': '/sim/joint_states', 'enable_robot_state_publisher': 'false'}.items()))
    return actions


def generate_launch_description():
    defaults = {'params_file': 'topo_dual_arm_max_long.yaml', 'config_file': 'robot_follow.yaml', 'profile': 'manual',
        'hardware_config': 'dynamixel_leader_follower.yaml', 'input_config': 'dynamixel_joint_state_input.yaml',
        'follower_mapping_file': '', 'follower_input_type': 'fresh', 'enable_joint_state_input': 'true',
        'allow_hardware_output': 'false', 'max_current_ma': '', 'enable_keyboard': 'false', 'enable_viewer': 'false',
        'enable_simulator': 'true', 'simulator_root': '', 'simulator_port': '8877', 'bridge_port': '8879',
        'hand_guiding_target': 'none', 'hand_guiding_control_mode': 'gravity',
        'hand_guiding_config': '', 'hand_guiding_mode_config': '', 'hand_guiding_interaction_source': '',
        'allow_hand_guiding_hardware_output': 'false'}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()], OpaqueFunction(function=launch_setup)])

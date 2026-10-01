"""既存Gazeboと接続可能なDynamixel小動作・停止試験の単一端末起動。"""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('gng_vlut_system'))
    params = yaml.safe_load(Path(value('params_file')).read_text())['/**']['ros__parameters']
    namespace = 'hw_'+params['robot_name']
    sim = 'sim_'+params['robot_name']
    names = yaml.safe_load(value('joint_names'))
    if not isinstance(names, list) or not names or not all(isinstance(name, str) for name in names):
        raise ValueError('joint_namesには空でない関節名リストが必要です')
    actions = []
    if value('enable_gazebo').lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/pointcloud_avoidance.launch.py')),
            launch_arguments={'robot_config': value('robot_config'), 'input_config': value('input_config'),
                'gui': value('gui'), 'enable_keyboard': 'false',
                'gazebo_master_uri': value('gazebo_master_uri')}.items()))
    output = Node(package='gng_vlut_system', executable='dynamixel_sim_output.py', namespace=namespace,
        output='log', parameters=[value('hardware_config'), {'use_sim_time': False,
            'urdf_path': params['urdf_path'], 'mapping_file': value('mapping_file'), 'joint_names': names,
            **({'max_current_ma': float(value('max_current_ma'))} if value('max_current_ma') else {}),
            'sim_namespace': '/'+sim, 'allow_hardware_output': value('allow_hardware_output').lower() == 'true',
            'allow_sim_follow': value('allow_sim_follow').lower() == 'true'}])
    actions.append(output)
    actions.append(RegisterEventHandler(OnProcessExit(target_action=output,
        on_exit=[EmitEvent(event=Shutdown(reason='実機出力監視の終了'))])))
    if value('enable_keyboard').lower() == 'true':
        if not sys.stdin.isatty():
            raise ValueError('キー操作には対話TTYが必要です。Dockerではexec -itを使用してください')
        keyboard = Node(package='gng_vlut_system', executable='dynamixel_sim_keyboard.py', output='log',
            arguments=['--namespace', namespace, '--sim-namespace', sim, '--tty-path', os.ttyname(sys.stdin.fileno())])
        actions.extend([keyboard, RegisterEventHandler(OnProcessExit(target_action=keyboard,
            on_exit=[EmitEvent(event=Shutdown(reason='実機操作端末の終了'))]))])
    return actions


def generate_launch_description():
    # このlaunchの端末出力は操作端末のGazebo状態2行へ集約
    launch_config.get_screen_handler().addFilter(lambda _record: False)
    os.environ['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = 'log'
    share = Path(get_package_share_directory('gng_vlut_system'))
    bridge = Path(get_package_share_directory('dynamixel_joint_state_bridge'))
    defaults = {'params_file': str(share/'config/ToPoDualArm.yaml'),
        'hardware_config': str(share/'config/topodualarm_hardware_test.yaml'),
        'mapping_file': str(bridge/'config/dynamixel_joint_state_bridge_ids_31_52.yaml'),
        'joint_names': '[L_joint1, L_joint2, L_joint3, L_joint4, L_joint5, L_joint6, L_joint7]',
        'robot_config': str(share/'config/pointcloud_avoidance_topodualarm_left_forward.yaml'), 'input_config': '',
        'allow_hardware_output': 'false', 'allow_sim_follow': 'false',
        'max_current_ma': '',
        'enable_gazebo': 'false', 'enable_keyboard': 'true', 'gui': 'true',
        'gazebo_master_uri': 'http://127.0.0.1:11355'}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
                              OpaqueFunction(function=launch_setup)])

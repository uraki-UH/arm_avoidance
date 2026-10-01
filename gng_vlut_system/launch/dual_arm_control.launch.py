"""単一端末でのGazebo操作と、設定指定時のみのUDP出力窓口。"""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    robot = value('robot')
    if robot not in ('max', 'max_long'):
        raise ValueError('robotはmaxまたはmax_longが必要です')
    share = Path(get_package_share_directory('gng_vlut_system'))
    params_file = value('params_file') or str(share / ('config/topo_dual_arm_' + robot + '.yaml'))
    params = yaml.safe_load(Path(params_file).read_text())['/**']['ros__parameters']
    namespace = 'sim_topo_dual_arm_' + robot
    if params['robot_name'] != 'topo_dual_arm_' + robot:
        raise ValueError('robotとparams_fileの機種名が不一致です')
    demo_config = value('demo_config')
    demo = yaml.safe_load(Path(demo_config).read_text())['dual_arm_gazebo_demo']
    if demo.get('namespace') not in ('', None, namespace):
        raise ValueError('demo_configの名前空間が統合操作と不一致です')
    leader_topic = value('leader_topic')
    if not leader_topic.startswith('/') or leader_topic in ('/joint_states', '/' + namespace + '/joint_states'):
        raise ValueError('フォロワー実測とは別の絶対leader_topicが必要です')
    enable_keyboard = value('enable_keyboard').lower() == 'true'
    if value('udp_config') and not Path(value('udp_config')).is_file():
        raise ValueError('udp_configが見つかりません')
    if enable_keyboard and not sys.stdin.isatty():
        raise ValueError('同じ端末でのキー操作には対話TTYが必要です。Dockerではexec -itを使用してください')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/dual_arm_gazebo_demo.launch.py')),
        launch_arguments={
            'params_file': params_file, 'demo_config': demo_config,
            'avoidance_config': str(share / 'config/dual_arm_gng_lidar_demo.yaml'),
            'gui': value('gui'), 'enable_auto_start': 'false',
            'enable_external_control': 'false', 'enable_dynamixel_leader': 'false',
            'enable_integrated_control': 'true', 'leader_joint_state_topic': leader_topic,
            'gazebo_master_uri': value('gazebo_master_uri'),
            'udp_config': value('udp_config'), 'allow_remote_udp': value('allow_remote_udp'),
        }.items())]
    if value('leader_mapping_file'):
        if not Path(value('leader_mapping_file')).is_file():
            raise ValueError('leader_mapping_fileが見つかりません')
        actions.append(Node(package='dynamixel_joint_state_bridge',
            executable='dynamixel_joint_state_bridge_node', namespace='leader',
            name='dual_arm_leader_input', output='log', parameters=[value('leader_mapping_file'), {
                'input_topic': value('leader_input_topic'), 'output_topic': leader_topic,
                'command_output_topic': '', 'control_claim_topic': '',
            }]))
    if enable_keyboard:
        keyboard = Node(package='gng_vlut_system', executable='dual_arm_control_keyboard.py', output='log',
                        arguments=['--namespace', namespace, '--tty-path', os.ttyname(sys.stdin.fileno())])
        actions.extend([keyboard, RegisterEventHandler(OnProcessExit(
            target_action=keyboard, on_exit=[EmitEvent(event=Shutdown(reason='操作端末の終了'))]))])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    defaults = {'robot': 'max', 'params_file': '', 'gui': 'true', 'enable_keyboard': 'true',
                'demo_config': str(share / 'config/dual_arm_gazebo_demo.yaml'),
                'leader_topic': '/leader/joint_states', 'leader_mapping_file': '',
                'leader_input_topic': '/leader/dynamixel/state/present',
                'udp_config': '', 'allow_remote_udp': 'false',
                'gazebo_master_uri': 'http://127.0.0.1:11355'}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])

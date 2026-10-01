"""Dynamixel実測角度の現在姿勢表示。実機指令出力なし。"""
from pathlib import Path
import math

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

    params_file = value('params_file')
    params = yaml.safe_load(Path(params_file).read_text())['/**']['ros__parameters']
    robot_name = params['robot_name']
    mapping_file = value('mapping_file')
    mapping = yaml.safe_load(Path(mapping_file).read_text())['/**']['ros__parameters']
    joint_ids = mapping['joint_ids']
    joint_names = mapping['joint_names']
    if not joint_ids or len(joint_ids) != len(joint_names) or len(set(joint_names)) != len(joint_names):
        raise ValueError('ID・関節名の対応不正')
    if any(type(id_value) is not int or not 0 <= id_value <= 252 for id_value in joint_ids):
        raise ValueError('サーボIDの範囲不正')
    for name in ('joint_scales', 'joint_offsets_deg'):
        values = mapping[name]
        if len(values) != len(joint_ids) or not all(math.isfinite(v) for v in values):
            raise ValueError('関節換算係数の設定不正: ' + name)
    state_topic = f'/{robot_name}/joint_states'
    viewer_topic = f'/{robot_name}/viewer_joint_states'
    input_topic = value('input_topic')
    actions = [Node(
        package='dynamixel_joint_state_bridge', executable='dynamixel_joint_state_bridge_node',
        name='dynamixel_current_pose', namespace=robot_name, output='screen',
        parameters=[mapping_file, {
            'input_topic': input_topic, 'output_topic': state_topic,
            # 既存の追加配信口をViewer実測表示へ固定。制御権要求・指令トピックへの接続なし
            'command_output_topic': viewer_topic, 'control_claim_topic': '',
        }])]
    if value('enable_reader').lower() == 'true':
        reader = Node(
            package='dynamixel_handler', executable='dynamixel_state_reader', output='screen',
            parameters=[{'device_name': value('device_name'), 'baudrate': int(value('baudrate')),
                         'publish_hz': float(value('reader_publish_hz')), 'output_topic': input_topic,
                         'joint_ids': sorted(set(joint_ids))}])
        actions[0:0] = [RegisterEventHandler(OnProcessExit(
            target_action=reader, on_exit=[EmitEvent(event=Shutdown(reason='実測角度リーダーの終了'))])), reader]
    if value('enable_viewer').lower() == 'true':
        share = Path(get_package_share_directory('gng_vlut_system'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/gng_viewer_bridge.launch.py')),
            launch_arguments={
                'params_file': params_file, 'joint_control_backend': 'external',
                # 実測入力の到着前だけに使用する初期表示
                'enable_joint_state_publisher': 'true',
            }.items()))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    bridge_share = Path(get_package_share_directory('dynamixel_joint_state_bridge'))
    defaults = {
        'params_file': str(share/'config/ToPoDualArm.yaml'),
        'mapping_file': str(bridge_share/'config/dynamixel_joint_state_bridge_ids_31_52.yaml'),
        'device_name': '/dev/ttyUSB0', 'baudrate': '1000000', 'reader_publish_hz': '30.0',
        'input_topic': '/dynamixel/state/present', 'enable_reader': 'true', 'enable_viewer': 'true',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])

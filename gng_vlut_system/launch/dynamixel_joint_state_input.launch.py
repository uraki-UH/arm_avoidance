"""リーダー・フォロワー実測トピックの共通起動。USB・実機指令なし。"""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.validate_full_topic_name import validate_full_topic_name
import yaml


sys.path.insert(0, str(Path(__file__).resolve().parent))
from launch_config import load_root_parameters


def resolve_path(value, package, directory='config'):
    if value.startswith('package://'):
        name, relative = value[len('package://'):].split('/', 1)
        path = Path(get_package_share_directory(name))/relative
    else:
        path = Path(value)
        if not path.is_file() and path.parent == Path('.'):
            path = Path(get_package_share_directory(package))/directory/path
    if not path.is_file():
        raise FileNotFoundError('実測入力設定が見つかりません: '+str(path))
    return path


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    params = load_root_parameters(resolve_path(value('params_file'), 'gng_vlut_system'))
    config = yaml.safe_load(resolve_path(value('config_file'), 'gng_vlut_system').read_text())
    bridge = Path(get_package_share_directory('dynamixel_joint_state_bridge'))/'config'
    urdf_path = resolve_path(value('urdf_path') or params['urdf_path'], 'gng_vlut_system')
    is_max_model = params['robot_name'] in ('topo_dual_arm_max', 'topo_dual_arm_max_long')
    follower_name = 'dynamixel_joint_state_bridge_max_ids_31_52.yaml' if is_max_model else 'dynamixel_joint_state_bridge_ids_31_52.yaml'
    default_mappings = {'leader': str(bridge/'dynamixel_joint_state_bridge.yaml'),
                        'follower': params.get('dynamixel_mapping_file') or str(bridge/follower_name)}
    actions, topics, buses = [], set(), {}
    for role in ('leader', 'follower'):
        setting = value('enable_'+role).lower()
        if setting not in ('true', 'false'):
            raise ValueError('enable_'+role+'にはtrueまたはfalseが必要です')
        if setting == 'false':
            continue
        role_config = dict(config[role])
        if value(role+'_output_topic'):
            role_config['output_topic'] = value(role+'_output_topic')
        input_type = (value('follower_input_type') if role == 'follower' else '') or role_config.pop('input_type', 'fresh')
        role_config.pop('input_type', None)
        if input_type not in ('fresh', 'present') or (role == 'leader' and input_type != 'fresh'):
            raise ValueError('リーダーにはfresh、フォロワーにはfreshまたはpresentが必要です')
        bus = role_config['driver_namespace'].rstrip('/')
        input_topic = role_config.pop('input_topic', bus+('/fresh_joint_states' if input_type == 'fresh' else '/state/present'))
        topic = role_config['output_topic']
        for name in (topic, input_topic):
            validate_full_topic_name(name)
        if topic in topics or topic == input_topic:
            raise ValueError('実測入力トピックの重複・折返しです')
        topics.add(topic)
        mapping_file = resolve_path(value(role+'_mapping_file') or role_config.pop('mapping_file', '') or default_mappings[role], 'dynamixel_joint_state_bridge')
        mapping = load_root_parameters(mapping_file)
        selected_names = role_config.get('joint_names')
        if selected_names and role_config.get('enable_gripper_input'):
            selected_names = list(selected_names)+['R_gripper_joint', 'L_gripper_joint']
        ids = {motor for motor, name in zip(mapping['joint_ids'], mapping['joint_names']) if not selected_names or name in selected_names}
        if buses.get(bus, set()) & ids:
            raise ValueError('同一バス内のリーダー・フォロワーID重複です')
        buses.setdefault(bus, set()).update(ids)
        if input_type == 'fresh':
            role_config['input_topic'] = input_topic
            actions.append(Node(package='gng_vlut_system', executable='dynamixel_joint_state_input.py',
                name=role+'_joint_state_input', namespace=value('robot_name') or params['robot_name'], output='screen',
                parameters=[role_config, {'urdf_path': str(urdf_path), 'mapping_file': str(mapping_file), 'role': role, 'use_sim_time': False}]))
        else:
            # 読取り専用readerとの既存互換。速度の仮値補完・実機制御入力への接続なし
            actions.append(Node(package='dynamixel_joint_state_bridge', executable='dynamixel_joint_state_bridge_node',
                name=role+'_joint_state_input', namespace=value('robot_name') or params['robot_name'], output='screen',
                parameters=[str(mapping_file), {'input_topic': input_topic, 'output_topic': topic,
                            'command_output_topic': '', 'control_claim_topic': '', 'use_sim_time': False}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    defaults = {'params_file': str(share/'config/topo_dual_arm_max_long.yaml'),
                'config_file': str(share/'config/dynamixel_joint_state_input.yaml'), 'urdf_path': '', 'robot_name': '',
                'leader_mapping_file': '', 'follower_mapping_file': '', 'enable_leader': 'true', 'enable_follower': 'true', 'follower_input_type': '', 'leader_output_topic': '', 'follower_output_topic': ''}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
                              OpaqueFunction(function=launch_setup)])

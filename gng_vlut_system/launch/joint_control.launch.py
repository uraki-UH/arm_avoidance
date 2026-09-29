"""部分関節指令の仲裁とViewer・Gazebo・実機への共通出力。"""
import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    params_file = value('params_file')
    params = yaml.safe_load(Path(params_file).read_text())['/**']['ros__parameters']
    urdf_path = value('urdf_path') or params['urdf_path']
    robot_name = value('robot_name') or params['robot_name']
    backend = value('backend')
    if backend not in ('viewer', 'gazebo', 'dynamixel'):
        raise ValueError('backendはviewer・gazebo・dynamixelのいずれかが必要です')
    module_path = Path(get_package_prefix('gng_vlut_system'))/'lib/gng_vlut_system/joint_command_model.py'
    spec = importlib.util.spec_from_file_location('joint_control_model', module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    model = module.joint_command_model(urdf_path)
    aliases = [(name, item) for name, item in model.aliases.items() if name != item[0]]
    source_timeouts = [0.0, 0.0, float(value('leader_timeout_sec'))]
    common = {'use_sim_time': value('use_sim_time').lower() == 'true'}
    mux_params = {
        **common, 'joint_names': list(model.joints),
        'command_timeout_sec': float(value('command_timeout_sec')),
        'source_topics': ['joint_commands', 'gripper_commands', 'leader_joint_states'],
        'source_priorities': [50, 200, 100], 'source_timeouts_sec': source_timeouts,
        'gripper_joint_names': [name for name in model.independent_names if 'gripper' in name],
    }
    if not mux_params['gripper_joint_names']:
        del mux_params['gripper_joint_names']
        mux_params['source_topics'] = ['joint_commands', 'leader_joint_states']
        mux_params['source_priorities'] = [50, 100]
        mux_params['source_timeouts_sec'] = [0.0, source_timeouts[2]]
    if aliases:
        mux_params.update(alias_names=[name for name, _ in aliases],
                          alias_parents=[item[0] for _, item in aliases],
                          alias_multipliers=[item[1] for _, item in aliases],
                          alias_offsets=[item[2] for _, item in aliases])
    mapping_file = value('mapping_file')
    input_setting = value('enable_dynamixel_input')
    enable_input = input_setting.lower() == 'true' if input_setting else backend == 'dynamixel'
    if (backend == 'dynamixel' or enable_input) and not mapping_file:
        mapping_file = str(Path(get_package_share_directory('dynamixel_joint_state_bridge'))/
                           'config/dynamixel_joint_state_bridge.yaml')
    output_params = {
        **common, 'backend': backend, 'urdf_path': urdf_path,
        'state_topic': value('state_topic'), 'viewer_topic': value('viewer_topic'),
        'trajectory_topic': value('trajectory_topic'), 'dynamixel_topic': value('dynamixel_topic'),
        'mapping_file': mapping_file, 'max_joint_velocity': float(value('max_joint_velocity')),
        'max_state_age_sec': float(value('max_state_age_sec')),
        'max_command_age_sec': float(value('max_command_age_sec')),
        'enable_direct_tracking': value('enable_direct_tracking').lower() == 'true',
    }
    actions = [
        Node(package='gng_vlut_system', executable='joint_state_mux_node', namespace=robot_name,
             output='screen', parameters=[mux_params]),
        Node(package='gng_vlut_system', executable='joint_command_output.py', namespace=robot_name,
             output='screen', parameters=[output_params]),
    ]
    if enable_input:
        actions.append(Node(package='dynamixel_joint_state_bridge',
            executable='dynamixel_joint_state_bridge_node', namespace=robot_name,
            name='joint_control_dynamixel_input', output='screen', parameters=[mapping_file, {
                'input_topic': value('dynamixel_input_topic'),
                'output_topic': value('state_topic') if backend == 'dynamixel' else 'leader_measured_joint_states',
                'command_output_topic': '' if backend == 'dynamixel' else 'leader_joint_states',
                'control_claim_topic': 'control_claims', 'control_claim_priority': 100,
            }]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    defaults = {'params_file': str(share/'config/ToPoDualArm.yaml'), 'urdf_path': '', 'robot_name': '',
                'backend': 'viewer', 'use_sim_time': 'false', 'state_topic': 'joint_states',
                'viewer_topic': 'viewer_joint_states',
                'trajectory_topic': 'dual_arm_controller/joint_trajectory',
                'dynamixel_topic': '/dynamixel/command/goal', 'mapping_file': '',
                'enable_dynamixel_input': '', 'dynamixel_input_topic': '/dynamixel/state/present',
                'command_timeout_sec': '1.0', 'leader_timeout_sec': '0.5',
                'max_joint_velocity': '0.6', 'max_state_age_sec': '1.0',
                'max_command_age_sec': '0.5', 'enable_direct_tracking': 'false'}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])

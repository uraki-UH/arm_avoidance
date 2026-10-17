"""共通ROS回避と実行先の接続。bag・実センサの起動とブラウザ表示は独立。"""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from launch_config import resolve_package_path


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    backend = value('execution_backend')
    if backend not in ('external', 'mujoco', 'dynamixel'):
        raise ValueError('execution_backendはexternal・mujoco・dynamixelが必要です')
    params_file = Path(resolve_package_path(value('params_file'), get_package_share_directory))
    if not params_file.is_file() and params_file.parent == Path('.'):
        params_file = Path(get_package_share_directory('gng_vlut_system'))/'config'/params_file
    params = yaml.safe_load(params_file.read_text())['/**']['ros__parameters']
    config = params.get('avoidance', {})
    root = Path(value('simulator_root')) if value('simulator_root') else Path(params['urdf_path']).parents[2]/'ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator'
    script_dir = root/'integrations/ros2'
    if not (script_dir/'gng_ros.py').is_file():
        raise FileNotFoundError('共通回避の実装がありません。simulator_rootを指定してください: '+str(root))
    state_topic = value('state_topic') or ('/'+params['robot_name']+'/joint_states' if backend == 'dynamixel' else config.get('state_topic', '/sim/joint_states'))
    goal_topic = value('goal_topic') or config.get('goal_topic', '/avoidance/goal')
    command_topic = value('command_topic') or config.get('command_topic', '/avoidance/joint_command')
    controller = ExecuteProcess(cmd=[sys.executable, str(script_dir/'gng_ros.py'), '--params-file', str(params_file),
        '--model', value('model'), '--state-topic', state_topic, '--goal-topic', goal_topic,
        '--command-topic', command_topic, '--source-tf-topic', value('source_tf_topic'),
        '--source-static-tf-topic', value('source_static_tf_topic'), '--input-kind',
        value('input_kind') or config.get('settings', {}).get('input_kind', 'pointcloud')], output='screen')
    processes = [controller]
    actions = [LogInfo(msg=f'共通ROS回避: {backend} / 実測 {state_topic} / 指令 {command_topic}'), controller]
    if backend == 'mujoco':
        simulator = ExecuteProcess(cmd=[sys.executable, str(script_dir/'simulate.py'), '--avoidance', 'ros',
            '--params-file', str(params_file), '--model', value('model'), '--pose', 'forward', '--duration', '0',
            '--state-topic', state_topic, '--command-topic', command_topic], output='screen')
        actions.append(simulator)
        processes.append(simulator)
    if value('allow_hardware_output') not in ('true', 'false'):
        raise ValueError('allow_hardware_outputはtrue・falseが必要です')
    if backend == 'dynamixel' and value('allow_hardware_output') == 'true':
        output = Node(package='gng_vlut_system', executable='joint_command_output.py',
            name='avoidance_output', namespace='avoidance', output='screen', parameters=[{
                'backend': 'dynamixel', 'urdf_path': params['urdf_path'], 'state_topic': state_topic,
                'target_topic': command_topic, 'target_message_type': 'joint_trajectory',
                'mapping_file': resolve_package_path(params['dynamixel_mapping_file'], get_package_share_directory),
                'max_joint_velocity': 3., 'max_state_age_sec': float(config.get('max_state_age_sec', .5)),
                'dynamixel_topic': value('dynamixel_topic')}])
        actions.append(output)
        processes.append(output)
    elif backend == 'dynamixel':
        actions.append(LogInfo(msg='実機出力OFF。共通の回避指令トピックまでを配信'))
    for process in processes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='回避計算・実行先プロセスの終了'))])))
    return actions


def generate_launch_description():
    defaults = dict(params_file='topo_dual_arm_max_long.yaml', model='long', execution_backend='external',
        state_topic='', goal_topic='', command_topic='', simulator_root='', source_tf_topic='/tf', input_kind='',
        source_static_tf_topic='/tf_static', allow_hardware_output='false', dynamixel_topic='/dynamixel/command/goal')
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
                              OpaqueFunction(function=launch_setup)])

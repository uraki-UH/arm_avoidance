"""タスク実行器とGazebo Harmonicの一括起動。"""
from pathlib import Path
import json
import os
import sys

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


package_dir = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(package_dir / 'launch'))
from dual_arm_effort_config import default_urdf

script_dir = package_dir / 'scripts'
if not script_dir.is_dir():
    from ament_index_python.packages import get_package_prefix
    script_dir = Path(get_package_prefix('gng_vlut_system')) / 'lib/gng_vlut_system'
sys.path.insert(0, str(script_dir))
from task_program import load_program, read_joint_bounds
from harmonic_launcher import make_request


def relay_exited(event, context):
    if event.returncode != 0 and not context.is_shutdown:
        raise RuntimeError('Harmonicの起動・接続に失敗しました。直前の接続ログを確認してください')
    return [EmitEvent(event=Shutdown(reason='Harmonic接続の終了'))]


def launch_setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if os.environ.get('ROS_DISTRO') == 'humble':
        names = ('urdf', 'task_file', 'namespace', 'scenario', 'gui', 'output_dir', 'enable_autostart')
        request = make_request({name: value(name) for name in names}, package_dir.parent,
                               os.environ.get('ROS_DOMAIN_ID', '0'))
        relay = ExecuteProcess(cmd=[sys.executable, str(script_dir / 'harmonic_launcher.py'),
            'connect', '--socket', value('launcher_socket'), '--request', json.dumps(request)],
            output='screen', sigterm_timeout='25', sigkill_timeout='5')
        return [RegisterEventHandler(OnProcessExit(target_action=relay, on_exit=relay_exited)), relay]
    # Gazebo起動前の全設定検査。実行器側でも同じ検査を実施。
    load_program(yaml.safe_load(Path(value('task_file')).read_text()), read_joint_bounds(value('urdf')))
    executor = ExecuteProcess(cmd=[sys.executable, str(script_dir / 'task_executor.py'), '--ros-args',
                                  '-r', '__ns:=/' + value('namespace'), '-p', 'use_sim_time:=true',
                                  '-p', 'task_file:=' + value('task_file'), '-p', 'urdf:=' + value('urdf'),
                                  '-p', 'enable_autostart:=' + value('enable_autostart')], output='screen')
    simulator = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_dir / 'launch/dual_arm_gz.launch.py')),
        launch_arguments={name: value(name) for name in ('urdf', 'namespace', 'scenario', 'gui', 'output_dir')}.items())
    return [RegisterEventHandler(OnProcessExit(target_action=executor,
                on_exit=[EmitEvent(event=Shutdown(reason='タスク実行器の終了'))])), simulator, executor]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=str(default_urdf())),
        DeclareLaunchArgument('task_file', default_value=str(package_dir / 'config/simulation/task_program.yaml')),
        DeclareLaunchArgument('namespace', default_value='sim_topo_dual_arm_max_long'),
        DeclareLaunchArgument('scenario', default_value='empty'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('enable_autostart', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launcher_socket', default_value=os.environ.get('GNG_HARMONIC_SOCKET',
            str(package_dir.parent / 'artifacts/harmonic_launcher/launcher.sock'))),
        OpaqueFunction(function=launch_setup),
    ])

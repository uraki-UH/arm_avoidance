"""機体設定で切り替えるGazebo点群・自己除去・VLUT・GNG回避。"""
import importlib.util
import os
from pathlib import Path
import shutil
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    share = Path(get_package_share_directory('gng_vlut_system'))
    spec = importlib.util.spec_from_file_location('pointcloud_config', share / 'launch/pointcloud_avoidance_config.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pose_arg = LaunchConfiguration('camera_pose').perform(context)
    params_path, params, config = module.load_config(LaunchConfiguration('robot_config').perform(context),
        LaunchConfiguration('input_config').perform(context), yaml.safe_load(pose_arg) if pose_arg else None)
    namespace = 'sim_' + params['robot_name']
    enable_keyboard = LaunchConfiguration('enable_keyboard').perform(context).lower() == 'true'
    if enable_keyboard and not sys.stdin.isatty():
        raise ValueError('キー操作には対話TTYが必要です。Dockerではexec -itを使用してください')
    run_dir = Path(tempfile.mkdtemp(prefix='pointcloud_avoidance_'))
    avoidance_path, demo_path = run_dir / 'avoidance.yaml', run_dir / 'demo.yaml'
    avoidance_path.write_text(yaml.safe_dump({'dual_arm_avoidance_demo': config}, allow_unicode=True))
    demo = {'namespace': namespace, 'enable_gui': True, 'enable_auto_start': False,
            'enable_viewer': LaunchConfiguration('enable_viewer').perform(context).lower() == 'true',
            'motor_position_gain': 60.0, 'motor_limit_scale': 0.95,
            'initial_joint_positions': config.get('initial_joint_positions', {})}
    demo_path.write_text(yaml.safe_dump({'dual_arm_gazebo_demo': demo}))
    actions = [
        LogInfo(msg=f'点群回避: {namespace}, 計画グループ: ' + ', '.join(group['name'] for group in config['planning_groups'])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: (shutil.rmtree(run_dir, ignore_errors=True), [])[1])])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/dual_arm_gazebo_demo.launch.py')),
            launch_arguments={'params_file': str(params_path), 'demo_config': str(demo_path),
                'avoidance_config': str(avoidance_path), 'enable_integrated_control': 'true',
                'enable_auto_start': 'false', 'gui': LaunchConfiguration('gui'),
                'gazebo_master_uri': LaunchConfiguration('gazebo_master_uri')}.items())]
    if enable_keyboard:
        keyboard = Node(package='gng_vlut_system', executable='dual_arm_control_keyboard.py', output='log',
                        arguments=['--namespace', namespace, '--tty-path', os.ttyname(sys.stdin.fileno())])
        actions.extend([keyboard, RegisterEventHandler(OnProcessExit(target_action=keyboard,
            on_exit=[EmitEvent(event=Shutdown(reason='操作端末の終了'))]))])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    defaults = {'robot_config': str(share / 'config/pointcloud_avoidance_topodualarm.yaml'),
                'input_config': '', 'camera_pose': '',
                'gui': 'true', 'enable_keyboard': 'true', 'enable_viewer': 'true',
                'gazebo_master_uri': 'http://127.0.0.1:11355'}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup)])

"""外部Isaac Simの標準Controller Managerへの接続とTF配信。"""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dual_arm_effort_config import validate_namespace


def launch_setup(context):
    namespace = validate_namespace(LaunchConfiguration('namespace').perform(context))
    return [
        # 合成済みURDFからのTF。状態と形状の配信元はIsaacのみ
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
             parameters=[{'use_sim_time': True, 'use_robot_description_topic': True}], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'dual_arm_controller', '-c', '/'+namespace+'/controller_manager',
                        '--controller-manager-timeout', '300'], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='sim_topo_dual_arm_max_long'),
        OpaqueFunction(function=launch_setup)])

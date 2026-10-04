"""制御経路から独立した関節運動状態の観測。既定はノード未起動。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    if not IfCondition(LaunchConfiguration('enable_joint_motion_state')).evaluate(context):
        return []
    value = lambda name: LaunchConfiguration(name).perform(context)
    return [Node(
        package='gng_vlut_system', executable='joint_motion_observer.py',
        namespace=value('namespace'), output='screen', parameters=[{
            'state_topic': value('state_topic'), 'motion_state_topic': value('motion_state_topic'),
            'use_sim_time': IfCondition(LaunchConfiguration('use_sim_time')).evaluate(context),
            'max_derivative_order': int(value('max_derivative_order')),
            'min_sample_period_sec': float(value('min_sample_period_sec')),
            'max_sample_gap_sec': float(value('max_sample_gap_sec')),
        }])]


def generate_launch_description():
    defaults = {'enable_joint_motion_state': 'false', 'namespace': '',
                'state_topic': 'joint_states', 'motion_state_topic': 'joint_motion_states',
                'use_sim_time': 'false', 'max_derivative_order': '3',
                'min_sample_period_sec': '0.0001', 'max_sample_gap_sec': '0.5'}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup)])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    target_topic = LaunchConfiguration("target_topic").perform(context)
    state_topic = LaunchConfiguration("state_topic").perform(context)
    command_topic = LaunchConfiguration("command_topic").perform(context)
    publish_hz = LaunchConfiguration("publish_hz").perform(context)
    max_joint_velocity = LaunchConfiguration("max_joint_velocity").perform(context)
    position_tolerance = LaunchConfiguration("position_tolerance").perform(context)
    use_wraparound = LaunchConfiguration("use_wraparound").perform(context)

    node_params = {
        "target_topic": target_topic,
        "state_topic": state_topic,
        "command_topic": command_topic,
        "publish_hz": float(publish_hz),
        "max_joint_velocity": float(max_joint_velocity),
        "position_tolerance": float(position_tolerance),
        "use_wraparound": use_wraparound.lower() in ("1", "true", "yes", "on"),
    }

    return [
        Node(
            package="gng_vlut_system",
            executable="target_joint_state_executor_node",
            name="target_joint_state_executor_node",
            namespace=robot_name,
            output="screen",
            parameters=[node_params],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("target_topic", default_value="target_joint_states"),
        DeclareLaunchArgument("state_topic", default_value="joint_states"),
        DeclareLaunchArgument("command_topic", default_value="joint_commands"),
        DeclareLaunchArgument("publish_hz", default_value="50.0"),
        DeclareLaunchArgument("max_joint_velocity", default_value="0.6"),
        DeclareLaunchArgument("position_tolerance", default_value="0.01"),
        DeclareLaunchArgument("use_wraparound", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])

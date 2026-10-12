#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("dynamixel_joint_state_bridge"), "config", "dynamixel_joint_state_bridge.yaml"]
    )

    node = Node(
        package="dynamixel_joint_state_bridge",
        executable="dynamixel_joint_state_bridge_node",
        namespace=namespace,
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="", description="Namespace for joint_states"),
            DeclareLaunchArgument("params_file", default_value=default_params_file, description="Parameter file"),
            node,
        ]
    )

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("mode", default_value="generic", description="generic or topoarm_udp"),
        DeclareLaunchArgument("joint_state_topic", default_value="/joint_states_real"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),

        # 1. Generic Robot Bridge (New high-performance bridge)
        Node(
            package="gng_vlut_system",
            executable="robot_bridge_node",
            name="robot_bridge",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == 'generic'"])),
            parameters=[{
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "base_frame": LaunchConfiguration("base_frame")
            }]
        ),

        # 2. Topoarm Legacy UDP Bridge (Specific for existing hardware)
        Node(
            package="gng_vlut_system",
            executable="topoarm_udp_bridge",
            name="topoarm_udp_bridge",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == 'topoarm_udp'"])),
            parameters=[{
                "output_topic": LaunchConfiguration("joint_state_topic"),
                "base_frame": LaunchConfiguration("base_frame")
            }]
        )
    ])
